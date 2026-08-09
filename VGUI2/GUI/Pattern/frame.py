import tkinter as tk
from tkinter import ttk
from typing import Callable, List, Optional, Tuple
import os
import re
import math

from PIL import Image, ImageTk

from GUI.base_frame import BaseFrame

from GUI.Pattern.constants import MARKER_ICON_SIZE, AREA_COLOR, PREVIEW_COLOR, START_COLOR, END_COLOR
from GUI.Pattern.patterns import PATTERNS, PARAM_SPECS, generate_lawnmower, generate_expanding_square
from GUI.Pattern.geometry import (polygon_area_m2, find_self_intersection, haversine_distance)

base_path = os.path.join(os.path.dirname(__file__), "..", "..")

def _format_area(area_m2: float) -> str:
    if area_m2 < 1000000:
        return f"{area_m2:,.0f} m\u00b2"

    return f"{area_m2 / 1000000:,.2f} km\u00b2"

def _remove_duplicates_consec(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    result: List[Tuple[float, float]] = []

    for p in points:
        if not result:
            result.append(p)

        else:
            # ~11cm tolerance to remove almost identical points
            dist = math.hypot(p[0] - result[-1][0], p[1] - result[-1][1])

            if dist > 1e-6:
                result.append(p)

    return result

class PatternPlannerFrame(BaseFrame):

    def __init__(self, parent, theme, ctrl, map_frame, on_close: Callable[[], None], on_apply_route: Callable[[List[Tuple[float, float]]], None]):
        self.map_frame = map_frame
        self.map_widget = map_frame.widget
        self.on_close = on_close
        self.on_apply_route = on_apply_route

        self._area_points: List[Tuple[float, float]] = []

        self._preview_waypoints: List[Tuple[float, float]] = []
        self._preview_path = None
        self._start_marker = None
        self._end_marker = None

        self._area_path = None
        self._area_markers = []
        self._param_vars = {}
        self._refresh_job = None
        self._status_kind = None
        self._refreshing_preview  = False

        self._area_icon = self._load_icon("orange_dot.png")
        self._start_icon = self._load_icon("green_dot.png")
        self._end_icon = self._load_icon("red_dot.png")

        super().__init__(parent, theme, ctrl)

    @staticmethod
    def _load_icon(filename: str) -> Optional[ImageTk.PhotoImage]:
        try:
            icon_path = os.path.join(base_path, "Icons", filename)
            raw = Image.open(icon_path).convert("RGBA")
        except Exception:
            print(f"Failed to load icon: {filename}")
            return None

        raw.thumbnail((MARKER_ICON_SIZE, MARKER_ICON_SIZE), Image.LANCZOS)
        canvas = Image.new("RGBA", (MARKER_ICON_SIZE, MARKER_ICON_SIZE), (0, 0, 0, 0))
        canvas.paste(raw, ((MARKER_ICON_SIZE - raw.width) // 2, (MARKER_ICON_SIZE - raw.height) // 2), raw)

        return ImageTk.PhotoImage(canvas)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Pattern Planner",
            font = ("Segoe UI", 10, "bold"), padx = 6, pady = 6,
            bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "both", expand = True)

        # Header / close
        header = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        header.pack(fill = "x", pady = (0, 6))
        self._header = header

        ttk.Button(header, text = "Close", command = self._on_close_click).pack(side = "right")

        # Pattern type
        self.pattern_box = self._make_section(self.frame, "PATTERN")

        type_row = tk.Frame(self.pattern_box, bg = self.theme["panel_bg"])
        type_row.pack(fill = "x", padx = 6, pady = (0, 6))
        self._type_row = type_row

        self.pattern_var = tk.StringVar(value = "Lawnmower")
        self.pattern_combo = ttk.Combobox(
            type_row, textvariable = self.pattern_var, values = list(PATTERNS.keys()),
            state = "readonly", width = 18)
        self.pattern_combo.pack(side = "left")
        self.pattern_combo.bind("<<ComboboxSelected>>", lambda _e: self._rebuild_params())

        # Search area
        self.area_box = self._make_section(self.frame, "AREA")

        area_status_row = tk.Frame(self.area_box, bg = self.theme["panel_bg"])
        area_status_row.pack(fill = "x", padx = 6, pady = (0, 4))
        self._area_status_row = area_status_row

        self.area_label = tk.Label(area_status_row, text = "0 defined", font = ("Consolas", 11),
                                   bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.area_label.pack(side = "left", padx = 6)

        self.area_size_label = tk.Label(area_status_row, text = "", font = ("Consolas", 11),
                                        bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.area_size_label.pack(side = "left", padx = (4, 0))

        area_btns = tk.Frame(self.area_box, bg = self.theme["panel_bg"])
        area_btns.pack(fill = "x", padx = 6, pady = (0, 6))
        self._area_btns = area_btns

        ttk.Button(area_btns, text = "Undo", command = self._undo_point).pack(side = "left", padx = (0, 4))
        ttk.Button(area_btns, text = "Clear", command = self._clear_area).pack(side = "left")

        self.param_box = self._make_section(self.frame, "PARAMETERS")

        self.param_container = tk.Frame(self.param_box, bg = self.theme["panel_bg"])
        self.param_container.pack(fill = "x", padx = 6, pady = (0, 6))

        # Status label
        self.status_label = tk.Label(self.frame, text = "", font = ("Segoe UI", 11),
                                     anchor = "w", wraplength = 340, justify = "left",
                                     bg = self.theme["panel_bg"], fg = self.theme["red"])
        self.status_label.pack(fill = "x", pady = (0, 8))

        # Action buttons
        action_row = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        action_row.pack(fill = "x", side = "bottom")
        self._action_row = action_row

        ttk.Button(action_row, text = "Add to Route", command = lambda: self._on_apply(overwrite = False)).pack(side = "left", fill = "x", expand = True, padx = (0, 2))
        ttk.Button(action_row, text = "Overwrite Route", command = lambda: self._on_apply(overwrite = True)).pack(side = "right", fill = "x", expand = True, padx = (2, 0))

        self._style_sliders()
        self._rebuild_params()
        self._begin_picking()

    def _style_sliders(self) -> None:
        style = ttk.Style()
        if style.theme_use() != "clam":
            try:
                style.theme_use("clam")
            except tk.TclError:
                pass

        style.configure("Pattern.Horizontal.TScale",
                        background = self.theme["panel_bg"],
                        troughcolor = self.theme["canvas_bg"],
                        bordercolor = self.theme["canvas_bg"],
                        lightcolor = self.theme["accent"],
                        darkcolor = self.theme["accent"],
                        troughrelief = "flat",
                        sliderrelief = "flat",
                        sliderthickness = 14,
                        sliderlength = 14,
                        borderwidth = 0)
        style.map("Pattern.Horizontal.TScale",
                 background = [("active", self.theme["panel_bg"])],
                 lightcolor = [("active", self.theme["accent"])],
                 darkcolor = [("active", self.theme["accent"])])
        
    def _make_section(self, parent, title: str) -> tk.Frame:
        box = tk.Frame(parent, bg = self.theme["panel_bg"], bd = 1,
                       relief = "solid", highlightbackground = self.theme["border"])
        box.pack(fill = "x", pady = (0, 8))

        header_row = tk.Frame(box, bg = self.theme["panel_bg"])
        header_row.pack(fill = "x", padx = 6, pady = (5, 3))

        title_label = tk.Label(header_row, text = title, font = ("Segoe UI", 8, "bold"),
                               bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        title_label.pack(side = "left")

        box._header_row = header_row
        box._title_label = title_label

        return box

    def _rebuild_params(self) -> None:
        for child in self.param_container.winfo_children():
            child.destroy()
        self._param_vars = {}

        for key in PATTERNS[self.pattern_var.get()]:
            spec = PARAM_SPECS[key]
            row = tk.Frame(self.param_container, bg = self.theme["panel_bg"])
            row.pack(fill = "x", pady = 2)

            if spec["slider"]:
                self._build_slider_row(row, key, spec)
            else:
                self._build_entry_row(row, key, spec)

        self._clear_status()
        self._schedule_refresh()

    def _build_entry_row(self, row: tk.Frame, key: str, spec: dict) -> None:
        tk.Label(row, text = spec["label"], font = ("Segoe UI", 9), width = 22, anchor = "w",
                 bg = self.theme["panel_bg"], fg = self.theme["fg"]).pack(side = "left")

        var = tk.StringVar(value = spec["default"])
        var.trace_add("write", self._schedule_refresh)
        ttk.Entry(row, textvariable = var, width = 10).pack(side = "left")

        self._param_vars[key] = var

    def _build_slider_row(self, row: tk.Frame, key: str, spec: dict) -> None:
        tk.Label(row, text = spec["label"], font = ("Segoe UI", 9), width = 22, anchor = "w",
                 bg = self.theme["panel_bg"], fg = self.theme["fg"]).pack(side = "left")

        value_var = tk.StringVar(value = spec["fmt"].format(spec["default"]))

        unit_label = tk.Label(row, text = spec.get("unit", ""), font = ("Segoe UI", 12),
                              bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        unit_label.pack(side = "right", padx = (6, 0))
        unit_label._is_unit_label = True

        value_entry = tk.Entry(row, textvariable = value_var, width = 5, font = ("Segoe UI", 12),
                               justify = "right", bg = self.theme["canvas_bg"], fg = self.theme["fg"],
                               insertbackground = self.theme["fg"], relief = "flat")
        value_entry.pack(side = "right")

        var = tk.DoubleVar(value = spec["default"])

        def _on_move(raw_value, var = var, spec = spec, value_var = value_var) -> None:
            snapped = round(float(raw_value) / spec["step"]) * spec["step"]
            var.set(snapped)
            value_var.set(spec["fmt"].format(snapped))
            self._immediate_refresh()

        def _on_entry_commit(_event = None, var = var, spec = spec, value_var = value_var, value_entry = value_entry) -> None:
            # Pull the number back out of whatever was typed
            match = re.search(r"-?\d+(?:\.\d+)?", value_var.get())
            if match is None:
                value_var.set(spec["fmt"].format(var.get()))
                value_entry.config(bg = self.theme["red"])
                self.frame.after(400, lambda: value_entry.config(bg = self.theme["canvas_bg"]))
                return

            raw = float(match.group())
            clamped = max(spec["min"], min(spec["max"], raw))
            snapped = round(clamped / spec["step"]) * spec["step"]

            var.set(snapped)
            value_var.set(spec["fmt"].format(snapped))
            self._immediate_refresh()

        value_entry.bind("<Return>", _on_entry_commit)
        value_entry.bind("<FocusOut>", _on_entry_commit)

        scale = ttk.Scale(row, from_ = spec["min"], to = spec["max"], orient = "horizontal",
                          variable = var, command = _on_move, style = "Pattern.Horizontal.TScale")
        scale.pack(side = "left", fill = "x", expand = True, padx = (0, 6))

        self._param_vars[key] = var

    def _read_params(self) -> dict:
        values = {}

        for key, var in self._param_vars.items():
            if isinstance(var, tk.StringVar):
                raw = var.get().strip()
                try:
                    values[key] = float(raw)
                except ValueError:
                    raise ValueError(f"'{raw}' is not a valid number")
            else:
                values[key] = float(var.get())

        if "num_legs" in values:
            values["num_legs"] = int(values["num_legs"])

        return values

    # Route map clicks to the area for as long as this panel is open
    def _begin_picking(self) -> None:
        self._set_status("Double-click the map to add area points.", "info")
        self.map_frame.request_point(self._on_area_point_picked)

    def _on_area_point_picked(self, coords: Tuple[float, float]) -> None:
        self._area_points.append(coords)
        self._update_area_label()
        self._draw_area()
        self._schedule_refresh()

        # Stay in picking mode, re-arm for the next click
        self.map_frame.request_point(self._on_area_point_picked)

    def _undo_point(self) -> None:
        if self._area_points:
            self._area_points.pop()
            self._update_area_label()
            self._draw_area()
            self._schedule_refresh()

    def _clear_area(self) -> None:
        if not self._area_points:
            return

        self._area_points = []
        self._clear_area_map_items()
        self._update_area_label()
        self._schedule_refresh()

    def _update_area_label(self) -> None:
        n = len(self._area_points)
        text = "1 Point" if n == 1 else f"{n} Points"
        if n and n < 3:
            text += " (3+ Required)"
        self.area_label.config(text = text, fg = self.theme["fg"] if n else self.theme["fg_dim"])

        if n >= 3:
            area_m2 = polygon_area_m2(self._area_points)
            self.area_size_label.config(text = f"/  {_format_area(area_m2)}", fg = self.theme["fg"])
        else:
            self.area_size_label.config(text = "")

    def _clear_area_map_items(self) -> None:
        for m in self._area_markers:
            m.delete()
        self._area_markers.clear()

        if self._area_path is not None:
            self._area_path.delete()
            self._area_path = None

    def _draw_area(self) -> None:
        self._clear_area_map_items()

        for point in self._area_points:
            self._area_markers.append(
                self.map_widget.set_marker(point[0], point[1], icon = self._area_icon, text = "",
                                           text_color = self.theme[AREA_COLOR]))

        if len(self._area_points) > 2:
            closed_loop = self._area_points + [self._area_points[0]]
            self._area_path = self.map_widget.set_path(
                closed_loop, color = self.theme[AREA_COLOR], width = 2)

    def _generate(self) -> List[Tuple[float, float]]:
        if len(self._area_points) < 3:
            raise ValueError("Define an area with at least 3 points first")

        bad_point = find_self_intersection(self._area_points)
        if bad_point is not None:
            raise ValueError("Area boundary crosses itself")

        params = self._read_params()
        pattern = self.pattern_var.get()

        if pattern == "Lawnmower":
            waypoints = generate_lawnmower(self._area_points, params["spacing_m"], params["orientation_deg"])
        elif pattern == "Expanding Square":
            waypoints = generate_expanding_square(self._area_points, params["initial_leg_m"],
                                                   params["num_legs"], params["orientation_deg"])
        else:
            raise ValueError(f"Unknown pattern: {pattern}")

        return _remove_duplicates_consec(waypoints)

    # Debounced auto-refresh
    def _schedule_refresh(self, *_args) -> None:
        if not hasattr(self, "frame"):
            return

        if self._refresh_job is not None:
            self.frame.after_cancel(self._refresh_job)

        self._refresh_job = self.frame.after(120, self._auto_refresh)

    # Skip the debounce entirely, used by the sliders so the preview tracks
    # the handle live while dragging instead of lagging behind
    def _immediate_refresh(self) -> None:
        if self._refresh_job is not None:
            self.frame.after_cancel(self._refresh_job)
            self._refresh_job = None
        self._auto_refresh()

    def _auto_refresh(self) -> None:
        if getattr(self, "_refreshing_preview", False):
            return

        self._refreshing_preview = True
        try:
            self._refresh_job = None
            self._refresh_preview(show_incomplete_error = False)

        finally:
            self._refreshing_preview  = False

    def _refresh_preview(self, show_incomplete_error: bool) -> None:
        try:
            waypoints = self._generate()
        except ValueError as e:
            self._preview_waypoints = []
            self._clear_preview_map_items()

            if show_incomplete_error or len(self._area_points) >= 3:
                self._set_status(str(e), "error")
            else:
                self._clear_status()
            return

        self._preview_waypoints = waypoints
        self._draw_preview()

        length_m = sum(haversine_distance(a, b) for a, b in zip(waypoints, waypoints[1:]))
        length_text = f"{length_m:.0f} m" if length_m < 1000 else f"{length_m / 1000:.2f} km"
        self._set_status(f"Previewing {len(waypoints)} waypoints, {length_text} track length.", "ok")

    def _draw_preview(self) -> None:
        self._clear_preview_map_items()

        if len(self._preview_waypoints) > 1:
            self._preview_path = self.map_widget.set_path(
                self._preview_waypoints, color = self.theme[PREVIEW_COLOR], width = 3)
            
            start_lat, start_lon = self._preview_waypoints[0]
            self._start_marker = self.map_widget.set_marker(
                start_lat, start_lon, icon = self._start_icon, text = "Start",
                text_color = self.theme[START_COLOR])

            end_lat, end_lon = self._preview_waypoints[-1]
            self._end_marker = self.map_widget.set_marker(
                end_lat, end_lon, icon = self._end_icon, text = "End",
                text_color = self.theme[END_COLOR])

    def _clear_preview_map_items(self) -> None:
        if self._start_marker is not None:
            self._start_marker.delete()
            self._start_marker = None

        if self._end_marker is not None:
            self._end_marker.delete()
            self._end_marker = None

        if self._preview_path is not None:
            self._preview_path.delete()
            self._preview_path = None

    def clear_preview(self) -> None:
        self._clear_preview_map_items()
        self._clear_area_map_items()

    def on_overlay_close(self) -> None:
        self.map_frame.request_point(None)
        self.clear_preview()

    def _on_apply(self, overwrite: bool) -> None:
        if not self._preview_waypoints:
            try:
                self._preview_waypoints = self._generate()
            except ValueError as e:
                self._set_status(str(e), "error")
                return

        self.on_apply_route(self._preview_waypoints, overwrite)
        self._on_close_click()

    def _on_close_click(self) -> None:
        self.on_close()

    def _set_status(self, text: str, kind: str) -> None:
        self._status_kind = kind
        color = {
            "error": self.theme["red"],
            "ok": self.theme["green"],
            "info": self.theme["accent"],
        }.get(kind, self.theme["fg_dim"])
        self.status_label.config(text = text, fg = color)

    def _clear_status(self) -> None:
        self._status_kind = None
        self.status_label.config(text = "")

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])

        for widget in (self._header, self.param_container, self._action_row):
            widget.config(bg = theme["panel_bg"])

        for box, extra_rows in (
            (self.pattern_box, [self._type_row]),
            (self.area_box, [self._area_status_row, self._area_btns]),
            (self.param_box, []),
        ):
            box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
            box._header_row.config(bg = theme["panel_bg"])
            box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
            for row in extra_rows:
                row.config(bg = theme["panel_bg"])

        self._style_sliders()

        for row in self.param_container.winfo_children():
            row.config(bg = theme["panel_bg"])
            for child in row.winfo_children():
                if isinstance(child, tk.Label):
                    if getattr(child, "_is_unit_label", False):
                        child.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
                    else:
                        child.config(bg = theme["panel_bg"], fg = theme["fg"])
                elif isinstance(child, tk.Entry):
                    child.config(bg = theme["canvas_bg"], fg = theme["fg"],
                                insertbackground = theme["fg"])

        self.area_label.config(bg = theme["panel_bg"],
                               fg = theme["fg"] if self._area_points else theme["fg_dim"])
        self.area_size_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        status_color = {
            "error": theme["red"],
            "ok": theme["green"],
            "info": theme["accent"],
        }.get(self._status_kind, theme["fg_dim"])
        self.status_label.config(bg = theme["panel_bg"], fg = status_color)

        if self._preview_waypoints:
            self._draw_preview()

        if self._area_points:
            self._draw_area()