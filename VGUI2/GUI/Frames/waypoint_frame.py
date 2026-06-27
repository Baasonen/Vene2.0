import tkinter as tk
from tkinter import ttk
from typing import Tuple
import os

from GUI.base_frame import BaseFrame
from vcom.protocol import uploadStatus

base_path = os.path.join(os.path.dirname(__file__), "..", "..")

class WaypointFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, map_widget):
        self.map_widget = map_widget
        self.waypoints: list[Tuple[float, float]] = []
        self._markers = []
        self._path = None
        self._route_changed = True
        self._prev_status = uploadStatus.IDLE

        try:
            icon_path = os.path.join(base_path, "icons", "wp_icon.png")
            self._wp_icon = tk.PhotoImage(file = icon_path)
        except Exception:
            self._wp_icon = None
            print("Unable to load WP icon")

        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Waypoints",
            font = ("Segoe UI", 10, "bold"), padx = 10, pady = 8,
            bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "x", pady = (0, 10))

        self.list_container = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.list_container.pack(fill = "x", pady = 2)

        self.listbox = tk.Listbox(self.list_container, height = 20, font = ("Consolas", 9),
                                  borderwidth = 0, highlightthickness = 0, selectmode = "single",
                                  bg = self.theme["canvas_bg"], fg = self.theme["fg"],
                                  selectbackground = self.theme["accent"])
        self.listbox.pack(side = "left", fill = "x", expand = True)

        scrollbar = ttk.Scrollbar(self.list_container, orient = "vertical", command = self.listbox.yview)
        scrollbar.pack(side = "left", fill = "y")
        self.listbox.config(yscrollcommand = scrollbar.set)

        self.move_bar = tk.Frame(self.list_container, bg = self.theme["panel_bg"])
        self.move_bar.pack(side = "left", fill = "y", padx = (4, 0))

        ttk.Button(self.move_bar, text="↑", width=3, command=self._move_up).pack(pady=(0, 2))
        ttk.Button(self.move_bar, text="↓", width=3, command=self._move_down).pack()

        self.ctrl_bar = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.ctrl_bar.pack(fill  ="x", pady = (6, 0))

        ttk.Button(self.ctrl_bar, text = "Clear Route", command = self.clear).pack(side = "left", fill = "x", expand = True, padx = (0, 2))
        ttk.Button(self.ctrl_bar, text = "Upload Route", command = self.upload).pack(side = "right", fill = "x", expand = True, padx = (2, 0))

        self.progress_var = tk.IntVar(value = 0)
        self.progress_bar = ttk.Progressbar(self.frame, variable = self.progress_var, maximum = 100, mode = "determinate")
        self.progress_bar.pack(fill = "x", pady = (8, 2))

        self.status_label = tk.Label(self.frame, text = "No Route", font = ("Segoe UI", 12, "bold"),
                                     anchor = "w", bg = self.theme["panel_bg"], fg = self._status_color("idle"))
        self.status_label.pack(fill = "x")

    def add_waypoint(self, coords: Tuple[float, float]) -> None:
        self.waypoints.append(coords)
        self._route_changed = True
        self.refresh()

    def clear(self) -> None:
        self.waypoints.clear()
        self._route_changed = True
        self.refresh()

    def upload(self) -> None:
        #self.ctrl.send_test_route()
        if self.waypoints:
            self.ctrl.send_route(self.waypoints)

    # Reorder Functions
    def _move_up(self) -> None:
        sel = self.listbox.curselection()
        
        if not sel or sel[0] == 0:
            return
        
        idx = sel[0]
        self.waypoints[idx - 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx - 1]
        self._route_changed = True
        self.refresh(select = idx - 1)

    def _move_down(self) -> None:
        sel = self.listbox.curselection()

        if not sel or sel[0] == len(self.waypoints) - 1:
            return
        
        idx = sel[0]
        self.waypoints[idx + 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx + 1]
        self._route_changed = True
        self.refresh(select = idx + 1)

    # Sync list and map
    def refresh(self, select: int = None) -> None:
        self.listbox.delete(0, "end")

        for idx, (lat, lon) in enumerate(self.waypoints):
            self.listbox.insert("end", f" {idx + 1:02d}:  {lat:.6f}, {lon:.6f}")

        if select is not None and 0 <= select < len(self.waypoints):
            self.listbox.select_set(select)
            self.listbox.activate(select)

        for m in self._markers:
            m.delete()

        self._markers.clear()

        if self._path is not None:
            self._path.delete()
            self._path = None

        for idx, (lat, lon) in enumerate(self.waypoints):
            if self._wp_icon is not None:   
                self._markers.append(self.map_widget.set_marker(lat, lon, text = str(idx + 1), icon = self._wp_icon, text_color = self.theme["red"]))
            else:
                print("[GUI] WP icon not loaded")

        if len(self.waypoints) > 1:
            self._path = self.map_widget.set_path(self.waypoints, color = self.theme["accent"], width = 3)

    def update(self, telemetry: dict, connection: dict) -> None:
        super().update(telemetry, connection)

        status, current, total = self.ctrl.get_upload_status()

        if self._prev_status == uploadStatus.UPLOADING and status == uploadStatus.DONE:
            self._route_changed = False

        self._prev_status = status
        self._refresh_progress(status, current, total)

    def _refresh_progress(self, status: uploadStatus, current: int, total: int) -> None:
        if status == uploadStatus.UPLOADING:
            pct = int(current / total * 100) if total else 0
            self.progress_var.set(pct)
            self._set_status(f"Uploading {current} / {total}", "uploading")
            return
        
        if status == uploadStatus.FAILED:
            pct = int(current / total * 100) if total else 0
            self.progress_var.set(pct)
            self._set_status(f"Upload Failed ({current} / {total})", "error")
            return
        
        if not self.waypoints:
            self.progress_var.set(0)
            self._set_status("No Route", "idle")

        elif self._route_changed:
            self.progress_var.set(0)
            self._set_status("Current Route Not Uploaded", "dirty")

        else:
            self.progress_var.set(100)
            self._set_status(f"Uploaded ({total} Waypoints)", "done")

    def _status_color(self, state: str) -> str:
        return {
            "uploading": self.theme["accent"],
            "done": self.theme["green"],
            "error": self.theme["red"],
            "dirty": self.theme["orange"],
            "idle": self.theme["fg_dim"],
        }.get(state, self.theme["fg"])
    
    def _set_status(self, text: str, state: str) -> None:
        self.status_label.config(text = text, fg = self._status_color(state))

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.list_container.config(bg = theme["panel_bg"])
        self.move_bar.config(bg = theme["panel_bg"])
        self.ctrl_bar.config(bg = theme["panel_bg"])
        self.status_label.config(bg = theme["panel_bg"])

        self.listbox.config(bg = theme["canvas_bg"], fg = theme["fg"], selectbackground = theme["accent"])
        if self._path is not None:
            self._path.delete()
            self._path = None
            if len(self.waypoints) > 1:
                self._path = self.map_widget.set_path(self.waypoints, color=theme["accent"], width = 3)
