from __future__ import annotations

VERSION_MAJOR = 5
VERSION_MINOR = 6

VERSION = f"2.{VERSION_MAJOR}.{VERSION_MINOR}"

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import time
import sys
import os
from typing import Dict, Set, Tuple, Optional

# Import GUI only after showing splash screen

from GUI.themes import THEMES
from GUI.settings import load_settings

def _load_modules() -> None:
    global ConnectionStatusFrame, ModeSelectFrame, TelemetryFrame
    global ErrorFrame, WaypointFrame, WaypointListFrame, MapFrame
    global ManualControlFrame, PatternPlannerFrame, SwapContainer
    global PYGAME_AVAIL, Controller, TelemetryRecorder, SettingsFrame

    from GUI.Frames.connection_status_frame import ConnectionStatusFrame
    from GUI.Frames.mode_select_frame import ModeSelectFrame
    from GUI.Frames.telemetry_frame import TelemetryFrame
    from GUI.Frames.errors_frame import ErrorFrame
    from GUI.Frames.waypoint_frame import WaypointFrame
    from GUI.Frames.waypoint_list_frame import WaypointListFrame
    from GUI.Frames.map_frame import MapFrame
    from GUI.Frames.manual_control_frame import ManualControlFrame
    from GUI.Pattern.frame import PatternPlannerFrame
    from GUI.swap_container import SwapContainer
    from GUI.Frames.settings_frame import SettingsFrame

    try:
        import pygame
        PYGAME_AVAIL = True
    except ImportError:
        PYGAME_AVAIL = False

    from VCOM.vcom import Controller
    from VCOM.telemetry_recorder import TelemetryRecorder

class VGUI:
    def __init__(self, root: tk.Tk, controller: Controller):
        self.root = root
        self.ctrl = controller

        self.settings = load_settings()
        self.current_theme_name = self.settings.get("theme", "dark")
        if self.current_theme_name not in THEMES:
            self.current_theme_name = "dark"

        self.theme = THEMES[self.current_theme_name]

        self.root.title(f"VGUI {VERSION}")
        self.root.geometry("1980x1080")
        self.root.configure(bg = self.theme["bg"])

        self.recorder = TelemetryRecorder(error_defs = self.ctrl.error_defs)

        self._build_layout()
        self._apply_ttk_style()
        
        # Callbacks
        self.ctrl.on_error_change = self._on_error_change
        self.ctrl.on_home_received = self._on_home_received

        self._refresh()
        self._update_clock()


    def _build_layout(self) -> None:
        self.header = tk.Frame(self.root, bg = self.theme["panel_bg"], height = 40,
                               bd = 1, relief = "solid", highlightbackground = self.theme["border"])
        self.header.pack(fill = "x", side = "top", pady = (0, 10))

        self.lbl_title = tk.Label(self.header, text = f"VGUI {VERSION}", font = ("Segoe UI", 12, "bold"),
                 bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_title.pack(side = "left", padx = 15)

        self.lbl_clock = tk.Label(self.header, text = "", font = ("Segoe UI", 12),
                                  bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_clock.pack(side = "left", padx = (0, 15))

        self.btn_settings = ttk.Button(self.header, text = "Settings", command = self._on_toggle_settings)
        self.btn_settings.pack(side = "right", padx = (0, 5), pady = 5)
        
        self.theme_var = tk.StringVar(value = self.current_theme_name)
        self.theme_combo = ttk.Combobox(self.header, textvariable = self.theme_var,
                                        values = list(THEMES.keys()), state = "readonly",
                                        font = ("Segoe UI", 12), width = 8)
        self.theme_combo.pack(side = "right", padx = (0, 5), pady = 5)
        self.theme_combo.bind("<<ComboboxSelected>>", self._on_theme_select)

        self.btn_record = ttk.Button(self.header, text = "Record", command = self._on_toggle_recording)
        self.btn_record.pack(side = "right", padx = (0, 5))

        self.main = tk.Frame(self.root, bg=self.theme["bg"])
        self.main.pack(fill="both", expand=True, padx=10, pady=5)

        self.col_left = tk.Frame(self.main, bg=self.theme["bg"], width=420)
        self.col_left.pack(side="left", fill="both", expand=False, padx=(0, 5))
        self.col_left.pack_propagate(False)

        self.col_mid = tk.Frame(self.main, bg=self.theme["bg"])
        self.col_mid.pack(side="left", fill="both", expand=True, padx=5)

        self.col_right = tk.Frame(self.main, bg=self.theme["bg"], width=350)
        self.col_right.pack(side="left", fill="both", expand=False, padx=(5, 0))
        self.col_right.pack_propagate(False)

        self.map_frame = MapFrame(self.col_mid, self.theme, self.ctrl, 
                                  on_add_waypoint = self._on_add_waypoint,
                                  on_set_home = self._on_set_home,
                                  on_save_route = self._on_save_route,
                                  on_load_route = self._on_load_route,
                                  on_pattern_planner = self._on_toggle_pattern_planner)
        self.connection_frame = ConnectionStatusFrame(self.col_left, self.theme, self.ctrl)

        self.error_swap = SwapContainer(self.col_left, self.theme)

        self.mode_select_frame = ModeSelectFrame(self.col_right, self.theme, self.ctrl)
        self.manual_control_frame = ManualControlFrame(self.col_right, self.theme, self.ctrl, self.root)
        self.telemetry_frame = TelemetryFrame(self.col_right, self.theme, self.ctrl)
        self.error_frame = ErrorFrame(self.error_swap.base_parent, self.theme, self.ctrl)
        self.waypoint_frame = WaypointFrame(self.col_right, self.theme, self.ctrl, self.map_frame.widget,
                                            on_toggle_list = self._on_toggle_waypoint_list)

        self.frames = [
            self.mode_select_frame,
            self.manual_control_frame,
            self.connection_frame,
            self.telemetry_frame,
            self.error_frame,
            self.waypoint_frame,
            self.map_frame
        ]

        self.map_frame.set_tiles(self.theme["dark_map"])

    def _on_add_waypoint(self, coords: Tuple[float, float]) -> None:
        self.waypoint_frame.add_waypoint(coords)

    def _on_set_home(self, coords: Tuple[float, float]) -> None:
        self.ctrl.set_home(coords[0], coords[1])

    def _on_save_route(self, path: str) -> None:
        self.waypoint_frame.save_route(path)

    def _on_load_route(self, path: str):
        return self.waypoint_frame.load_route(path)

    def _on_toggle_pattern_planner(self) -> None:
        if self.error_swap.is_overlaid:
            was_pattern = isinstance(self.error_swap.active_overlay, PatternPlannerFrame)
            self._close_overlay()
            if was_pattern:
                return

        self._open_pattern_planner()

    def _on_toggle_waypoint_list(self) -> None:
        if self.error_swap.is_overlaid:
            was_list = isinstance(self.error_swap.active_overlay, WaypointListFrame)
            self._close_overlay()
            if was_list:
                return

        self._open_waypoint_list()

    def _open_pattern_planner(self) -> None:
        pattern_planner_frame = self.error_swap.show_overlay(
            lambda parent: PatternPlannerFrame(
                parent, self.theme, self.ctrl, self.map_frame,
                on_close = self._close_overlay,
                on_apply_route = self._on_apply_pattern_route))

        self.frames.append(pattern_planner_frame)

    def _on_toggle_settings(self) -> None:
        if self.error_swap.is_overlaid:
            was_settings = isinstance(self.error_swap.active_overlay, SettingsFrame)
            self._close_overlay()
            if was_settings:
                return

        self._open_settings()

    def _open_settings(self) -> None:
        settings_frame = self.error_swap.show_overlay(
            lambda parent: SettingsFrame(
                parent, self.theme, self.ctrl,
                on_close = self._close_overlay,
                on_theme_change = self._on_settings_theme_change,
                on_overlay_change = self._on_settings_overlay_change))

        self.frames.append(settings_frame)

    def _on_settings_theme_change(self, theme_name: str) -> None:
        self.theme_var.set(theme_name)
        self._on_theme_select()

    def _on_settings_overlay_change(self, overlay_mode: str) -> None:
        self.map_frame.set_overlay_mode(overlay_mode)

    def _open_waypoint_list(self) -> None:
        list_frame = self.error_swap.show_overlay(
            lambda parent: WaypointListFrame(
                parent, self.theme, self.ctrl, self.waypoint_frame,
                on_close = self._close_overlay))

        self.waypoint_frame.attach_list(list_frame)
        self.frames.append(list_frame)

    def _close_overlay(self) -> None:
        if not self.error_swap.is_overlaid:
            return

        active = self.error_swap.active_overlay
        self.frames.remove(active)

        if isinstance(active, WaypointListFrame):
            self.waypoint_frame.select(None)
            self.waypoint_frame.detach_list()

        self.error_swap.close_overlay()

    def _on_apply_pattern_route(self, waypoints, overwrite: bool) -> None:
        self.waypoint_frame.load_waypoints(waypoints, overwrite)

    def _refresh(self) -> None:
        telemetry = self.ctrl.get_telemetry_data()
        connection = self.ctrl.get_connection_status()

        if self.recorder.active:
            self.recorder.record(telemetry, connection)

            elapsed = int(time.time() - self.recorder.start_time)
            mm, ss = divmod(elapsed, 60)
            self.btn_record.config(text = f"Stop ({mm:02d}:{ss:02d})")

        for frame in self.frames:
            frame.update(telemetry, connection)

        self.root.after(100, self._refresh)

    # Telemetry recording
    def _on_toggle_recording(self) -> None:
        if self.recorder.active:
            self._stop_recording()
        else:
            self._start_recording()

    def _start_recording(self) -> None:
        try:
            self.recorder.start()
        except OSError as e:
            messagebox.showerror("Recording", f"Could not start recording:\n{e}")
            return

        self.btn_record.config(text = "Stop (00:00)", style = "Recording.TButton")

    def _stop_recording(self) -> None:
        path = self.recorder.stop()
        self.btn_record.config(text = "Record", style = "TButton")

        if path:
            messagebox.showinfo("Recording saved", f"Telemetry recording saved to:\n{path}")

    def shutdown(self) -> None:
        # Close in-progress file when shutting down
        if self.recorder.active:
            self.recorder.stop()

    def _update_clock(self) -> None:
        self.lbl_clock.config(text = time.strftime("%H:%M:%S"))
        self.root.after(1000, self._update_clock)

    def _on_theme_select(self, event = None) -> None:
        selected = self.theme_var.get()

        if selected == self.current_theme_name:
            return
        
        self.current_theme_name = selected
        self.theme = THEMES[self.current_theme_name]
        self._apply_ttk_style()

        self.root.configure(bg = self.theme["bg"])
        self.header.config(bg = self.theme["panel_bg"], highlightbackground = self.theme["border"])
        self.lbl_title.config(bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.lbl_clock.config(bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.main.config(bg = self.theme["bg"])
        self.col_left.config(bg = self.theme["bg"])
        self.col_mid.config(bg = self.theme["bg"])
        self.col_right.config(bg = self.theme["bg"])
        self.error_swap.apply_theme(self.theme)

        for frame in self.frames:
            frame.apply_theme(self.theme)

        self.map_frame.set_tiles(self.theme["dark_map"])


    def _on_error_change(self, new_bits, cleared_bits) -> None:
        self.error_frame.log_changes(new_bits, cleared_bits)

    def _on_home_received(self, lat: float, lon: float) -> None:
        def _update() -> None:
            telemetry = self.ctrl.get_telemetry_data()
            connection = self.ctrl.get_connection_status()
            for frame in self.frames:
                frame.update(telemetry, connection)
        self.root.after(0, _update)

    def _apply_ttk_style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")

        style.configure("TButton",
            background = self.theme["panel_bg"],
            foreground = self.theme["fg"],
            bordercolor = self.theme["fg_dim"],
            focuscolor = self.theme["border"],
            relief = "solid",
            borderwidth = 1,
            padding = 4
        )

        style.map("TButton",
            background = [("active", self.theme["accent"]), ("pressed", self.theme["bg"])],
            foreground = [("active", self.theme["fg"])]
        )

        style.configure("Recording.TButton",
            background = self.theme["panel_bg"],
            foreground = self.theme["fg"],
            bordercolor = self.theme["green"],
            focuscolor = self.theme["green"],
            relief = "solid",
            borderwidth = 2,
            padding = 4
        )

        style.map("Recording.TButton",
            background = [("active", self.theme["accent"]), ("pressed", self.theme["bg"])],
            foreground = [("active", self.theme["fg"])],
            bordercolor = [("active", self.theme["green"]), ("!active", self.theme["green"])]
        )

        style.configure("TScrollbar",
            background = self.theme["panel_bg"],
            troughcolor = self.theme["canvas_bg"],
            arrowcolor = self.theme["fg"],
            bordercolor = self.theme["border"]
        )

        style.configure("TCombobox",
            fieldbackground = self.theme["panel_bg"], 
            background = self.theme["panel_bg"],
            foreground = self.theme["fg"],
            selectbackground = self.theme["panel_bg"], 
            selectforeground = self.theme["fg"],
            arrowcolor = self.theme["fg"]
        )

        style.map("TCombobox",
            fieldbackground = [("readonly", self.theme["panel_bg"]),
                               ("focus", self.theme["panel_bg"])],
            selectbackground = [("readonly", self.theme["panel_bg"])],
            selectforeground = [("readonly", self.theme["fg"])]
        )

        style.configure("Horizontal.TProgressbar",
            background = self.theme["accent"],
            troughcolor = self.theme["canvas_bg"],
            bordercolor = self.theme["border"]
        )

def _create_splash(root: tk.Tk):
    theme_id = next(iter(THEMES))

    splash = tk.Toplevel(root)
    splash.overrideredirect(True) # No title bar / borders
    splash.config(bg = THEMES[theme_id]["panel_bg"])

    width, height = 420, 220
    screen_w = splash.winfo_screenwidth()
    screen_h = splash.winfo_screenheight()

    x = (screen_w - width) // 2
    y = (screen_h - height) // 2

    splash.geometry(f"{width}x{height}+{x}+{y}")

    splash.attributes("-topmost", True)

    tk.Label(splash, text = "VGUI", font = ("Segoe UI", 28, "bold"),
             bg = THEMES[theme_id]["panel_bg"], fg = THEMES[theme_id]["fg"]).pack(pady = (45, 5))
    tk.Label(splash, text = f"{VERSION}", font = ("Segoe UI", 16),
             bg = THEMES[theme_id]["panel_bg"], fg = THEMES[theme_id]["fg"]).pack()

    status_var = tk.StringVar(value = "Starting...")
    tk.Label(splash, textvariable = status_var, font = ("Segoe UI", 10),
             bg = THEMES[theme_id]["panel_bg"], fg = THEMES[theme_id]["fg"]).pack(side = "bottom", pady = 20)

    # Force the window to paint
    splash.update_idletasks()
    splash.update()
    return splash, status_var

MIN_SPLASH_SECONDS = 1.0

def main() -> None:
    start_time = time.monotonic()

    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("VCOM.VGUI." + VERSION)
        except Exception:
            pass

    root = tk.Tk()
    root.withdraw() # Hide untill fully built

    splash, splash_status = _create_splash(root)

    try:
        icon_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Icons", "main.ico")
        root.iconbitmap(icon_path)
    except Exception:
        messagebox.showerror("Error", f"Could not load icon Icons/main.ico")

    def set_status(text: str) -> None:
        splash_status.set(text)
        splash.update()

    set_status("Loading Modules...")
    _load_modules()

    port = sys.argv[1] if len(sys.argv) > 1 else load_settings().get("port", "COM4")

    set_status("VCOM Init...")
    controller = Controller(port = port)

    set_status("Building UI...")
    prog = VGUI(root, controller)

    elapsed = time.monotonic() - start_time
    remaining = MIN_SPLASH_SECONDS - elapsed
    if remaining > 0:
        time.sleep(remaining)

    splash.destroy()
    root.deiconify()
    root.lift()
    root.focus_force()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        prog.shutdown()
        controller.stop()

if __name__ == "__main__":
    main()