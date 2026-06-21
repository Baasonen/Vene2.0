VERSION = 2.1

import tkinter as tk
from tkinter import ttk, scrolledtext

import time
import sys
from typing import Dict, Set, Tuple, Optional

from GUI.themes import THEMES

from GUI.Frames.connection_status_frame import ConnectionStatusFrame
from GUI.Frames.mode_select_frame import ModeSelectFrame
from GUI.Frames.telemetry_frame import TelemetryFrame
from GUI.Frames.errors_frame import ErrorFrame
from GUI.Frames.waypoint_frame import WaypointFrame
from GUI.Frames.map_frame import MapFrame

try:
    import pygame
    PYGAME_AVAIL = True
except ImportError:
    PYGAME_AVAIL = False

from vcom.vcom import Controller

class VGUI:
    def __init__(self, root: tk.Tk, controller: Controller):
        self.root = root
        self.ctrl = controller
        self.current_theme_name = "dark"
        self.theme = THEMES[self.current_theme_name]

        self.root.title(f"VGUI {VERSION} / VCOM {controller.version}")
        self.root.geometry("1980x1080")
        self.root.configure(bg = self.theme["bg"])

        self._build_layout()
        
        # Callbacks
        self.ctrl.on_error_change = self._on_error_change
        self.ctrl.on_home_received = self._on_home_received

        self._refresh()


    def _build_layout(self) -> None:
        self.header = tk.Frame(self.root, bg = self.theme["panel_bg"], height = 40,
                               bd = 1, relief = "solid", highlightbackground = self.theme["border"])
        self.header.pack(fill = "x", side = "top", pady = (0, 10))

        tk.Label(self.header, text = f"VGUI {VERSION}", font = ("Sege UI", 11, "bold"),
                 bg = self.theme["panel_bg"], fg = self.theme["fg"]).pack(side = "left", padx = 15)
        
        self.btn_theme = tk.Button(self.header, text = "Swithc Theme", font = ("Sege UI", 11, "bold"),
                                   command = self._toggle_theme, bg = self.theme["bg"],
                                   fg = self.theme["fg"], relief = "flat", bd = 0, padx = 10)
        self.btn_theme.pack(side = "right", padx = 15, pady = 5)

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
                                  on_set_home = self._on_set_home)
        self.connection_frame = ConnectionStatusFrame(self.col_left, self.theme, self.ctrl)
        self.mode_select_frame = ModeSelectFrame(self.col_right, self.theme, self.ctrl)
        self.telemetry_frame = TelemetryFrame(self.col_right, self.theme, self.ctrl)
        self.error_frame = ErrorFrame(self.col_left, self.theme, self.ctrl)
        self.waypoint_frame = WaypointFrame(self.col_right, self.theme, self.ctrl, self.map_frame.widget)

        self.frames = [
            self.mode_select_frame,
            self.connection_frame,
            self.telemetry_frame,
            self.error_frame,
            self.waypoint_frame,
            self.map_frame
        ]

        self.map_frame.set_tiles(self.current_theme_name == "dark")

    def _on_add_waypoint(self, coords: Tuple[float, float]) -> None:
        self.waypoint_frame.add_waypoint(coords)

    def _on_set_home(self, coords: Tuple[float, float]) -> None:
        self.ctrl.set_home(coords[0], coords[1])
    
    def _refresh(self) -> None:
        telemetry = self.ctrl.get_telemetry_data()
        connection = self.ctrl.get_connection_status()

        for frame in self.frames:
            frame.update(telemetry, connection)

        self.root.after(100, self._refresh)

    def _toggle_theme(self) -> None:
        self.current_theme_name = "light" if self.current_theme_name == "dark" else "dark"
        self.theme = THEMES[self.current_theme_name]

        self.root.configure(bg = self.theme["bg"])
        self.header.config(bg = self.theme["panel_bg"], highlightbackground = self.theme["border"])
        self.btn_theme.config(bg = self.theme["bg"])
        self.col_left.config(bg = self.theme["bg"])
        self.col_mid.config(bg = self.theme["bg"])
        self.col_right.config(bg = self.theme["bg"])

        for frame in self.frames:
            frame.apply_theme(self.theme)

        self.map_frame.set_tiles(self.current_theme_name == "dark")


    def _on_error_change(self, new_bits, cleared_bits) -> None:
        pass

    def _on_home_received(self, lat: float, lon: float) -> None:
        def _update() -> None:
            telemetry = self.ctrl.get_telemetry_data()
            connection = self.ctrl.get_connection_status()
            for frame in self.frames:
                frame.update(telemetry, connection)
        self.root.after(0, _update)

        

def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    controller = Controller(port = port)

    root = tk.Tk()
    VGUI(root, controller)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()

if __name__ == "__main__":
    main()