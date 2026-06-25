import tkinter as tk
from tkinter import ttk
from typing import Tuple

from GUI.base_frame import BaseFrame

class WaypointFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, map_widget):
        self.map_widget = map_widget
        self.waypoints: list[Tuple[float, float]] = []
        self._markers = []
        self._path = None

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

    def add_waypoint(self, coords: Tuple[float, float]) -> None:
        self.waypoints.append(coords)
        self.refresh()

    def clear(self) -> None:
        self.waypoints.clear()
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
        self.refresh(select = idx - 1)

    def _move_down(self) -> None:
        sel = self.listbox.curselection()

        if not sel or sel[0] == len(self.waypoints) - 1:
            return
        
        idx = sel[0]
        self.waypoints[idx + 1], self.waypoints[idx] = self.waypoints[idx], self.waypoints[idx + 1]
        self.refresh(select = idx + 1)

    # Sync list and map
    def refresh(self, select: int = None) -> None:
        self.listbox.delete(0, "end")

        for idx, (lat, lon) in enumerate(self.waypoints):
            self.listbox.insert("end", f" #{idx + 1:02d}:  {lat:.6f}, {lon:.6f}")

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
            self._markers.append(self.map_widget.set_marker(lat, lon, text = str(idx + 1)))
 
        if len(self.waypoints) > 1:
            self._path = self.map_widget.set_path(self.waypoints, color = self.theme["accent"], width = 3)

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.list_container.config(bg = theme["panel_bg"])
        self.move_bar.config(bg = theme["panel_bg"])
        self.ctrl_bar.config(bg = theme["panel_bg"])

        self.listbox.config(bg = theme["canvas_bg"], fg = theme["fg"], selectbackground = theme["accent"])
        if self._path is not None:
            self._path.delete()
            self._path = None
            if len(self.waypoints) > 1:
                self._path = self.map_widget.set_path(self.waypoints, color=theme["accent"], width = 3)