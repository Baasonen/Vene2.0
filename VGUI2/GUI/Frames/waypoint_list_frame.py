import tkinter as tk
from tkinter import ttk

from GUI.base_frame import BaseFrame
from GUI.Pattern.geometry import haversine_distance

def _format_length(length_m: float) -> str:
    return f"{length_m:.0f} m" if length_m < 1000 else f"{length_m / 1000:.2f} km"

class WaypointListFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, owner, on_close):
        self.owner = owner
        self.on_close = on_close

        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Waypoint List",
            font = ("Segoe UI", 10, "bold"), padx = 6, pady = 6,
            bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "both", expand = True)

        header = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        header.pack(fill = "x", pady = (0, 6))
        self._header = header

        ttk.Button(header, text = "Close", command = self._on_close_click).pack(side = "right")

        self.summary_box = self._make_section(self.frame)

        summary_row = tk.Frame(self.summary_box, bg = self.theme["panel_bg"])
        summary_row.pack(fill = "x", padx = 6, pady = 6)
        self._summary_row = summary_row

        self.count_label = tk.Label(summary_row, text = "0 defined", font = ("Consolas", 11),
                                    bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.count_label.pack(side = "left", padx = 6)

        self.length_label = tk.Label(summary_row, text = "", font = ("Consolas", 11),
                                     bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.length_label.pack(side = "left", padx = (4, 0))

        self.bottom_bar = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.bottom_bar.pack(side = "bottom", fill = "x", pady = (8, 0))

        ttk.Button(self.bottom_bar, text = "Clear Route", command = self._clear_route).pack(fill = "x")

        self.list_box = self._make_section(self.frame, expand = True)

        self.list_container = tk.Frame(self.list_box, bg = self.theme["panel_bg"])
        self.list_container.pack(side = "top", fill = "both", expand = True, padx = 6, pady = 6)

        self.listbox = tk.Listbox(self.list_container, font = ("Consolas", 9),
                                  borderwidth = 0, highlightthickness = 0, selectmode = "single",
                                  bg = self.theme["canvas_bg"], fg = self.theme["fg"],
                                  selectbackground = self.theme["accent"])
        self.listbox.pack(side = "left", fill = "both", expand = True)
        self.listbox.bind("<<ListboxSelect>>", self._on_listbox_select)

        scrollbar = ttk.Scrollbar(self.list_container, orient = "vertical", command = self.listbox.yview)
        scrollbar.pack(side = "left", fill = "y")
        self.listbox.config(yscrollcommand = scrollbar.set)

        self.move_bar = tk.Frame(self.list_container, bg = self.theme["panel_bg"])
        self.move_bar.pack(side = "left", fill = "y", padx = (4, 0))

        ttk.Button(self.move_bar, text = "↑", width = 3, command = self._move_up).pack(pady = (0, 2))
        ttk.Button(self.move_bar, text = "↓", width = 3, command = self._move_down).pack()
        ttk.Button(self.move_bar, text = "D", width = 3, command = self._delete_selected).pack(pady = (8, 0))
        ttk.Button(self.move_bar, text = "R", width = 3, command = self._reverse_route).pack(pady = (4, 0))

        self.refresh_list()

    def _make_section(self, parent, expand: bool = False) -> tk.Frame:
        box = tk.Frame(parent, bg = self.theme["panel_bg"], bd = 1,
                       relief = "solid", highlightbackground = self.theme["border"])
        box.pack(fill = "both" if expand else "x", expand = expand, pady = (0, 8))

        return box

    def _move_up(self) -> None:
        self.owner.move_up()

    def _move_down(self) -> None:
        self.owner.move_down()

    def _delete_selected(self) -> None:
        self.owner.delete_selected()

    def _reverse_route(self) -> None:
        self.owner.reverse_route()

    def _clear_route(self) -> None:
        self.owner.clear()

    def _on_close_click(self) -> None:
        self.on_close()

    def _on_listbox_select(self, event = None) -> None:
        sel = self.listbox.curselection()
        idx = sel[0] if sel else None
        self.owner.select(idx)

    def refresh_list(self) -> None:
        self.listbox.delete(0, "end")

        for idx, (lat, lon) in enumerate(self.owner.waypoints):
            self.listbox.insert("end", f" {idx + 1:02d}:  {lat:.6f}, {lon:.6f}")

        sel = self.owner.selected_idx
        if sel is not None and 0 <= sel < len(self.owner.waypoints):
            self.listbox.select_set(sel)
            self.listbox.activate(sel)

        self._update_summary()

    def _update_summary(self) -> None:
        waypoints = self.owner.waypoints
        n = len(waypoints)

        text = "1 waypoint" if n == 1 else f"{n} waypoints"
        self.count_label.config(text = text, fg = self.theme["fg"] if n else self.theme["fg_dim"])

        if n >= 2:
            length_m = sum(haversine_distance(a, b) for a, b in zip(waypoints, waypoints[1:]))
            self.length_label.config(text = f"/  {_format_length(length_m)}", fg = self.theme["fg"])
        else:
            self.length_label.config(text = "")

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])
        self._header.config(bg = theme["panel_bg"])
        self.bottom_bar.config(bg = theme["panel_bg"])

        for box, extra_rows in (
            (self.summary_box, [self._summary_row]),
            (self.list_box, [self.list_container, self.move_bar]),
        ):
            box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
            for row in extra_rows:
                row.config(bg = theme["panel_bg"])

        self.listbox.config(bg = theme["canvas_bg"], fg = theme["fg"], selectbackground = theme["accent"])

        self.count_label.config(bg = theme["panel_bg"],
                                fg = theme["fg"] if self.owner.waypoints else theme["fg_dim"])
        self.length_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])