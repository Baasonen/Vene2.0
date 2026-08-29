import tkinter as tk
from tkinter import ttk
from typing import Callable

from GUI.base_frame import BaseFrame
from GUI.themes import THEMES
from GUI.settings import load_settings, save_settings

OVERLAY_OPTIONS = ["None", "Speed", "Heading", "Both"]

class SettingsFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl, on_close: Callable[[], None],
                 on_theme_change: Callable[[str], None],
                 on_overlay_change: Callable[[str], None]):
        self.on_close = on_close
        self.on_theme_change = on_theme_change
        self.on_overlay_change = on_overlay_change

        self.settings = load_settings()

        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Settings",
            font = ("Segoe UI", 10, "bold"), padx = 6, pady = 6,
            bg = self.theme["panel_bg"], fg = self.theme["fg"])
        self.frame.pack(fill = "both", expand = True)

        header = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        header.pack(fill = "x", pady = (0, 6))
        self._header = header

        ttk.Button(header, text = "Close", command = self._on_close_click).pack(side = "right")

        self.general_box = self._make_section(self.frame, "GENERAL")

        self.theme_row = self._make_row(self.general_box)
        self.theme_label = self._make_label(self.theme_row, "Default Theme")
        self.theme_label.pack(side = "left")

        self.theme_var = tk.StringVar(value = self.settings.get("theme", "dark"))
        self.theme_var.trace_add("write", self._on_field_change)
        self.theme_combo = ttk.Combobox(
            self.theme_row, textvariable = self.theme_var, values = list(THEMES.keys()),
            state = "readonly", width = 10)
        self.theme_combo.pack(side = "right")
        self.theme_combo.bind("<<ComboboxSelected>>", self._on_theme_pick)

        self.port_row = self._make_row(self.general_box)
        self.port_label = self._make_label(self.port_row, "Default Serial Port")
        self.port_label.pack(side = "left")

        self.port_var = tk.StringVar(value = self.settings.get("port", "COM4"))
        self.port_var.trace_add("write", self._on_field_change)
        self.port_entry = ttk.Entry(self.port_row, textvariable = self.port_var, width = 10)
        self.port_entry.pack(side = "right")

        self.map_box = self._make_section(self.frame, "MAP")

        self.overlay_row = self._make_row(self.map_box)
        self.overlay_label = self._make_label(self.overlay_row, "Nav Overlay")
        self.overlay_label.pack(side = "left")

        self.overlay_var = tk.StringVar(value = self.settings.get("overlay", "Both"))
        self.overlay_var.trace_add("write", self._on_field_change)
        self.overlay_combo = ttk.Combobox(
            self.overlay_row, textvariable = self.overlay_var, values = OVERLAY_OPTIONS,
            state = "readonly", width = 10)
        self.overlay_combo.pack(side = "right")
        self.overlay_combo.bind("<<ComboboxSelected>>", self._on_overlay_pick)

        self.bottom_bar = tk.Frame(self.frame, bg = self.theme["panel_bg"])
        self.bottom_bar.pack(side = "bottom", fill = "x", pady = (8, 0))

        self.status_label = tk.Label(
            self.bottom_bar, text = "", font = ("Segoe UI", 9),
            bg = self.theme["panel_bg"], fg = self.theme["green"])
        self.status_label.pack(side = "top", anchor = "w", pady = (0, 4))

        self.save_frame = tk.Frame(
            self.bottom_bar, bg = self.theme["panel_bg"],
            highlightthickness = 2, highlightbackground = self.theme["panel_bg"])
        self.save_frame.pack(side = "top", fill = "x")

        self.save_button = ttk.Button(self.save_frame, text = "Save", command = self._on_save)
        self.save_button.pack(fill = "x")

        self._update_unsaved_indicator()

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

    def _make_row(self, parent: tk.Frame) -> tk.Frame:
        row = tk.Frame(parent, bg = self.theme["panel_bg"])
        row.pack(fill = "x", padx = 6, pady = (0, 6))

        return row

    def _make_label(self, parent: tk.Frame, text: str) -> tk.Label:
        return tk.Label(parent, text = text, font = ("Segoe UI", 10),
                        bg = self.theme["panel_bg"], fg = self.theme["fg"])

    def _on_theme_pick(self, event = None) -> None:
        self.status_label.config(text = "")
        self.on_theme_change(self.theme_var.get())

    def _on_overlay_pick(self, event = None) -> None:
        self.status_label.config(text = "")
        self.on_overlay_change(self.overlay_var.get())

    def _on_save(self) -> None:
        self.settings["theme"] = self.theme_var.get()
        self.settings["port"] = self.port_var.get().strip() or "COM4"
        self.settings["overlay"] = self.overlay_var.get()

        if save_settings(self.settings):
            self.status_label.config(text = "Saved", fg = self.theme["green"])
            self._update_unsaved_indicator()
            self.on_close()
        else:
            self.status_label.config(text = "Failed to save settings", fg = self.theme["red"])
            self._update_unsaved_indicator()

    def _on_field_change(self, *args) -> None:
        self._update_unsaved_indicator()

    def _has_unsaved_changes(self) -> bool:
        return (self.theme_var.get() != self.settings.get("theme", "dark") or
                self.port_var.get().strip() != self.settings.get("port", "COM4") or
                self.overlay_var.get() != self.settings.get("overlay", "Both"))

    def _update_unsaved_indicator(self) -> None:
        if self._has_unsaved_changes():
            self.save_frame.config(highlightbackground = self.theme["orange"])
            self.save_button.config(text = "Save Changes")
        else:
            self.save_frame.config(highlightbackground = self.theme["panel_bg"])
            self.save_button.config(text = "Save")

    def _on_close_click(self) -> None:
        self.on_close()

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"])
        self._header.config(bg = theme["panel_bg"])
        self.bottom_bar.config(bg = theme["panel_bg"])
        self.save_frame.config(bg = theme["panel_bg"])

        self.general_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.general_box._header_row.config(bg = theme["panel_bg"])
        self.general_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.map_box.config(bg = theme["panel_bg"], highlightbackground = theme["border"])
        self.map_box._header_row.config(bg = theme["panel_bg"])
        self.map_box._title_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        for row in (self.theme_row, self.port_row, self.overlay_row):
            row.config(bg = theme["panel_bg"])

        self.theme_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.port_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.overlay_label.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.status_label.config(bg = theme["panel_bg"])

        self._update_unsaved_indicator()