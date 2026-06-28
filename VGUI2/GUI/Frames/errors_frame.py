import time
import sys
import tkinter as tk
from tkinter import ttk
from typing import Set

from GUI.base_frame import BaseFrame
from GUI.print_redirect import PrintRedirector

IGNORED_LOG_NAMES = {"HDG_A1", "HDG_A2", "HDG_A3"}

class ErrorFrame(BaseFrame):
    def __init__(self, parent, theme, ctrl):
        self.error_defs = ctrl.error_defs
        self._last_error = None
        super().__init__(parent, theme, ctrl)

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text = "Errors",
            font = ("Segoe UI", 10, "bold"), padx = 10, pady = 8,
        )
        self.frame.pack(fill = "both", expand = True)

        self.lbl_hex = tk.Label(self.frame, text = "0x00000000", font = ("Consolas", 14, "bold"))
        self.lbl_hex.pack(anchor = "w", pady = (0, 4))

        self.listbox = tk.Listbox(self.frame, height = 20, font = ("Consolas", 9),
                                  selectmode = "browse", borderwidth = 0, highlightthickness = 0)
        self.listbox.pack(fill = "x", pady = 2)

        ttk.Button(self.frame, text = "Reset Errors", command = self.ctrl.reset_errors).pack(fill = "x", pady = (6, 0))

        self.log_label = tk.Label(self.frame, text = "Event Log", font = ("Segoe UI", 8, "bold"),
                                  bg = self.theme["panel_bg"], fg = self.theme["fg_dim"])
        self.log_label.pack(anchor = "w", pady = (4, 2))

        self.log_frame = tk.Frame(self.frame, bg = self.theme["canvas_bg"])
        self.log_frame.pack(fill = "both", expand = True)

        self.log = tk.Text(self.log_frame, height = 4, font = ("Consolas", 12),
                           state = "disabled", wrap = "word",
                           borderwidth = 0, highlightthickness = 0,
                           bg = self.theme["canvas_bg"], fg = self.theme["fg"])
        self.log_scroll = ttk.Scrollbar(self.log_frame, orient = "vertical", command = self.log.yview)
        self.log.config(yscrollcommand = self.log_scroll.set)

        self.log.pack(side = "left", fill = "both", expand = True)
        self.log_scroll.pack(side = "right", fill = "y")

        self.log.tag_configure("set", foreground = self.theme["red"])
        self.log.tag_configure("cleared", foreground = self.theme["green"])
        self.log.tag_configure("ts", foreground = self.theme["fg_dim"])
        self.log.tag_configure("stdout", foreground = self.theme["fg_dim"])
        self.log.tag_configure("stderr", foreground = self.theme["red"])

        self.apply_theme(self.theme)
        self._attach_console()

    def update(self, telemetry: dict, connection: dict) -> None:
        self._last_error = telemetry["error"]
        self._refresh_error_display(self._last_error)

    def _refresh_error_display(self, error: int) -> None:
        self.lbl_hex.config(text = f"0x{error:08X}",
                            fg = self.theme["green"] if error == 0 else self.theme["orange"])

        self.listbox.delete(0, "end")

        if error == 0:
            self.listbox.insert("end", "No active errors")
            self.listbox.itemconfig(0, fg = self.theme["green"], bg = self.theme["canvas_bg"])

        else:
            for bit in range(32):
                if error >> bit & 1:
                    name, desc = self.error_defs.get(bit, (f"BIT{bit}", ""))

                    if name in IGNORED_LOG_NAMES:
                        continue

                    self.listbox.insert("end", f"{bit:02d}: {name} - {desc}")

                    color = self.theme["green"] if bit == 0 else self.theme["red"]
                    self.listbox.itemconfig("end", fg = color, bg = self.theme["canvas_bg"])

    def log_changes(self, new_bits: Set[int], cleared_bits: Set[int]) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.config(state = "normal")

        for bit in sorted(new_bits):
            name, desc = self.error_defs.get(bit, (f"BIT{bit}", ""))

            if name in IGNORED_LOG_NAMES:
                continue

            self.log.insert("end", f"[{ts}] ", "ts")
            self.log.insert("end", f"ERROR SET: BIT {bit} - {name}", "set")

            if desc:
                self.log.insert("end", f" ({desc})")

            self.log.insert("end", "\n")

        for bit in sorted(cleared_bits):
            name, _ = self.error_defs.get(bit, (f"BIT{bit}", ""))

            if name in IGNORED_LOG_NAMES:
                continue

            self.log.insert("end", f"[{ts}] ", "ts")
            self.log.insert("end", f"ERROR CLEARED: BIT {bit} - {name}\n", "cleared")

        self.log.see("end")
        self.log.config(state = "disabled")

    def _attach_console(self) -> None:
        self._orig_stdout = sys.stdout
        self._orig_stderr = sys.stderr

        sys.stdout = PrintRedirector(self.log, tag = "stdout", echo_to = self._orig_stdout)
        sys.stderr = PrintRedirector(self.log, tag = "stderr", echo_to = self._orig_stderr)

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg = theme["panel_bg"], fg = theme["fg"],
                          highlightbackground = theme["border"], highlightthickness = 1,
                          bd = 1, relief = "solid")

        self.lbl_hex.config(bg = theme["panel_bg"])
        self.log_label.config(bg = theme["panel_bg"], fg = theme["fg_dim"])

        self.listbox.config(bg = theme["canvas_bg"], fg = theme["fg"])
        self.log_frame.config(bg = theme["canvas_bg"])
        self.log.config(bg = theme["canvas_bg"], fg = theme["fg"])

        self.log.tag_configure("set", foreground = theme["red"])
        self.log.tag_configure("cleared", foreground = theme["green"])
        self.log.tag_configure("ts", foreground = theme["fg_dim"])
        self.log.tag_configure("stdout", foreground = theme["fg_dim"])
        self.log.tag_configure("stderr", foreground = theme["red"])

        if self._last_error is not None:
            self._refresh_error_display(self._last_error)