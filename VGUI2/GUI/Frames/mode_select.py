import time
import tkinter as tk
from GUI.base_frame import BaseFrame

class ModeSelectFrame(BaseFrame):
    MODES = [(0, "STOP"), (1, "MANUAL"), (2, "AUTO"), (3, "RTL")]
    PENDING_TIMEOUT = 5.0 

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text="Mode Select",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8,
        )

        self.frame.pack(fill="x", pady=(0, 10))

        self.grid = tk.Frame(self.frame)
        self.grid.pack(fill="x", pady=4)

        self._requested_mode = None
        self._requested_at = None
        self._last_confirmed = None
        self._buttons: dict[int, tk.Button] = {}

        for col, (mode_id, label) in enumerate(self.MODES):
            btn = tk.Button(
                self.grid, text=label, font=("Segoe UI", 9, "bold"), width=8, height=1,
                command=lambda m=mode_id: self._on_click(m),
            )

            btn.grid(row=0, column=col, padx=4, pady=2, sticky="ew")
            self.grid.grid_columnconfigure(col, weight=1)
            self._buttons[mode_id] = btn

        self.apply_theme(self.theme)

    def _on_click(self, mode_id: int) -> None:
        self._requested_mode = mode_id
        self._requested_at = time.monotonic()
        self.ctrl.set_mode(mode_id)
        self._refresh_buttons()

    def update(self, telemetry: dict, connection: dict) -> None:
        confirmed = telemetry["mode"]
        self._last_confirmed = confirmed

        if self._requested_mode is not None:
            if self._requested_mode == confirmed:
                self._requested_mode = None
                self._requested_at = None

            elif time.monotonic() - self._requested_at > self.PENDING_TIMEOUT:
                self._requested_mode = None
                self._requested_at = None

        self._refresh_buttons()

    def _refresh_buttons(self) -> None:
        confirmed = self._last_confirmed

        for mode_id, btn in self._buttons.items():
            if mode_id == confirmed:
                btn.config(bg = self.theme["accent"], fg = "white",
                           font = ("Segoe UI", 9, "bold"),
                           bd = 2, relief = "flat",
                           highlightthickness=0)
                
            elif mode_id == self._requested_mode:
                btn.config(bg = self.theme["panel_bg"], fg = self.theme["orange"],
                           font = ("Segoe UI", 9, "bold"),
                           bd = 0, relief="solid",
                           highlightbackground = self.theme["orange"],
                           highlightcolor = self.theme["orange"],
                           highlightthickness = 2)
                
            else:
                btn.config(bg = self.theme["bg"], fg = self.theme["fg"],
                           font = ("Segoe UI", 9),
                           bd = 2, relief="flat",
                           highlightthickness = 0)

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)

        self.frame.config(bg=theme["panel_bg"], fg=theme["fg"],
                          highlightbackground=theme["border"], highlightthickness=1,
                          bd=1, relief="solid")
        
        self.grid.config(bg=theme["panel_bg"])
        self._refresh_buttons()