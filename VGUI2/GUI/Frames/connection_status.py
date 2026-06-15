import tkinter as tk
from GUI.base_frame import BaseFrame


class ConnectionStatusFrame(BaseFrame):

    LINKS = (
        ("RX", "RX"),
        ("LORA", "LoRa"),
        ("WIFI", "WiFi"),
    )

    def build(self):
        self.frame = tk.LabelFrame(self.parent, text="Connection Status",
                                    font=("Segoe UI", 10, "bold"), padx=10, pady=8)
        self.frame.pack(fill="x", pady=(0, 10))

        self.widgets = {}
        self.status = {code: False for code, _ in self.LINKS}

        for code, label_text in self.LINKS:
            container = tk.Frame(self.frame)
            container.pack(fill="x", pady=3)

            name = tk.Label(container, text=label_text, font=("Segoe UI", 9, "bold"),
                            width=6, anchor="w")
            name.pack(side="left", padx=8)

            status = tk.Label(container, text="OFFLINE", font=("Segoe UI", 9, "bold"),
                              width=10, anchor="center")
            status.pack(side="right", padx=8, pady=3)

            self.widgets[code] = (name, status, container)

        self.apply_theme(self.theme)

    def _refresh(self, code):
        name, status, container = self.widgets[code]
        if self.status[code]:
            status.config(text="Online", bg=self.theme["green"], fg=self.theme.get("status_fg", "#ffffff"))
        else:
            status.config(text="Offline", bg=self.theme["red"], fg=self.theme.get("status_fg", "#ffffff"))

    def update(self, telemetry: dict, connection: dict) -> None:
        self.status["RX"] = connection["receiver_connected"]
        self.status["LORA"] = connection["lora_online"]
        self.status["WIFI"] = connection["wifi_online"]

        for code in self.widgets:
            self._refresh(code)

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        for code, (name, status, container) in self.widgets.items():
            container.config(bg=theme["panel_bg"])
            name.config(bg=theme["panel_bg"], fg=theme["fg"])
            self._refresh(code)