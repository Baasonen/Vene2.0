import tkinter as tk
from GUI.base_frame import BaseFrame
from vcom.protocol import MODE_NAMES


class TelemetryFrame(BaseFrame):
    METRICS = [
        ("POS", "GPS", "—, —"),
        ("HDG", "Heading", "—°"),
        ("NAV", "Target WP (idx)", "—"),
        ("HOME", "Home WP", "—, —"),
        ("BAT", "Battery", "—%"),
        ("GPS", "GPS HDOP", "—"),
        ("SIG", "LoRa RSSI", "— dBm"),
    ]

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text="Live Telemetry",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8,
        )
        self.frame.pack(fill="x", pady=(0, 10))

        self.widgets = {}
        self._last_telemetry = None
        self._last_connection = None

        for code, label, default in self.METRICS:
            container = tk.Frame(self.frame, bg=self.theme["panel_bg"])
            container.pack(fill="x", pady=4)

            desc = tk.Label(container, text=label, font=("Segoe UI", 8, "bold"),
                            bg=self.theme["panel_bg"], fg=self.theme["fg_dim"])
            desc.pack(anchor="w")

            value = tk.Label(container, text=default, font=("Consolas", 12, "bold"),
                              bg=self.theme["panel_bg"], fg=self.theme["fg"])
            value.pack(anchor="w", padx=2)

            self.widgets[code] = (value, container, desc)

        self.apply_theme(self.theme)

    def update(self, telemetry: dict, connection: dict) -> None:
        self._last_telemetry = telemetry
        self._last_connection = connection
        self._refresh(telemetry, connection)

    def _refresh(self, telemetry: dict, connection: dict) -> None:
        pos, _, _ = self.widgets["POS"]
        pos.config(text=f"{telemetry['lat']:.6f} N, {telemetry['lon']:.6f} E"
                   if telemetry["lat"] != 0 else "NO GPS FIX")

        self.widgets["HDG"][0].config(text=f"{telemetry['heading']:.1f}°")
        self.widgets["NAV"][0].config(
            text=f"WP {telemetry['target_idx']} [{MODE_NAMES.get(telemetry['mode'], 'UNKNOWN')}]")

        bat, _, _ = self.widgets["BAT"]
        bat.config(text=f"{telemetry['battery']}%",
                   fg=self.theme["green"] if telemetry["battery"] > 30 else self.theme["red"])

        gps, _, _ = self.widgets["GPS"]
        gps.config(text=f"{telemetry['hdop']:.2f} (Good)" if telemetry["hdop"] < 1.5
                   else f"{telemetry['hdop']:.2f} (Degraded)")

        self.widgets["SIG"][0].config(text=f"{telemetry['signal']} dBm")

        home, _, _ = self.widgets["HOME"]
        if connection["home_set"]:
            home.config(text=f"{connection['home_lat']:.6f} N, {connection['home_lon']:.6f} E",
                        fg=self.theme["fg"])
        else:
            home.config(text="N/A", fg=self.theme["orange"])

    def apply_theme(self, theme: dict) -> None:
        super().apply_theme(theme)
        self.frame.config(bg=theme["panel_bg"], fg=theme["fg"],
                          highlightbackground=theme["border"], highlightthickness=1,
                          bd=1, relief="solid")

        for value, container, desc in self.widgets.values():
            container.config(bg=theme["panel_bg"])
            desc.config(bg=theme["panel_bg"], fg=theme["fg_dim"])
            value.config(bg=theme["panel_bg"], fg=theme["fg"])

        if self._last_telemetry is not None:
            self._refresh(self._last_telemetry, self._last_connection)