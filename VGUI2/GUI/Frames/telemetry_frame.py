import tkinter as tk
from GUI.base_frame import BaseFrame
from vcom.protocol import MODE_NAMES


class TelemetryFrame(BaseFrame):
    METRICS = [
        ("POS", "GPS", "—, —"),
        ("HDG", "Heading", "—°"),
        ("GPS", "GPS HDOP", "—"),
        ("MAG", "Mag Accuracy", "—"),
        ("NAV", "Target WP (idx)", "—"),
        ("BAT", "Battery", "—%"),
        ("SIG", "LoRa RSSI", "— dBm"),
        ("HOME", "Home WP", "—, —"),
    ]

    MAG_ACC_BIT_NAMES = ["HDG_A1", "HDG_A2", "HDG_A3"]

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text="Telemetry",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8,
        )
        self.frame.pack(fill="x", pady=(0, 10))

        self.widgets = {}
        self._last_telemetry = None
        self._last_connection = None

        self._mag_acc_bits = [self._find_bit(name) for name in self.MAG_ACC_BIT_NAMES]

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

    def _find_bit(self, name: str):
        for bit, (defname, _desc) in self.ctrl.error_defs.items():
            if defname == name:
                return bit
            
        return None

    def update(self, telemetry: dict, connection: dict) -> None:
        self._last_telemetry = telemetry
        self._last_connection = connection
        self._refresh(telemetry, connection)

    def _refresh(self, telemetry: dict, connection: dict) -> None:
        pos, _, _ = self.widgets["POS"]

        if telemetry["lat"] != 0:
            pos.config(text=f"{telemetry['lat']:.6f} N, {telemetry['lon']:.6f} E", fg = self.theme["fg"])
        else:
            pos.config(text = "NO GPS FIX", fg = self.theme["red"])

        self.widgets["HDG"][0].config(text=f"{telemetry['heading']:.1f}°")
        self.widgets["NAV"][0].config(
            text=f"WP {telemetry['target_idx']} [{MODE_NAMES.get(telemetry['mode'], 'UNKNOWN')}]")
        
        mag, _, _ = self.widgets["MAG"]
        error = telemetry.get("error", 0)

        level = 0
        for i, bit in enumerate(self._mag_acc_bits, start = 1):
            if bit is not None and error >> bit & 1:
                level = i

        if level == 0:
            mag.config(text = "0/3", fg = self.theme["red"])
        elif level == 1:
            mag.configure(text = "1/3", fg = self.theme["orange"])
        elif level == 2:
            mag.configure(text = "2/3", fg = self.theme["orange"])
        else:
            mag.configure(text = "3/3", fg = self.theme["green"])

        bat, _, _ = self.widgets["BAT"]
        bat.config(text = f"{telemetry['battery']}%",
                   fg=self.theme["green"] if telemetry["battery"] > 30 else self.theme["red"])

        gps, _, _ = self.widgets["GPS"]

        if telemetry['hdop'] == 0.0:
            gps_text = f"{telemetry['hdop']:.2f}"
            gps_color = self.theme["red"]

        elif telemetry['hdop'] < 1.5:
            gps_text = f"{telemetry['hdop']:.2f}"
            gps_color = self.theme["green"]

        else:
            gps_text = f"{telemetry['hdop']:.2f}"
            self.theme["orange"]

        gps.config(text = gps_text, fg = gps_color)

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