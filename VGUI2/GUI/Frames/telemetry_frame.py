import math
import tkinter as tk
from GUI.base_frame import BaseFrame
from VCOM.protocol import MODE_NAMES


class TelemetryFrame(BaseFrame):
    METRICS = [
        ("POS", "GPS", "—, —"),
        ("HDG", "Heading", "—°"),
        ("GPS", "GPS HACC", "m"),
        ("MAG", "Mag Accuracy", "—"),
        ("NAV", "Target WP (idx)", "—"),
        ("BAT", "Battery", "—V"),
        ("SIG", "LoRa RSSI", "— dBm"),
    ]

    MAG_ACC_BIT_NAMES = ["HDG_A1", "HDG_A2", "HDG_A3"]

    POS_MODES = ["coords", "home"]

    def build(self):
        self.frame = tk.LabelFrame(
            self.parent, text="Telemetry",
            font=("Segoe UI", 10, "bold"), padx=10, pady=8,
        )
        self.frame.pack(fill="x", pady=(0, 10))

        self.widgets = {}
        self._last_telemetry = None
        self._last_connection = None
        self._pos_mode_idx = 0

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

            if code == "POS":
                desc.config(cursor="hand2")
                value.config(cursor="hand2")
                desc.bind("<Button-1>", self._cycle_pos_mode)
                value.bind("<Button-1>", self._cycle_pos_mode)

            self.widgets[code] = (value, container, desc)

        self.apply_theme(self.theme)

    def _find_bit(self, name: str):
        for bit, (defname, _desc) in self.ctrl.error_defs.items():
            if defname == name:
                return bit
            
        return None

    def _cycle_pos_mode(self, _event=None) -> None:
        self._pos_mode_idx = (self._pos_mode_idx + 1) % len(self.POS_MODES)

        if self._last_telemetry is not None:
            self._refresh(self._last_telemetry, self._last_connection)

    @staticmethod
    def _haversine_m(p1, p2):
        R = 6371000.0
        lat1, lon1 = p1
        lat2, lon2 = p2

        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.asin(min(1.0, math.sqrt(a)))

    @staticmethod
    def _bearing_deg(p1, p2):
        lat1, lon1 = p1
        lat2, lon2 = p2

        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)

        x = math.sin(dlambda) * math.cos(phi2)
        y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)

        return math.degrees(math.atan2(x, y)) % 360

    def update(self, telemetry: dict, connection: dict) -> None:
        self._last_telemetry = telemetry
        self._last_connection = connection
        self._refresh(telemetry, connection)

    def _refresh(self, telemetry: dict, connection: dict) -> None:
        pos, _, pos_desc = self.widgets["POS"]

        if self.POS_MODES[self._pos_mode_idx] == "home":
            pos_desc.config(text = "Home")

            if telemetry["lat"] == 0.0 and telemetry["lon"] == 0.0:
                pos.config(text = "NO GPS FIX", fg = self.theme["red"])
            elif not connection.get("home_set", False):
                pos.config(text = "NO HOME SET", fg = self.theme["red"])
            else:
                cur = (telemetry["lat"], telemetry["lon"])
                home = (connection["home_lat"], connection["home_lon"])
                dist_m = self._haversine_m(cur, home)
                brg_deg = self._bearing_deg(cur, home)

                if dist_m >= 1000:
                    dist_str = f"{dist_m / 1000:.2f} km"
                else:
                    dist_str = f"{dist_m:.0f} m"

                pos.config(text = f"{dist_str} / {brg_deg:.0f}°", fg = self.theme["fg"])
        else:
            pos_desc.config(text = "GPS")

            if telemetry["lat"] != 0.0 and telemetry["lon"] != 0.0 and telemetry["hAcc"] < 4.0:
                pos.config(text=f"{telemetry['lat']:.6f} N, {telemetry['lon']:.6f} E", fg = self.theme["fg"])
            elif telemetry["lat"] != 0.0 and telemetry["lon"] != 0.0:
                pos.config(text=f"{telemetry['lat']:.6f} N, {telemetry['lon']:.6f} E", fg = self.theme["red"])
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
            mag.configure(text = "2/3", fg = self.theme["green"])
        else:
            mag.configure(text = "3/3", fg = self.theme["green"])

        bat, _, _ = self.widgets["BAT"]

        c1 = telemetry["battery_c1"]
        c2 = telemetry["battery_c2"]
        c3 = telemetry["battery_c3"]

        bat_text = f"{c1} / {c2} / {c3} ({(c1 + c3 + c3):.2f}) V"

        bat.config(text = bat_text,
                   fg=self.theme["green"] if telemetry["battery_c1"] > 30 else self.theme["red"])

        gps, _, _ = self.widgets["GPS"]

        if telemetry['hAcc'] == 0.0 or telemetry["hAcc"] == 25.5:
            gps_text = "N/A"
            gps_color = self.theme["red"]

        elif telemetry['hAcc'] < 3.0:
            gps_text = f"{telemetry['hAcc']:.1f} m"
            gps_color = self.theme["green"]

        else:
            gps_text = f"{telemetry['hAcc']:.1f} m"
            gps_color = self.theme["orange"]

        gps.config(text = gps_text, fg = gps_color)


        self.widgets["SIG"][0].config(text=f"{telemetry['signal']} dBm")

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