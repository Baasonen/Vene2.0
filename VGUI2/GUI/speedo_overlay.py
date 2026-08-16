import tkinter as tk
from tkinter import ttk
from typing import Tuple, Optional
import math
import time
from collections import deque

MPS_TO_KN = 1.9438444924574
MPS_TO_KMH = 3.6
AVG_WINDOW_OPTIONS = [10, 15, 20, 30]
DEFAULT_AVG_WINDOW = 15
MAX_FIX_BUFFER_S = max(AVG_WINDOW_OPTIONS) + 5

SPEED_UNITS = ["kn", "km/h", "m/s"]
UNIT_FACTORS = {"kn": MPS_TO_KN, "km/h": MPS_TO_KMH, "m/s": 1.0}

OVERLAY_X = -8
OVERLAY_TOP_Y = 8

class SpeedoOverlay:
    def __init__(self, parent):
        self._parent = parent

        self._last_fix_latlon: Optional[Tuple[float, float]] = None
        self._last_fix_time: float = 0.0
        self._fix_buffer: deque = deque()
        self._top_speed_mps: float = 0.0
        self._cur_speed_mps: Optional[float] = None
        self._avg_window_idx: int = AVG_WINDOW_OPTIONS.index(DEFAULT_AVG_WINDOW)
        self._unit_idx: int = 0

        self.frame = tk.Frame(parent, bd = 1, relief = "solid")
        self.frame.place(in_ = parent, relx = 1.0, rely = 0.0, x = OVERLAY_X, y = OVERLAY_TOP_Y, anchor = "ne")

        self.lbl_speed_caption = tk.Label(self.frame, text = "GPS SPEED", font = ("Segoe UI", 8, "bold"))
        self.lbl_speed_caption.pack(padx = 10, pady = (6, 0))

        self.speed_row = tk.Frame(self.frame)
        self.speed_row.pack(padx = 10)

        self.lbl_speed_num = tk.Label(self.speed_row, text = "--", font = ("Segoe UI", 18, "bold"), cursor = "hand2")
        self.lbl_speed_num.pack(side = "left")
        self.lbl_speed_num.bind("<Button-1>", self._cycle_unit)

        self.lbl_speed_unit = tk.Label(self.speed_row, text = "kn", font = ("Segoe UI", 11), cursor = "hand2")
        self.lbl_speed_unit.pack(side = "left", padx = (3, 0), pady = (6, 0))
        self.lbl_speed_unit.bind("<Button-1>", self._cycle_unit)

        self.lbl_top_val = tk.Label(self.frame, text = "Top: -- kn", font = ("Segoe UI", 11))
        self.lbl_top_val.pack(padx = 10, pady = (0, 4))

        self.win_row = tk.Frame(self.frame)
        self.win_row.pack(padx = 8, pady = (0, 4))

        ttk.Button(self.win_row, text = "-", width = 2,
                   command = self._decrease_avg_window).pack(side = "left")

        self.lbl_avg_window = tk.Label(self.win_row, font = ("Segoe UI", 11), width = 7)
        self.lbl_avg_window.pack(side = "left", padx = 2)

        ttk.Button(self.win_row, text = "+", width = 2,
                   command = self._increase_avg_window).pack(side = "left")

        ttk.Button(self.frame, text = "Reset Top",
                  command = self._reset_top_speed).pack(padx = 8, pady = (0, 8), fill = "x")

        self._update_avg_window_label()
        self.frame.lift()

    def _increase_avg_window(self) -> None:
        self._avg_window_idx = min(len(AVG_WINDOW_OPTIONS) - 1, self._avg_window_idx + 1)
        self._update_avg_window_label()

    def _decrease_avg_window(self) -> None:
        self._avg_window_idx = max(0, self._avg_window_idx - 1)
        self._update_avg_window_label()

    def _update_avg_window_label(self) -> None:
        self.lbl_avg_window.config(text = f"{AVG_WINDOW_OPTIONS[self._avg_window_idx]} s")

    def _reset_top_speed(self) -> None:
        self._top_speed_mps = 0.0
        self._refresh_speed_labels()

    def _cycle_unit(self, _event = None) -> None:
        self._unit_idx = (self._unit_idx + 1) % len(SPEED_UNITS)
        self._refresh_speed_labels()

    def _refresh_speed_labels(self) -> None:
        unit = SPEED_UNITS[self._unit_idx]
        factor = UNIT_FACTORS[unit]

        if self._cur_speed_mps is None:
            self.lbl_speed_num.config(text = "--")
        else:
            self.lbl_speed_num.config(text = f"{self._cur_speed_mps * factor:0.1f}")

        self.lbl_speed_unit.config(text = unit)

        self.lbl_top_val.config(text = f"Top: {self._top_speed_mps * factor:0.1f} {unit}")

    @staticmethod
    def _haversine_m(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        R = 6371000.0
        lat1, lon1 = p1
        lat2, lon2 = p2

        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.asin(min(1.0, math.sqrt(a)))

    def _register_fix(self, coord: Tuple[float, float]) -> None:
        now = time.monotonic()
        dt = now - self._last_fix_time

        if dt > 0:
            dist_m = self._haversine_m(self._last_fix_latlon, coord)
            self._fix_buffer.append((now, dist_m, dt))

        self._last_fix_latlon = coord
        self._last_fix_time = now
        self._cur_speed_mps = self._compute_avg_speed_mps()

        if self._cur_speed_mps is not None and self._cur_speed_mps > self._top_speed_mps:
            self._top_speed_mps = self._cur_speed_mps

    def _prune_fix_buffer(self) -> None:
        cutoff = time.monotonic() - MAX_FIX_BUFFER_S

        while self._fix_buffer and self._fix_buffer[0][0] < cutoff:
            self._fix_buffer.popleft()

    def _compute_avg_speed_mps(self) -> float:
        if not self._fix_buffer:
            return 0.0

        now = time.monotonic()
        cutoff = now - AVG_WINDOW_OPTIONS[self._avg_window_idx]

        total_dist = 0.0
        total_time = 0.0

        for t, dist, dt in self._fix_buffer:
            if t >= cutoff:
                total_dist += dist
                total_time += dt

        if total_time <= 0:
            return 0.0

        return total_dist / total_time

    def update(self, telemetry: dict) -> None:
        lat, lon = telemetry["lat"], telemetry["lon"]
        have_fix = lat != 0.0 or lon != 0.0

        if have_fix:
            coord = (lat, lon)

            if self._last_fix_latlon is None:
                self._last_fix_latlon = coord
                self._last_fix_time = time.monotonic()
            elif coord != self._last_fix_latlon:
                self._register_fix(coord)

        self._prune_fix_buffer()

        #self._cur_speed_mps = self._compute_avg_speed_mps() if have_fix else None

        self._refresh_speed_labels()

    def set_visible(self, visible: bool) -> None:
        if visible:
            self.frame.place(in_ = self._parent, relx = 1.0, rely = 0.0, x = OVERLAY_X, y = OVERLAY_TOP_Y, anchor = "ne")
        else:
            self.frame.place_forget()

    def apply_theme(self, theme: dict) -> None:
        self.frame.config(bg = theme["panel_bg"], highlightbackground = theme["border"],
                          highlightthickness = 1)
        self.win_row.config(bg = theme["panel_bg"])
        self.speed_row.config(bg = theme["panel_bg"])
        self.lbl_speed_caption.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.lbl_speed_num.config(bg = theme["panel_bg"], fg = theme["fg"])
        self.lbl_speed_unit.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.lbl_top_val.config(bg = theme["panel_bg"], fg = theme["fg_dim"])
        self.lbl_avg_window.config(bg = theme["panel_bg"], fg = theme["fg_dim"])