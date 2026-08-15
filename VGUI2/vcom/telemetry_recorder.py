import math
import os
import time

from typing import Dict, Optional, Tuple

RECORDING_DIR = os.path.join(os.getcwd(), "Telemetry")

_EARTH_RADIUS_M = 6371000.0
 
MPS_TO_KN = 1.943844

def _harvesine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)

    dphi = math.radians(lat2 - lat1)

    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)

    return 2 * _EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))

class TelemetryRecorder:
    FIELDS: Tuple[str, ...] = (
        "lat", "lon", "heading", "mode", "target_idx", "battery", "hAcc", "rssi", "error",
    )

    MAX_FIX_HACC_M = 4.0

    def __init__(self, error_defs: Optional[Dict[int, Tuple[str, str]]] = None, 
                 out_dir: str = RECORDING_DIR):
        self.error_defs = error_defs or {}
        self.out_dir = out_dir

        self.active = None

        self._fh = None
        self._path: Optional[str] = None

        self._start_time = 0.0
        self._last_sample = None

        self._last_fix: Optional[Tuple[float, float, float]] = None # lat, lon, t

        self._total_distance_m = 0.0
        self._top_speed_mps = 0.0

    @property
    def path(self) -> Optional[str]:
        return self._path

    @property
    def start_time(self) -> float:
        return self._start_time

    def start(self) -> str:
        if self.active:
            return self.path

        os.makedirs(self.out_dir, exist_ok = True)

        fname = time.strftime("telemetry_%Y-%m-%d_%H-%M-%S.txt")
        self._path = os.path.join(self.out_dir, fname)

        self._fh = open(self._path, "w", encoding = "utf-8")
        self._fh.write(f"# Recording started {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        self._fh.flush()

        self._start_time = time.time()
        self._last_sample = None
        self._last_fix = None
        self._total_distance_m = 0.0
        self._top_speed_mps = 0.0

        self.active = True
        return self._path

    def stop(self) -> Optional[str]:
        if not self.active:
            return None

        self.active = False
        duration_s = time.time() - self._start_time

        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass

        path = self._path
        self._write_summary(duration_s)

        self._fh = None
        self._path = None

        return path

    def record(self, telemetry: dict, connection: dict) -> None:
        if not self.active or self._fh is None:
            return

        sample = tuple(telemetry.get(f) for f in self.FIELDS)

        if sample == self._last_sample:
            return # Nothing new

        self._last_sample = sample
        now = time.time()

        self._update_distance(telemetry, now)

        try:
            self._fh.write(self._format_line(now, telemetry, connection))
            self._fh.flush()
        except Exception:
            pass

    def _update_distance(self, telemetry: dict, now: float) -> None:
        lat, lon = telemetry.get("lat", 0.0), telemetry.get("lon", 0.0)
        hacc = telemetry.get("hAcc", 0.0)

        have_fix = (lat != 0.0 or lon != 0.0) and hacc < self.MAX_FIX_HACC_M

        if not have_fix:
            return

        if self._last_fix is not None:
            plat, plon, pt = self._last_fix
            dt = now - pt

            if dt > 0:
                dist = _harvesine_m(plat, plon, lat, lon)
                self._total_distance_m += dist

                self._top_speed_mps = max(self._top_speed_mps, dist / dt)

        self._last_fix = (lat, lon, now)

    def _format_line(self, ts: float, telemetry: dict, connection: dict) -> str:
        t_str = time.strftime("%H:%M:%S", time.localtime(ts)) + f".{int(ts * 1000) % 1000:03d}"
 
        lat, lon = telemetry.get("lat", 0.0), telemetry.get("lon", 0.0)
        pos = f"{lat:.6f},{lon:.6f}" if (lat != 0.0 or lon != 0.0) else "NO FIX"
 
        err = telemetry.get("error", 0)
        err_names = [self.error_defs[b][0] for b in range(32)
                     if (err >> b & 1) and b in self.error_defs]
        err_str = ",".join(err_names) if err_names else "-"
 
        return (
            f"{t_str}  POS={pos}  HDG={telemetry.get('heading', 0.0):.1f}  "
            f"MODE={telemetry.get('mode', 0)}  WP={telemetry.get('target_idx', 0)}  "
            f"BAT={telemetry.get('battery', 0.0):.1f}V  "
            f"HACC={telemetry.get('hAcc', 0.0):.1f}m  RSSI={telemetry.get('signal', 0)}dBm  "
            f"ERR={err:08X}[{err_str}]  "
            f"LORA={'Y' if connection.get('lora_online') else 'N'}  "
            f"WIFI={'Y' if connection.get('wifi_online') else 'N'}\n"
        )

    def _write_summary(self, duration_s: float) -> None:
        if not self._path or not os.path.isfile(self._path):
            return
 
        dist_km = self._total_distance_m / 1000.0
        avg_speed_mps = (self._total_distance_m / duration_s) if duration_s > 0 else 0.0
 
        header = (
            f"# Duration     : {self._fmt_duration(duration_s)}\n"
            f"# Distance     : {dist_km:.3f} km  ({self._total_distance_m:.0f} m)\n"
            f"# Avg speed    : {avg_speed_mps * MPS_TO_KN:.2f} kn  ({avg_speed_mps:.2f} m/s)\n"
            f"# Top speed    : {self._top_speed_mps * MPS_TO_KN:.2f} kn  ({self._top_speed_mps:.2f} m/s)\n"
            "# ----------------------------\n\n"
        )
 
        with open(self._path, "r", encoding = "utf-8") as f:
            body = f.read()
 
        with open(self._path, "w", encoding = "utf-8") as f:
            f.write(header)
            f.write(body)

    @staticmethod
    def _fmt_duration(seconds: float) -> str:
        seconds = int(seconds)
        h, rem = divmod(seconds, 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{s:02d}"