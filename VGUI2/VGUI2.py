import tkinter as tk
from tkinter import ttk, scrolledtext
import time
import os
from typing import Dict, Set, Tuple, Optional

from PIL import Image, ImageDraw, ImageTk
import tkintermapview

from vcom.vcom import Controller, MODE_NAMES

# Quick test gui

_GREEN  = "#27ae60"
_RED    = "#c0392b"
_BLUE   = "#2980b9"
_ORANGE = "#e67e22"

_BTN_ACTIVE_BG = "#27ae60"
_BTN_ACTIVE_FG = "white"
_BTN_NORMAL_BG = "#ecf0f1"
_BTN_NORMAL_FG = "#2c3e50"

_DEFAULT_BIT_NAMES: Dict[int, Tuple[str, str]] = {
    0: ("START_OK",        "Init completed successfully"),
    1: ("GPS_MODULE_FAIL", "Could not connect to the gps module"),
}


def _load_error_defs(path: str = "error_codes.txt") -> Dict[int, Tuple[str, str]]:
    defs: Dict[int, Tuple[str, str]] = dict(_DEFAULT_BIT_NAMES)

    if not os.path.isfile(path):
        return defs

    with open(path, encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            bit_str, rest = line.split("=", 1)
            bit_str = bit_str.strip().upper()
            if not bit_str.startswith("BIT"):
                continue
            try:
                bit_num = int(bit_str[3:])
            except ValueError:
                continue
            if ":" in rest:
                name, desc = rest.split(":", 1)
            else:
                name, desc = rest, ""
            defs[bit_num] = (name.strip(), desc.strip())

    return defs


class BoatGUI:
    MODES = [(0, "STOP"), (1, "MANUAL"), (2, "AUTO"), (3, "RTL")]

    _ICON_SIZE = 52   # boat sprite square (px)
    _HOME_ICON_SIZE = 30   # home pin square (px)

    def __init__(self, root: tk.Tk, controller: Controller):
        self.root = root
        self.ctrl = controller

        self._error_defs = _load_error_defs()
        self._prev_error: int = 0
        self._mode_btns: dict[int, tk.Button] = {}

        # ── Map state ──────────────────────────────────────────────────────────
        self._boat_marker                          = None
        self._boat_icon_cache: dict[int, ImageTk.PhotoImage] = {}
        self._map_initialised                      = False
        self._map_fallback_espoo_done              = False
        self._map_last_heading: int                = -1

        self._home_marker                          = None
        self._map_last_home: Tuple[float, float]   = (0.0, 0.0)
        self._home_icon: Optional[ImageTk.PhotoImage] = None  # created in _build_map
        # ──────────────────────────────────────────────────────────────────────

        self.root.title("Vene 2.0 Ground Station")
        self.root.resizable(False, False)

        self._build_ui()

        # Register callbacks
        self.ctrl.on_error_change  = self._on_error_change_cb
        self.ctrl.on_home_received = self._on_home_received_cb

        self._refresh()

    # ── Layout ────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        pad = {"padx": 10, "pady": 8}
        self._build_telemetry(row=0, col=0, **pad)
        self._build_status(row=0, col=1, **pad)
        self._build_mode_ctrl(row=1, **pad)
        self._build_manual(row=2, **pad)
        self._build_upload_status(row=3, **pad)
        self._build_error_panel(row=4, **pad)
        self._build_map(row=5, **pad)

    def _build_telemetry(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Telemetry", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_loc   = ttk.Label(f, text="Location : —")
        self.lbl_head  = ttk.Label(f, text="Heading  : —")
        self.lbl_nav   = ttk.Label(f, text="Target WP: —  |  Mode: —")
        self.lbl_stats = ttk.Label(f, text="Batt: —%  |  HDOP: —  |  RSSI: — dBm  |  Err: —")

        for lbl in (self.lbl_loc, self.lbl_head, self.lbl_nav, self.lbl_stats):
            lbl.pack(anchor="w", pady=1)

    def _build_status(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Connection Status", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_receiver = ttk.Label(f, text="Receiver : OFFLINE", foreground=_RED)
        self.lbl_lora     = ttk.Label(f, text="LoRa     : OFFLINE", foreground=_RED)
        self.lbl_wifi     = ttk.Label(f, text="WiFi     : OFFLINE", foreground=_RED)
        self.lbl_home_status = ttk.Label(f, text="Home WP  : NOT SET", foreground=_RED)

        for lbl in (self.lbl_receiver, self.lbl_lora, self.lbl_wifi, self.lbl_home_status):
            lbl.pack(anchor="w", pady=1)

    def _build_mode_ctrl(self, row, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Mode Control", padx=10, pady=8)
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        for col, (mode_id, label) in enumerate(self.MODES):
            btn = tk.Button(
                f, text=label, width=10,
                bg=_BTN_NORMAL_BG, fg=_BTN_NORMAL_FG,
                relief="raised", bd=2,
                command=lambda m=mode_id: self.ctrl.set_mode(m),
            )
            btn.grid(row=0, column=col, padx=6, pady=4)
            self._mode_btns[mode_id] = btn

        ttk.Button(
            f, text="Upload Test Route",
            command=self.ctrl.send_test_route,
        ).grid(row=0, column=len(self.MODES), padx=20)

    def _build_manual(self, row, **kw) -> None:
        f = tk.LabelFrame(
            self.root,
            text="Manual Controls  (Receiver ESP → WiFi UDP → Boat)",
            padx=10, pady=8,
        )
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        ttk.Label(f, text="Throttle  −100 → +100").grid(row=0, column=0, sticky="e", padx=6)
        self.scale_throttle = ttk.Scale(f, from_=-100, to=100, orient="horizontal", length=230)
        self.scale_throttle.grid(row=0, column=1)
        self.scale_throttle.set(0)
        self.lbl_throttle_val = ttk.Label(f, text="  0", width=5)
        self.lbl_throttle_val.grid(row=0, column=2)

        ttk.Label(f, text="Rudder     −80 →  +80").grid(row=1, column=0, sticky="e", padx=6)
        self.scale_rudder = ttk.Scale(f, from_=-80, to=80, orient="horizontal", length=230)
        self.scale_rudder.grid(row=1, column=1)
        self.scale_rudder.set(0)
        self.lbl_rudder_val = ttk.Label(f, text="  0", width=5)
        self.lbl_rudder_val.grid(row=1, column=2)

        ttk.Button(f, text="Reset Neutral", command=self._reset_controls).grid(
            row=0, column=3, rowspan=2, padx=14
        )

    def _build_upload_status(self, row, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Route Upload", padx=10, pady=8)
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        self.lbl_upload = ttk.Label(f, text="—", width=32, anchor="w")
        self.lbl_upload.grid(row=0, column=0, sticky="w", padx=(0, 10))

        self.pb_upload = ttk.Progressbar(
            f, orient="horizontal", length=320, mode="determinate"
        )
        self.pb_upload.grid(row=0, column=1, sticky="ew")
        self.pb_upload["value"]   = 0
        self.pb_upload["maximum"] = 1

    def _build_error_panel(self, row, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Error Codes", padx=10, pady=8)
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        left = tk.Frame(f)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 12))

        ttk.Label(left, text="Active errors:").pack(anchor="w")

        self.lbl_error_hex = ttk.Label(
            left, text="0x0000", font=("Courier", 14, "bold"), foreground=_GREEN
        )
        self.lbl_error_hex.pack(anchor="w", pady=(2, 4))

        self.lb_errors = tk.Listbox(
            left, width=36, height=6,
            font=("Courier", 9),
            selectmode="browse",
            activestyle="none",
        )
        self.lb_errors.pack(anchor="w")

        ttk.Label(
            left, text="(place error_codes.txt next to the script to customise names)",
            font=("TkDefaultFont", 7), foreground="grey",
        ).pack(anchor="w", pady=(2, 0))

        right = tk.Frame(f)
        right.grid(row=0, column=1, sticky="nsew")

        ttk.Label(right, text="Error event log:").pack(anchor="w")

        self.txt_error_log = scrolledtext.ScrolledText(
            right, width=46, height=7,
            font=("Courier", 9),
            state="disabled",
            wrap="word",
        )
        self.txt_error_log.pack(fill="both", expand=True)

        self.txt_error_log.tag_configure("set",     foreground=_RED)
        self.txt_error_log.tag_configure("cleared", foreground=_GREEN)
        self.txt_error_log.tag_configure("ts",      foreground="grey")

        ttk.Button(
            right, text="Reset Errors on Boat",
            command=self.ctrl.reset_errors,
        ).pack(anchor="e", pady=(4, 0))

    # ── Map panel ──────────────────────────────────────────────────────────────

    def _build_map(self, row, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Map", padx=4, pady=4)
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        # Toolbar
        toolbar = tk.Frame(f)
        toolbar.pack(fill="x", pady=(0, 4))

        self._follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            toolbar, text="Follow boat", variable=self._follow_var,
        ).pack(side="left")
        ttk.Button(
            toolbar, text="Centre now", command=self._centre_on_boat,
        ).pack(side="left", padx=8)

        ttk.Separator(toolbar, orient="vertical").pack(side="left", fill="y", padx=6)

        ttk.Button(
            toolbar, text="Request Home WP",
            command=self.ctrl.request_home,
        ).pack(side="left")

        ttk.Label(
            toolbar,
            text="  Right-click on map to set Home WP",
            foreground="grey",
            font=("TkDefaultFont", 8),
        ).pack(side="left", padx=6)

        # Map widget
        self.map_widget = tkintermapview.TkinterMapView(
            f, width=660, height=340, corner_radius=0,
        )
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_zoom(16)

        # Right-click → set home WP at clicked location
        self.map_widget.add_right_click_menu_command(
            label="Set as Home WP",
            command=self._map_right_click_set_home,
            pass_coords=True,
        )

        # ── Boat sprite ────────────────────────────────────────────────────────
        try:
            raw = Image.open("icons/vene1.png").convert("RGBA")
        except (FileNotFoundError, OSError):
            raw = self._make_fallback_icon()

        raw.thumbnail((self._ICON_SIZE, self._ICON_SIZE), Image.LANCZOS)
        canvas = Image.new("RGBA", (self._ICON_SIZE, self._ICON_SIZE), (0, 0, 0, 0))
        ox = (self._ICON_SIZE - raw.width)  // 2
        oy = (self._ICON_SIZE - raw.height) // 2
        canvas.paste(raw, (ox, oy), raw)
        self._boat_img_orig = canvas

        # ── Home pin icon ──────────────────────────────────────────────────────
        self._home_icon = ImageTk.PhotoImage(self._make_home_pin_icon())

    # ── Icon helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _make_fallback_icon() -> Image.Image:
        s = 48
        img  = Image.new("RGBA", (s, s), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)
        cx   = s // 2
        draw.polygon(
            [(cx, 2), (s - 6, s - 8), (cx, s - 16), (6, s - 8)],
            fill=(41, 128, 185, 220),
            outline=(21, 67, 96, 255),
        )
        return img

    @staticmethod
    def _make_home_pin_icon() -> Image.Image:
        s  = 30
        cx = s // 2
        img  = Image.new("RGBA", (s, s), (0, 0, 0, 0))
        draw = ImageDraw.Draw(img)

        # Green circle (top of pin)
        draw.ellipse([3, 1, s - 3, s - 3 - (s // 3)], fill=(39, 174, 96, 230))
        # Green triangle (bottom of pin)
        draw.polygon(
            [(3, s - 3 - (s // 3)), (s - 3, s - 3 - (s // 3)), (cx, s - 2)],
            fill=(39, 174, 96, 230),
        )
        # White dot centre
        r = 4
        draw.ellipse([cx - r, 5, cx + r, 5 + r * 2], fill=(255, 255, 255, 210))

        return img

    def _get_boat_icon(self, heading: int) -> ImageTk.PhotoImage:
        if heading not in self._boat_icon_cache:
            rotated = self._boat_img_orig.rotate(-heading, resample=Image.BICUBIC)
            self._boat_icon_cache[heading] = ImageTk.PhotoImage(rotated)
        return self._boat_icon_cache[heading]

    # ── Map interaction ────────────────────────────────────────────────────────

    def _centre_on_boat(self) -> None:
        d = self.ctrl.boat_data
        if d["lat"] or d["lon"]:
            self.map_widget.set_position(d["lat"], d["lon"])

    def _map_right_click_set_home(self, coords: tuple[float, float]) -> None:
        lat, lon = coords  
        self.ctrl.set_home(lat, lon)

    # ── Periodic refresh ───────────────────────────────────────────────────────

    def _refresh(self) -> None:
        self._update_telemetry()
        self._update_status()
        self._update_mode_buttons()
        self._update_upload()
        self._push_manual_values()
        self._update_error_panel()
        self._update_map()
        self.root.after(100, self._refresh)

    def _update_telemetry(self) -> None:
        d = self.ctrl.boat_data
        self.lbl_loc.config(text=f"Location : {d['lat']:.6f},  {d['lon']:.6f}")
        self.lbl_head.config(text=f"Heading  : {d['heading']:.1f}°")
        self.lbl_nav.config(
            text=f"Target WP: {d['target_idx']}   |   Mode: {MODE_NAMES.get(d['mode'], '?')}"
        )
        self.lbl_stats.config(
            text=(
                f"Batt: {d['battery']}%   |   HDOP: {d['hdop']:.1f}   |"
                f"   RSSI: {d['signal']} dBm   |"
                f"   Err: 0x{d['error']:04X}"
            )
        )

    def _update_status(self) -> None:
        rx = self.ctrl.receiver_connected

        if rx:
            self.lbl_receiver.config(
                text=f"Receiver : ONLINE  ({self.ctrl.serial_port})", foreground=_GREEN
            )
        else:
            self.lbl_receiver.config(text="Receiver : OFFLINE", foreground=_RED)

        self.lbl_lora.config(
            text="LoRa     : ONLINE"  if (rx and self.ctrl.lora_online) else "LoRa     : OFFLINE",
            foreground=_GREEN if (rx and self.ctrl.lora_online) else _RED,
        )
        self.lbl_wifi.config(
            text="WiFi     : ONLINE"  if (rx and self.ctrl.wifi_online) else "WiFi     : OFFLINE",
            foreground=_GREEN if (rx and self.ctrl.wifi_online) else _RED,
        )

        # Home WP status
        if self.ctrl.home_set:
            self.lbl_home_status.config(
                text=f"Home WP  : {self.ctrl.home_lat:.5f}, {self.ctrl.home_lon:.5f}",
                foreground=_GREEN,
            )
        else:
            self.lbl_home_status.config(
                text="Home WP  : NOT SET",
                foreground=_RED,
            )

    def _update_mode_buttons(self) -> None:
        confirmed = self.ctrl.boat_data["mode"]
        for mode_id, btn in self._mode_btns.items():
            if mode_id == confirmed:
                btn.config(bg=_BTN_ACTIVE_BG, fg=_BTN_ACTIVE_FG, relief="sunken", bd=3)
            else:
                btn.config(bg=_BTN_NORMAL_BG, fg=_BTN_NORMAL_FG, relief="raised", bd=2)

    def _update_upload(self) -> None:
        status  = self.ctrl.upload_status
        current = self.ctrl.upload_current
        total   = self.ctrl.upload_total

        if status == "uploading":
            self.pb_upload.config(maximum=max(total, 1))
            self.pb_upload["value"] = current
            self.lbl_upload.config(
                text=f"Uploading…  {current} / {total} WP", foreground=_BLUE
            )
        elif status == "done":
            self.pb_upload["value"] = total
            self.lbl_upload.config(
                text=f"Upload complete ✓  ({total} WP)", foreground=_GREEN
            )
        elif status == "failed":
            self.lbl_upload.config(
                text=f"Upload failed ✗  (stopped at {current} / {total})", foreground=_RED
            )
        else:
            self.pb_upload["value"] = 0
            self.pb_upload.config(maximum=1)
            self.lbl_upload.config(text="—", foreground="black")

    def _update_error_panel(self) -> None:
        error = self.ctrl.boat_data["error"]

        self.lbl_error_hex.config(
            text=f"0x{error:08X}",
            foreground=_GREEN if error == 0 else _ORANGE,
        )

        self.lb_errors.delete(0, "end")
        if error == 0:
            self.lb_errors.insert("end", "  — no active errors —")
            self.lb_errors.itemconfig(0, foreground=_GREEN)
        else:
            for bit in range(32):
                if error >> bit & 1:
                    name, desc = self._error_defs.get(bit, (f"BIT{bit}", ""))
                    entry = f"  BIT{bit:2d}  {name}"
                    if desc:
                        entry += f"  ({desc})"
                    self.lb_errors.insert("end", entry)
                    self.lb_errors.itemconfig("end", foreground=_RED)

    def _update_map(self) -> None:
        d   = self.ctrl.boat_data
        lat = d["lat"]
        lon = d["lon"]

        hlat = self.ctrl.home_lat
        hlon = self.ctrl.home_lon
        hset = self.ctrl.home_set

        # ── Fallback Positioning If No Boat Position ──────────────────────────
        if lat == 0.0 and lon == 0.0:
            if not self._map_initialised:
                if hset and (hlat != 0.0 or hlon != 0.0):
                    # Center on Home WP only when it updates or is first synced
                    if (hlat, hlon) != self._map_last_home:
                        self.map_widget.set_position(hlat, hlon)
                elif not self._map_fallback_espoo_done:
                    # Default map location: Otaniemi, Espoo, Finland
                    self.map_widget.set_position(60.1849, 24.8250)
                    self._map_fallback_espoo_done = True

        # ── Boat marker ───────────────────────────────────────────────────────
        if lat != 0.0 or lon != 0.0:
            heading = int(round(d["heading"])) % 360
            icon    = self._get_boat_icon(heading)

            if self._boat_marker is None or heading != self._map_last_heading:
                if self._boat_marker is not None:
                    self._boat_marker.delete()
                self._boat_marker      = self.map_widget.set_marker(lat, lon, icon=icon, text="")
                self._map_last_heading = heading
            else:
                self._boat_marker.set_position(lat, lon)

            if not self._map_initialised or self._follow_var.get():
                self.map_widget.set_position(lat, lon)
                self._map_initialised = True

        # ── Home WP marker ────────────────────────────────────────────────────
        if hset and (hlat != 0.0 or hlon != 0.0):
            if self._home_marker is None or (hlat, hlon) != self._map_last_home:
                if self._home_marker is not None:
                    self._home_marker.delete()
                self._home_marker = self.map_widget.set_marker(
                    hlat, hlon,
                    icon=self._home_icon,
                    text="Home",
                )
                self._map_last_home = (hlat, hlon)
        else:
            if self._home_marker is not None:
                self._home_marker.delete()
                self._home_marker   = None
                self._map_last_home = (0.0, 0.0)

    # ── Manual controls ────────────────────────────────────────────────────────

    def _push_manual_values(self) -> None:
        t = self.scale_throttle.get()
        r = self.scale_rudder.get()
        self.ctrl.manual_throttle = t
        self.ctrl.manual_rudder   = r
        self.lbl_throttle_val.config(text=f"{t:+.0f}")
        self.lbl_rudder_val.config(text=f"{r:+.0f}")

    def _reset_controls(self) -> None:
        self.scale_throttle.set(0)
        self.scale_rudder.set(0)

    # ── Callbacks (from vcom threads -> scheduled on main thread) ──────────────

    def _on_error_change_cb(self, new_bits: Set[int], cleared_bits: Set[int]) -> None:
        self.root.after(0, self._log_error_changes, new_bits, cleared_bits)

    def _on_home_received_cb(self, lat: float, lon: float) -> None:
        self.root.after(0, self._update_status)

    def _log_error_changes(self, new_bits: Set[int], cleared_bits: Set[int]) -> None:
        ts = time.strftime("%H:%M:%S")

        self.txt_error_log.config(state="normal")

        for bit in sorted(new_bits):
            name, desc = self._error_defs.get(bit, (f"BIT{bit}", ""))
            self.txt_error_log.insert("end", f"[{ts}] ", "ts")
            self.txt_error_log.insert("end", f"SET     BIT{bit} {name}", "set")
            if desc:
                self.txt_error_log.insert("end", f" — {desc}")
            self.txt_error_log.insert("end", "\n")

        for bit in sorted(cleared_bits):
            name, desc = self._error_defs.get(bit, (f"BIT{bit}", ""))
            self.txt_error_log.insert("end", f"[{ts}] ", "ts")
            self.txt_error_log.insert("end", f"CLEARED BIT{bit} {name}", "cleared")
            self.txt_error_log.insert("end", "\n")

        self.txt_error_log.see("end")
        self.txt_error_log.config(state="disabled")


if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"

    controller = Controller(serial_port=port)
    root = tk.Tk()
    app  = BoatGUI(root, controller)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()