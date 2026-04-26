import tkinter as tk
from tkinter import ttk
import time

from vcom import Controller, MODE_NAMES

# ---------------------------------------------------------------------------
# Style constants
# ---------------------------------------------------------------------------
_GREEN  = "#27ae60"
_RED    = "#c0392b"
_BLUE   = "#2980b9"

_BTN_ACTIVE_BG = "#27ae60"
_BTN_ACTIVE_FG = "white"
_BTN_NORMAL_BG = "#ecf0f1"
_BTN_NORMAL_FG = "#2c3e50"


class BoatGUI:
    """Main ground-station window."""

    MODES = [(0, "STOP"), (1, "MANUAL"), (2, "AUTO"), (3, "RTL")]

    def __init__(self, root: tk.Tk, controller: Controller):
        self.root  = root
        self.ctrl  = controller
        self._mode_btns: dict[int, tk.Button] = {}

        self.root.title("Vene 2.0 Ground Station")
        self.root.resizable(False, False)

        self._build_ui()
        self._refresh()

    # -----------------------------------------------------------------------
    # UI construction
    # -----------------------------------------------------------------------

    def _build_ui(self) -> None:
        pad = {"padx": 10, "pady": 8}
        self._build_telemetry(row=0, col=0, **pad)
        self._build_status(row=0, col=1, **pad)
        self._build_mode_ctrl(row=1, **pad)
        self._build_manual(row=2, **pad)
        self._build_upload_status(row=3, **pad)

    def _build_telemetry(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Telemetry", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_loc   = ttk.Label(f, text="Location : —")
        self.lbl_head  = ttk.Label(f, text="Heading  : —")
        self.lbl_nav   = ttk.Label(f, text="Target WP: —  |  Mode: —")
        self.lbl_stats = ttk.Label(f, text="Batt: —%  |  HDOP: —  |  Err: —")

        for lbl in (self.lbl_loc, self.lbl_head, self.lbl_nav, self.lbl_stats):
            lbl.pack(anchor="w", pady=1)

    def _build_status(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Connection Status", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_receiver = ttk.Label(f, text="Receiver : OFFLINE", foreground=_RED)
        self.lbl_lora     = ttk.Label(f, text="LoRa     : OFFLINE", foreground=_RED)
        self.lbl_wifi     = ttk.Label(f, text="WiFi     : OFFLINE", foreground=_RED)

        for lbl in (self.lbl_receiver, self.lbl_lora, self.lbl_wifi):
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

        # Throttle
        ttk.Label(f, text="Throttle  −100 → +100").grid(row=0, column=0, sticky="e", padx=6)
        self.scale_throttle = ttk.Scale(f, from_=-100, to=100, orient="horizontal", length=230)
        self.scale_throttle.grid(row=0, column=1)
        self.scale_throttle.set(0)
        self.lbl_throttle_val = ttk.Label(f, text="  0", width=5)
        self.lbl_throttle_val.grid(row=0, column=2)

        # Rudder — boat's turnRudder() expects int8_t −80…+80 (0 = straight ahead)
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
        """
        Route upload panel.  The progressbar maximum is set dynamically so it
        works correctly for any route length.
        """
        f = tk.LabelFrame(self.root, text="Route Upload", padx=10, pady=8)
        f.grid(row=row, column=0, columnspan=2, sticky="nsew", **kw)

        self.lbl_upload = ttk.Label(f, text="—", width=32, anchor="w")
        self.lbl_upload.grid(row=0, column=0, sticky="w", padx=(0, 10))

        self.pb_upload = ttk.Progressbar(
            f, orient="horizontal", length=320, mode="determinate"
        )
        self.pb_upload.grid(row=0, column=1, sticky="ew")
        self.pb_upload["value"]   = 0
        self.pb_upload["maximum"] = 1   # avoids divide-by-zero before first upload

    # -----------------------------------------------------------------------
    # Periodic UI refresh
    # -----------------------------------------------------------------------

    def _refresh(self) -> None:
        self._update_telemetry()
        self._update_status()
        self._update_mode_buttons()
        self._update_upload()
        self._push_manual_values()
        self.root.after(100, self._refresh)

    def _update_telemetry(self) -> None:
        d = self.ctrl.boat_data
        self.lbl_loc.config(text=f"Location : {d['lat']:.6f},  {d['lon']:.6f}")
        self.lbl_head.config(text=f"Heading  : {d['heading']:.1f}°")
        self.lbl_nav.config(
            text=f"Target WP: {d['target_idx']}   |   Mode: {MODE_NAMES.get(d['mode'], '?')}"
        )
        self.lbl_stats.config(
            text=f"Batt: {d['battery']}%   |   HDOP: {d['hdop']:.1f}   |   Err: 0x{d['error']:04X}"
        )

    def _update_status(self) -> None:
        rx = self.ctrl.receiver_connected

        if rx:
            self.lbl_receiver.config(
                text=f"Receiver : ONLINE  ({self.ctrl.serial_port})", foreground=_GREEN
            )
        else:
            self.lbl_receiver.config(text="Receiver : OFFLINE", foreground=_RED)

        # LoRa: last_lora_time is only updated by real boat frames (heartbeat echoes
        # are filtered in _handle_ack), so ONLINE genuinely means the boat is alive.
        if rx and self.ctrl.lora_online:
            self.lbl_lora.config(text="LoRa     : ONLINE",  foreground=_GREEN)
        else:
            self.lbl_lora.config(text="LoRa     : OFFLINE", foreground=_RED)

        if rx and self.ctrl.wifi_online:
            self.lbl_wifi.config(text="WiFi     : ONLINE",  foreground=_GREEN)
        else:
            self.lbl_wifi.config(text="WiFi     : OFFLINE", foreground=_RED)

    def _update_mode_buttons(self) -> None:
        confirmed = self.ctrl.boat_data["mode"]
        for mode_id, btn in self._mode_btns.items():
            if mode_id == confirmed:
                btn.config(bg=_BTN_ACTIVE_BG, fg=_BTN_ACTIVE_FG, relief="sunken", bd=3)
            else:
                btn.config(bg=_BTN_NORMAL_BG, fg=_BTN_NORMAL_FG, relief="raised", bd=2)

    def _update_upload(self) -> None:
        """
        Render upload progress.  Works for any route length because the
        progressbar maximum is always set to upload_total before updating value.
        """
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
            # Leave bar at last known position so user can see where it stopped
            self.lbl_upload.config(
                text=f"Upload failed ✗  (stopped at {current} / {total})", foreground=_RED
            )

        else:  # idle
            self.pb_upload["value"] = 0
            self.pb_upload.config(maximum=1)
            self.lbl_upload.config(text="—", foreground="black")

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
