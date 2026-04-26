"""
boat_gui.py  –  Vene 2.0 Ground Station GUI
============================================
Tkinter front-end for the BoatController module.

Run directly:
    python boat_gui.py

Or embed in a larger application:
    from boat_controller import BoatController
    from boat_gui import BoatGUI
    import tkinter as tk

    ctrl = BoatController(serial_port="COM4")
    root = tk.Tk()
    app  = BoatGUI(root, ctrl)
    root.mainloop()
    ctrl.stop()
"""

import tkinter as tk
from tkinter import ttk
import time

from boat_controller import BoatController, MODE_NAMES

# ---------------------------------------------------------------------------
# Style helpers
# ---------------------------------------------------------------------------
_GREEN   = "#27ae60"
_ORANGE  = "#e67e22"
_RED     = "#c0392b"
_GREY    = "#7f8c8d"

_BTN_ACTIVE_BG  = "#27ae60"   # highlighted mode button
_BTN_ACTIVE_FG  = "white"
_BTN_NORMAL_BG  = "#ecf0f1"
_BTN_NORMAL_FG  = "#2c3e50"


def _indicator(label: ttk.Label, text: str, colour: str) -> None:
    label.config(text=text, foreground=colour)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class BoatGUI:
    """
    Main ground-station window.

    Parameters
    ----------
    root        : tk.Tk or tk.Toplevel
    controller  : BoatController instance (already running)
    """

    MODES = [(0, "STOP"), (1, "MANUAL"), (2, "AUTO"), (3, "RTL")]

    def __init__(self, root: tk.Tk, controller: BoatController):
        self.root       = root
        self.ctrl       = controller
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

        # Row 0 — Telemetry | Connection status
        self._build_telemetry(row=0, col=0, **pad)
        self._build_status(row=0, col=1, **pad)

        # Row 1 — Mode control bar
        self._build_mode_ctrl(row=1, **pad)

        # Row 2 — Manual steering
        self._build_manual(row=2, **pad)

    # ---- Telemetry ----

    def _build_telemetry(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Telemetry", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_loc   = ttk.Label(f, text="Location : —")
        self.lbl_head  = ttk.Label(f, text="Heading  : —")
        self.lbl_nav   = ttk.Label(f, text="Target WP: —  |  Mode: —")
        self.lbl_stats = ttk.Label(f, text="Batt: —%  |  HDOP: —  |  Err: —")

        for lbl in (self.lbl_loc, self.lbl_head, self.lbl_nav, self.lbl_stats):
            lbl.pack(anchor="w", pady=1)

    # ---- Connection status ----

    def _build_status(self, row, col, **kw) -> None:
        f = tk.LabelFrame(self.root, text="Connection Status", padx=10, pady=8)
        f.grid(row=row, column=col, sticky="nsew", **kw)

        self.lbl_receiver = ttk.Label(f, text="Receiver : OFFLINE")
        self.lbl_lora     = ttk.Label(f, text="LoRa     : —")
        self.lbl_wifi     = ttk.Label(f, text="WiFi     : —")

        for lbl in (self.lbl_receiver, self.lbl_lora, self.lbl_wifi):
            lbl.pack(anchor="w", pady=1)

    # ---- Mode control ----

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

    # ---- Manual controls ----

    def _build_manual(self, row, **kw) -> None:
        f = tk.LabelFrame(
            self.root, text="Manual Controls  (Receiver ESP → WiFi UDP → Boat)",
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

        # Rudder  (boat expects −80…+80 as int8_t, 0 = straight)
        ttk.Label(f, text="Rudder    −80 →  +80").grid(row=1, column=0, sticky="e", padx=6)
        self.scale_rudder = ttk.Scale(f, from_=-80, to=80, orient="horizontal", length=230)
        self.scale_rudder.grid(row=1, column=1)
        self.scale_rudder.set(0)

        self.lbl_rudder_val = ttk.Label(f, text="  0", width=5)
        self.lbl_rudder_val.grid(row=1, column=2)

        ttk.Button(f, text="Reset Neutral", command=self._reset_controls).grid(
            row=0, column=3, rowspan=2, padx=14
        )

    # -----------------------------------------------------------------------
    # Periodic UI refresh  (runs in main thread via after())
    # -----------------------------------------------------------------------

    def _refresh(self) -> None:
        self._update_telemetry()
        self._update_status()
        self._update_mode_buttons()
        self._push_manual_values()

        self.root.after(100, self._refresh)

    def _update_telemetry(self) -> None:
        d = self.ctrl.boat_data
        self.lbl_loc.config(
            text=f"Location : {d['lat']:.6f},  {d['lon']:.6f}"
        )
        self.lbl_head.config(text=f"Heading  : {d['heading']:.1f}°")
        self.lbl_nav.config(
            text=f"Target WP: {d['target_idx']}   |   Mode: {MODE_NAMES.get(d['mode'], '?')}"
        )
        self.lbl_stats.config(
            text=f"Batt: {d['battery']}%   |   HDOP: {d['hdop']:.1f}   |   Err: 0x{d['error']:04X}"
        )

    def _update_status(self) -> None:
        rx = self.ctrl.receiver_connected
        now = time.time()

        # Receiver
        if rx:
            _indicator(
                self.lbl_receiver,
                f"Receiver : ONLINE  ({self.ctrl.serial_port})",
                _GREEN,
            )
        else:
            _indicator(self.lbl_receiver, "Receiver : OFFLINE", _RED)

        # LoRa  — 4 states:
        #   OFFLINE  = receiver itself not connected
        #   WAITING  = receiver up but never got a LoRa frame yet
        #   TIMEOUT  = had frames before but went silent
        #   ONLINE   = frame received within _LORA_STALE_S
        if not rx:
            _indicator(self.lbl_lora, "LoRa     : OFFLINE", _RED)
        elif self.ctrl.lora_online:
            age = now - self.ctrl.last_lora_time
            _indicator(self.lbl_lora, f"LoRa     : ONLINE  ({age:.1f}s ago)", _GREEN)
        elif self.ctrl.last_lora_time == 0.0:
            _indicator(self.lbl_lora, "LoRa     : WAITING …", _ORANGE)
        else:
            silent = now - self.ctrl.last_lora_time
            _indicator(self.lbl_lora, f"LoRa     : TIMEOUT  ({silent:.0f}s)", _RED)

        # WiFi — same 4 states
        if not rx:
            _indicator(self.lbl_wifi, "WiFi     : OFFLINE", _RED)
        elif self.ctrl.wifi_online:
            age = now - self.ctrl.last_wifi_time
            _indicator(self.lbl_wifi, f"WiFi     : ONLINE  ({age:.1f}s ago)", _GREEN)
        elif self.ctrl.last_wifi_time == 0.0:
            _indicator(self.lbl_wifi, "WiFi     : WAITING …", _ORANGE)
        else:
            silent = now - self.ctrl.last_wifi_time
            _indicator(self.lbl_wifi, f"WiFi     : TIMEOUT  ({silent:.0f}s)", _RED)

    def _update_mode_buttons(self) -> None:
        """Highlight the button whose mode matches the boat's telemetry-confirmed mode."""
        confirmed = self.ctrl.boat_data["mode"]
        for mode_id, btn in self._mode_btns.items():
            if mode_id == confirmed:
                btn.config(
                    bg=_BTN_ACTIVE_BG, fg=_BTN_ACTIVE_FG,
                    relief="sunken", bd=3,
                )
            else:
                btn.config(
                    bg=_BTN_NORMAL_BG, fg=_BTN_NORMAL_FG,
                    relief="raised", bd=2,
                )

    def _push_manual_values(self) -> None:
        """Write slider values to the controller and update value labels."""
        t = self.scale_throttle.get()
        r = self.scale_rudder.get()

        self.ctrl.manual_throttle = t
        self.ctrl.manual_rudder   = r

        self.lbl_throttle_val.config(text=f"{t:+.0f}")
        self.lbl_rudder_val.config(text=f"{r:+.0f}")

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _reset_controls(self) -> None:
        self.scale_throttle.set(0)
        self.scale_rudder.set(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"

    controller = BoatController(serial_port=port)
    root = tk.Tk()
    app  = BoatGUI(root, controller)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop()
