"""
boat_controller.py  –  Vene 2.0 Ground Station Controller Module
=================================================================
Portable, GUI-agnostic controller for the Vene 2.0 autonomous boat.
Handles serial connection, telemetry parsing, mode changes, and route
upload over LoRa via the receiver ESP32.

Typical usage
-------------
    from boat_controller import BoatController

    ctrl = BoatController(serial_port="COM4")
    ctrl.set_mode(1)                          # non-blocking
    ctrl.send_route([(lat1, lon1), ...])      # non-blocking
    print(ctrl.boat_data)
    ctrl.stop()

Callbacks
---------
    ctrl.on_telemetry   = lambda data: ...    # dict with latest boat_data
    ctrl.on_mode_change = lambda mode: ...    # int, fires on confirmed change
"""

import serial
import struct
import time
import threading
from typing import Callable, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Packet formats  (must match lora.h / wifiComm.h on the boat)
# ---------------------------------------------------------------------------
FAST_FORMAT          = "<BddfBB"
FAST_SIZE            = struct.calcsize(FAST_FORMAT)

SLOW_FORMAT          = "<BBBBI"
SLOW_SIZE            = struct.calcsize(SLOW_FORMAT)

ROUTE_FORMAT         = "<BBddBB"

ACK_FORMAT           = "<BBB"
ACK_SIZE             = struct.calcsize(ACK_FORMAT)

CONTROL_FORMAT       = "<BB"
MANUAL_FORMAT        = "<Bbb"          # ID(B), Throttle(b), Rudder(b)  –  all int8_t

WIFI_HEARTBEAT_FORMAT = "<BI"
WIFI_HEARTBEAT_SIZE  = struct.calcsize(WIFI_HEARTBEAT_FORMAT)

# Packet IDs
PKT_WP_DATA   = 0x01
PKT_TELE_FAST = 0x02
PKT_TELE_SLOW = 0x03
PKT_CONTROL   = 0x04
PKT_MANUAL    = 0x05
PKT_ACK       = 0x10   # same as PKT_DATA on the boat
PKT_WIFI_HB   = 0x06

# Human-readable mode names
MODE_NAMES: dict = {0: "STOP", 1: "MANUAL", 2: "AUTO", 3: "RTL"}

# Tuning constants
_MODE_TIMEOUT_S   = 1.2   # per-attempt LoRa round-trip budget (SF9 ≈ 370 ms min)
_MODE_RETRIES     = 5
_ROUTE_TIMEOUT_S  = 2.0
_ROUTE_RETRIES    = 5
_LORA_STALE_S     = 3.0   # seconds without a LoRa frame → TIMEOUT
_WIFI_STALE_S     = 3.0


class BoatController:
    """
    Thread-safe controller for the Vene 2.0 autonomous boat.

    All public methods are safe to call from any thread.
    """

    def __init__(self, serial_port: str = "COM4", baud_rate: int = 115200):
        self.serial_port = serial_port
        self.baud_rate   = baud_rate
        self.ser: Optional[serial.Serial] = None

        # ---- Live telemetry (read-only from caller) ----
        self.boat_data: dict = {
            "lat": 0.0, "lon": 0.0, "heading": 0.0,
            "mode": 0,  "target_idx": 0,
            "battery": 0, "hdop": 0.0, "signal": 0, "error": 0,
        }

        # ---- Manual control values (write from GUI) ----
        # Throttle: -100 … +100  (maps to ESC)
        # Rudder:     -80 …  +80  (turnRudder() on boat; 0 = straight)
        self.manual_throttle: float = 0
        self.manual_rudder:   float = 0

        # ---- Connection state ----
        self.receiver_connected: bool  = False
        self.last_lora_time: float     = 0.0   # epoch of last LoRa frame
        self.last_wifi_time: float     = 0.0   # epoch of last WiFi heartbeat

        # ---- Optional callbacks ----
        self.on_telemetry:   Optional[Callable[[dict], None]] = None
        self.on_mode_change: Optional[Callable[[int],  None]] = None

        # ---- Internal ----
        self._serial_lock  = threading.Lock()
        self._data_lock    = threading.Lock()
        self._ack_event    = threading.Event()   # waypoint ACK
        self._mode_event   = threading.Event()   # mode ACK
        self._ack_value    = -1
        self._mode_ack_val = -1
        self.running       = True

        # Start background threads
        for target in (
            self._connection_manager,
            self._serial_rx_thread,
            self._manual_tx_thread,
            self._heartbeat_tx_thread,
        ):
            threading.Thread(target=target, daemon=True).start()

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def stop(self) -> None:
        """Clean shutdown — call when the application exits."""
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    def set_mode(self, mode: int) -> None:
        """
        Request a mode change on the boat.  Non-blocking; runs in a daemon
        thread.  Mode is confirmed either by an explicit ACK over LoRa or by
        seeing the new mode arrive in telemetry.

        Modes:  0 = STOP  |  1 = MANUAL  |  2 = AUTO  |  3 = RTL
        """
        if not self.receiver_connected:
            print("[MODE] Cannot change mode: receiver offline")
            return
        threading.Thread(
            target=self._set_mode_task, args=(mode,), daemon=True
        ).start()

    def send_route(
        self,
        waypoints: List[Tuple[float, float]],
        route_id: int = 100,
    ) -> None:
        """
        Upload a list of (lat, lon) waypoints to the boat.  Non-blocking.
        Each waypoint is retried up to _ROUTE_RETRIES times on ACK timeout.
        """
        if not self.receiver_connected:
            print("[ROUTE] Cannot upload route: receiver offline")
            return
        threading.Thread(
            target=self._upload_route_task,
            args=(waypoints, route_id),
            daemon=True,
        ).start()

    def send_test_route(self) -> None:
        """Upload the built-in 5×10 lawnmower survey pattern near Helsinki."""
        wps = []
        start_lat, start_lon = 60.2055, 24.6559
        for row in range(5):
            lat = start_lat + row * 0.00015
            cols = range(10) if row % 2 == 0 else range(9, -1, -1)
            for col in cols:
                lon = start_lon + col * 0.00030
                wps.append((round(lat, 6), round(lon, 6)))
        self.send_route(wps)

    # ---- Computed connection properties ----

    @property
    def lora_online(self) -> bool:
        """True only if the receiver is up AND a LoRa frame arrived recently."""
        return (
            self.receiver_connected
            and self.last_lora_time > 0
            and (time.time() - self.last_lora_time) < _LORA_STALE_S
        )

    @property
    def wifi_online(self) -> bool:
        """True only if the receiver is up AND a WiFi heartbeat arrived recently."""
        return (
            self.receiver_connected
            and self.last_wifi_time > 0
            and (time.time() - self.last_wifi_time) < _WIFI_STALE_S
        )

    # -----------------------------------------------------------------------
    # Internal: Serial connection manager
    # -----------------------------------------------------------------------

    def _connection_manager(self) -> None:
        while self.running:
            if not self.receiver_connected:
                # Wipe link-layer timestamps so indicators go dark immediately
                self.last_lora_time = 0.0
                self.last_wifi_time = 0.0
                try:
                    if self.ser:
                        self.ser.close()
                    self.ser = serial.Serial(
                        self.serial_port, self.baud_rate, timeout=0.1
                    )
                    self.receiver_connected = True
                    print(f"[RX] Connected on {self.serial_port}")
                except Exception:
                    self.receiver_connected = False
            time.sleep(1)

    # -----------------------------------------------------------------------
    # Internal: Mode change task
    # -----------------------------------------------------------------------

    def _set_mode_task(self, mode: int) -> None:
        pkt = struct.pack(CONTROL_FORMAT, PKT_CONTROL, mode)

        for attempt in range(1, _MODE_RETRIES + 1):
            if not self.receiver_connected:
                print("[MODE] Aborted: receiver offline")
                return

            self._mode_ack_val = -1
            self._mode_event.clear()

            try:
                with self._serial_lock:
                    self.ser.write(pkt)
                print(f"[MODE] Requested mode={mode}  (attempt {attempt}/{_MODE_RETRIES})")
            except Exception as e:
                print(f"[MODE] TX error: {e}")
                self.receiver_connected = False
                return

            # Wait for explicit ACK or telemetry confirmation (whichever comes first)
            deadline = time.time() + _MODE_TIMEOUT_S
            while time.time() < deadline:
                # ACK arrived via _serial_rx_thread → _mode_event
                if self._mode_event.wait(timeout=0.02):
                    if self._mode_ack_val == mode:
                        print(f"[MODE] ACK confirmed mode={mode}")
                        return
                # Telemetry already reflects the new mode
                if self.boat_data["mode"] == mode:
                    print(f"[MODE] Telemetry confirmed mode={mode}")
                    return

            print(f"[MODE] Timeout on attempt {attempt}")

        print(f"[MODE] Failed after {_MODE_RETRIES} attempts")

    # -----------------------------------------------------------------------
    # Internal: Route upload task
    # -----------------------------------------------------------------------

    def _upload_route_task(
        self, waypoints: List[Tuple[float, float]], route_id: int
    ) -> None:
        total = len(waypoints)

        for order, (lat, lon) in enumerate(waypoints):
            pkt = struct.pack(ROUTE_FORMAT, PKT_WP_DATA, route_id, lat, lon, order, total)

            for attempt in range(1, _ROUTE_RETRIES + 1):
                if not self.receiver_connected:
                    print("[ROUTE] Aborted: receiver offline")
                    return

                self._ack_value = -1
                self._ack_event.clear()

                try:
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception as e:
                    print(f"[ROUTE] TX error: {e}")
                    self.receiver_connected = False
                    return

                if self._ack_event.wait(timeout=_ROUTE_TIMEOUT_S):
                    if self._ack_value == order:
                        print(f"[ROUTE] WP {order + 1}/{total} ACKed")
                        break
                else:
                    print(
                        f"[ROUTE] WP {order + 1}/{total} timeout "
                        f"(attempt {attempt}/{_ROUTE_RETRIES})"
                    )
                    if attempt == _ROUTE_RETRIES:
                        print("[ROUTE] Upload stopped: max retries reached")
                        return

        print("[ROUTE] Upload complete")

    # -----------------------------------------------------------------------
    # Internal: Serial RX
    # -----------------------------------------------------------------------

    def _serial_rx_thread(self) -> None:
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        byte = self.ser.read(1)

                        if byte == bytes([PKT_TELE_FAST]):
                            payload = byte + self.ser.read(FAST_SIZE - 1)
                            if len(payload) == FAST_SIZE:
                                self._handle_fast_tele(payload)

                        elif byte == bytes([PKT_TELE_SLOW]):
                            payload = byte + self.ser.read(SLOW_SIZE - 1)
                            if len(payload) == SLOW_SIZE:
                                self._handle_slow_tele(payload)

                        elif byte == bytes([PKT_ACK]):
                            payload = byte + self.ser.read(ACK_SIZE - 1)
                            if len(payload) == ACK_SIZE:
                                self._handle_ack(payload)

                        elif byte == bytes([PKT_WIFI_HB]):
                            payload = byte + self.ser.read(WIFI_HEARTBEAT_SIZE - 1)
                            if len(payload) == WIFI_HEARTBEAT_SIZE:
                                self.last_wifi_time = time.time()

                        # Unknown byte: discard and continue

                except Exception:
                    self.receiver_connected = False
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                    self.ser = None

            time.sleep(0.005)  # ~5 ms polling, tight enough for 115200 baud

    def _handle_fast_tele(self, payload: bytes) -> None:
        u = struct.unpack(FAST_FORMAT, payload)
        old_mode = self.boat_data["mode"]
        with self._data_lock:
            self.boat_data.update({
                "lat": u[1], "lon": u[2], "heading": u[3],
                "mode": u[4], "target_idx": u[5],
            })
        self.last_lora_time = time.time()

        if self.on_telemetry:
            self.on_telemetry(dict(self.boat_data))
        if u[4] != old_mode and self.on_mode_change:
            self.on_mode_change(u[4])

    def _handle_slow_tele(self, payload: bytes) -> None:
        u = struct.unpack(SLOW_FORMAT, payload)
        with self._data_lock:
            self.boat_data.update({
                "battery": u[1],
                "hdop":    u[2] / 10.0,
                "signal":  u[3],
                "error":   u[4],
            })
        self.last_lora_time = time.time()

    def _handle_ack(self, payload: bytes) -> None:
        _, ack_id, ack_val = struct.unpack(ACK_FORMAT, payload)
        self.last_lora_time = time.time()

        if ack_id == 255:          # mode ACK
            self._mode_ack_val = ack_val
            self._mode_event.set()
        else:                       # waypoint ACK
            self._ack_value = ack_val
            self._ack_event.set()

    # -----------------------------------------------------------------------
    # Internal: Periodic TX threads
    # -----------------------------------------------------------------------

    def _manual_tx_thread(self) -> None:
        """Send manual controls at 10 Hz via serial → receiver ESP → WiFi UDP."""
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    throttle = int(max(-100, min(100, self.manual_throttle)))
                    rudder   = int(max(-80,  min(80,  self.manual_rudder)))
                    pkt = struct.pack(MANUAL_FORMAT, PKT_MANUAL, throttle, rudder)
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
            time.sleep(0.1)

    def _heartbeat_tx_thread(self) -> None:
        """Send LoRa heartbeat every 2 s to prevent loraTimeout on the boat."""
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    pkt = struct.pack(ACK_FORMAT, PKT_ACK, 254, 0)
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
            time.sleep(2)
