import serial
import struct
import time
import threading
from typing import Callable, List, Optional, Tuple, Set

FAST_FORMAT = "<BddfBB"
FAST_SIZE = struct.calcsize(FAST_FORMAT)

SLOW_FORMAT = "<BBBBI"
SLOW_SIZE = struct.calcsize(SLOW_FORMAT)

ROUTE_FORMAT = "<BBddBB"

ACK_FORMAT = "<BBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)

CONTROL_FORMAT = "<BB"

MANUAL_FORMAT = "<Bbb"

RESET_ERRORS_FORMAT = "<B"

WIFI_HEARTBEAT_FORMAT = "<BI"
WIFI_HEARTBEAT_SIZE = struct.calcsize(WIFI_HEARTBEAT_FORMAT)

HOME_FORMAT = "<Bdd"
HOME_SIZE = struct.calcsize(HOME_FORMAT)  

TIME_DATA_FORMAT = "<BI"
TIME_DATA_SIZE = struct.calcsize(TIME_DATA_FORMAT)

PKT_WP_DATA        = 0x01
PKT_TELE_FAST      = 0x02
PKT_TELE_SLOW      = 0x03
PKT_CONTROL        = 0x04
PKT_MANUAL         = 0x05
PKT_WIFI_HB        = 0x06
PKT_RESET_ERRORS   = 0x07
PKT_HOME_SET       = 0x08 
PKT_HOME_DATA      = 0x09 
PKT_HOME_REQ       = 0x0A  
PKT_TIME_REQ       = 0x0B  
PKT_TIME_DATA      = 0x0C  
PKT_DATA           = 0x10   # generic ACK / heartbeat


MODE_NAMES: dict = {0: "STOP", 1: "MANUAL", 2: "AUTO", 3: "RTL"}

_MODE_TIMEOUT_S  = 1.2
_MODE_RETRIES    = 5
_ROUTE_TIMEOUT_S = 2.0
_ROUTE_RETRIES   = 5
_HOME_TIMEOUT_S  = 2.5    # per-attempt round-trip for home set / request
_HOME_RETRIES    = 4
_LORA_STALE_S    = 5.0
_WIFI_STALE_S    = 3.0


class Controller:
    def __init__(self, serial_port: str = "COM4", baud_rate: int = 115200):
        self.serial_port = serial_port
        self.baud_rate   = baud_rate
        self.ser: Optional[serial.Serial] = None

        self.boat_data: dict = {
            "lat": 0.0, "lon": 0.0, "heading": 0.0,
            "mode": 0,  "target_idx": 0,
            "battery": 0, "hdop": 0.0, "signal": 0, "error": 0,
        }

        # Home waypoint (0, 0 = not set)
        self.home_lat: float = 0.0
        self.home_lon: float = 0.0
        self.home_set: bool  = False

        # Throttle / rudder
        self.manual_throttle: float = 0
        self.manual_rudder:   float = 0

        # Connection state
        self.receiver_connected: bool  = False
        self.last_lora_time:     float = 0.0
        self.last_wifi_time:     float = 0.0

        # Upload state
        self.upload_status:  str = "idle"
        self.upload_current: int = 0
        self.upload_total:   int = 0

        # ── Callbacks ─────────────────────────────────────────────────────────
        self.on_telemetry:      Optional[Callable[[dict], None]]             = None
        self.on_mode_change:    Optional[Callable[[int], None]]              = None
        self.on_error_change:   Optional[Callable[[Set[int], Set[int]], None]] = None
        self.on_home_received:  Optional[Callable[[float, float], None]]     = None

        # ── Internal sync objects ──────────────────────────────────────────────
        self._serial_lock  = threading.Lock()
        self._data_lock    = threading.Lock()

        self._ack_event    = threading.Event()   # waypoint ACK
        self._ack_value    = -1

        self._mode_event   = threading.Event()   # mode ACK
        self._mode_ack_val = -1

        self._home_set_event  = threading.Event()   # home ACK
        self._home_data_event = threading.Event()  

        # LoRa online edge-detection for auto home-request
        self._lora_was_online: bool = False

        self.running = True

        for target in (
            self._connection_manager,
            self._serial_rx_thread,
            self._manual_tx_thread,
            self._heartbeat_tx_thread,
            self._lora_monitor_thread,
        ):
            threading.Thread(target=target, daemon=True).start()

    def stop(self) -> None:
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    # ── Public API ────────────────────────────────────────────────────────────

    def set_mode(self, mode: int) -> None:
        """Request a mode change (non-blocking)."""
        if not self.receiver_connected:
            print("[MODE] Cannot change mode: receiver offline")
            return
        threading.Thread(target=self._set_mode_task, args=(mode,), daemon=True).start()

    def send_route(self, waypoints: List[Tuple[float, float]], route_id: int = 100) -> None:
        if not self.receiver_connected:
            print("[ROUTE] Cannot upload route: receiver offline")
            return
        threading.Thread(
            target=self._upload_route_task, args=(waypoints, route_id), daemon=True
        ).start()

    def send_test_route(self) -> None:
        wps = []
        start_lat, start_lon = 60.2055, 24.6559
        for row in range(5):
            lat  = start_lat + row * 0.00015
            cols = range(10) if row % 2 == 0 else range(9, -1, -1)
            for col in cols:
                lon = start_lon + col * 0.00030
                wps.append((round(lat, 6), round(lon, 6)))
        self.send_route(wps)

    def reset_errors(self) -> None:
        if not self.receiver_connected:
            print("[ERR_RST] Cannot reset errors, receiver not connected")
            return
        threading.Thread(target=self._reset_errors_task, daemon=True).start()

    def set_home(self, lat: float, lon: float) -> None:
        if not self.lora_online:
            print("[HOME] Cannot set home: LORA offline")
            return
        threading.Thread(target=self._set_home_task, args=(lat, lon), daemon=True).start()

    def request_home(self) -> None:
        if not self.lora_online:
            print("[HOME] Cannot request home: LORA offline")
            return
        threading.Thread(target=self._request_home_task, daemon=True).start()

    # ── Computed connection properties ────────────────────────────────────────

    @property
    def lora_online(self) -> bool:
        return (
            self.receiver_connected
            and self.last_lora_time > 0
            and (time.time() - self.last_lora_time) < _LORA_STALE_S
        )

    @property
    def wifi_online(self) -> bool:
        return (
            self.receiver_connected
            and self.last_wifi_time > 0
            and (time.time() - self.last_wifi_time) < _WIFI_STALE_S
        )

    # ── Internal: connection manager ──────────────────────────────────────────

    def _connection_manager(self) -> None:
        while self.running:
            if not self.receiver_connected:
                self.last_lora_time = 0.0
                self.last_wifi_time = 0.0
                try:
                    if self.ser:
                        self.ser.close()
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    self.receiver_connected = True
                    print(f"[RX] Connected on {self.serial_port}")
                except Exception:
                    self.receiver_connected = False
            time.sleep(1)

    # ── Internal: LoRa-online edge detector ───────────────────────────────────

    def _lora_monitor_thread(self) -> None:
        while self.running:
            now = self.lora_online
            if not self._lora_was_online and now and not self.home_set:
                print("[HOME] Auto-requesting home")
                threading.Thread(target=self._request_home_task, daemon=True).start()
            self._lora_was_online = now
            time.sleep(0.5)

    # ── Internal: mode change task ────────────────────────────────────────────

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

            deadline = time.time() + _MODE_TIMEOUT_S
            while time.time() < deadline:
                if self._mode_event.wait(timeout=0.02):
                    if self._mode_ack_val == mode:
                        print(f"[MODE] ACK confirmed mode={mode}")
                        return
                if self.boat_data["mode"] == mode:
                    print(f"[MODE] Telemetry confirmed mode={mode}")
                    return

            print(f"[MODE] Timeout on attempt {attempt}")

        print(f"[MODE] Failed after {_MODE_RETRIES} attempts")

    # ── Internal: home set task ───────────────────────────────────────────────

    def _set_home_task(self, lat: float, lon: float) -> None:
        pkt = struct.pack(HOME_FORMAT, PKT_HOME_SET, lat, lon)

        for attempt in range(1, _HOME_RETRIES + 1):
            if not self.lora_online:
                print("[HOME] Aborted set: LoRa offline")
                return

            self._home_set_event.clear()

            try:
                with self._serial_lock:
                    self.ser.write(pkt)
                print(f"[HOME] Set {lat:.6f},{lon:.6f}  (attempt {attempt}/{_HOME_RETRIES})")
            except Exception as e:
                print(f"[HOME] TX error: {e}")
                self.receiver_connected = False
                return

            if self._home_set_event.wait(timeout=_HOME_TIMEOUT_S):
                self.home_lat = lat
                self.home_lon = lon
                self.home_set = True
                print(f"[HOME] Home ack: {lat:.6f}, {lon:.6f}")
                if self.on_home_received:
                    self.on_home_received(lat, lon)
                return

            print(f"[HOME] Set timeout on attempt {attempt}")

        print(f"[HOME] Failed to set home after {_HOME_RETRIES} attempts")

    # ── Internal: home request task ───────────────────────────────────────────

    def _request_home_task(self) -> None:
        pkt = struct.pack("<B", PKT_HOME_REQ)

        for attempt in range(1, _HOME_RETRIES + 1):
            if not self.lora_online:
                print("[HOME] Aborted request: LoRa offline")
                return

            self._home_data_event.clear()

            try:
                with self._serial_lock:
                    self.ser.write(pkt)
                print(f"[HOME] Requesting home  (attempt {attempt}/{_HOME_RETRIES})")
            except Exception as e:
                print(f"[HOME] TX error: {e}")
                self.receiver_connected = False
                return

            if self._home_data_event.wait(timeout=_HOME_TIMEOUT_S):
                if self.home_set:
                    print(f"[HOME] Received: {self.home_lat:.6f}, {self.home_lon:.6f}")
                else:
                    print("[HOME] Received: boat has no home set (0, 0)")
                return

            print(f"[HOME] Request timeout on attempt {attempt}")

        print(f"[HOME] Home request failed after {_HOME_RETRIES} attempts")

    # ── Internal: route upload task ───────────────────────────────────────────

    def _upload_route_task(self, waypoints: List[Tuple[float, float]], route_id: int) -> None:
        total = len(waypoints)
        self.upload_status  = "uploading"
        self.upload_current = 0
        self.upload_total   = total

        for order, (lat, lon) in enumerate(waypoints):
            pkt = struct.pack(ROUTE_FORMAT, PKT_WP_DATA, route_id, lat, lon, order, total)

            for attempt in range(1, _ROUTE_RETRIES + 1):
                if not self.receiver_connected:
                    print("[ROUTE] Aborted: receiver offline")
                    self.upload_status = "failed"
                    return

                self._ack_value = -1
                self._ack_event.clear()

                try:
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception as e:
                    print(f"[ROUTE] TX error: {e}")
                    self.receiver_connected = False
                    self.upload_status = "failed"
                    return

                if self._ack_event.wait(timeout=_ROUTE_TIMEOUT_S):
                    if self._ack_value == order:
                        self.upload_current = order + 1
                        print(f"[ROUTE] WP {order + 1}/{total} ACKed")
                        break
                else:
                    print(
                        f"[ROUTE] WP {order + 1}/{total} timeout "
                        f"(attempt {attempt}/{_ROUTE_RETRIES})"
                    )
                    if attempt == _ROUTE_RETRIES:
                        print("[ROUTE] Upload stopped: max retries reached")
                        self.upload_status = "failed"
                        return

        self.upload_current = total
        self.upload_status  = "done"
        print("[ROUTE] Upload complete")

    # ── Internal: reset errors task ───────────────────────────────────────────

    def _reset_errors_task(self) -> None:
        pkt = struct.pack(RESET_ERRORS_FORMAT, PKT_RESET_ERRORS)
        try:
            with self._serial_lock:
                self.ser.write(pkt)
            print("[ERR_RST] Reset packet sent")
        except Exception as e:
            print(f"[ERR_RST] TX error: {e}")
            self.receiver_connected = False

    # ── Internal: serial RX thread ────────────────────────────────────────────

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

                        elif byte == bytes([PKT_DATA]):
                            payload = byte + self.ser.read(ACK_SIZE - 1)
                            if len(payload) == ACK_SIZE:
                                self._handle_ack(payload)

                        elif byte == bytes([PKT_WIFI_HB]):
                            payload = byte + self.ser.read(WIFI_HEARTBEAT_SIZE - 1)
                            if len(payload) == WIFI_HEARTBEAT_SIZE:
                                self.last_wifi_time = time.time()

                        elif byte == bytes([PKT_HOME_DATA]):
                            # Boat responding to our PKT_HOME_REQ
                            payload = byte + self.ser.read(HOME_SIZE - 1)
                            if len(payload) == HOME_SIZE:
                                self._handle_home_data(payload)

                        elif byte == bytes([PKT_TIME_REQ]):
                            # Boat is asking for current unix time (single-byte packet)
                            # Respond in a background thread so we don't block RX
                            threading.Thread(
                                target=self._send_time_data, daemon=True
                            ).start()

                        # Unknown byte: discard and continue

                except Exception:
                    self.receiver_connected = False
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                    self.ser = None

            time.sleep(0.005)

    # ── Internal: packet handlers ─────────────────────────────────────────────

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
        new_error = u[4]
        old_error = self.boat_data["error"]

        with self._data_lock:
            self.boat_data.update({
                "battery": u[1],
                "hdop":    u[2] / 10.0,
                "signal":  u[3] - 128,
                "error":   new_error,
            })
        self.last_lora_time = time.time()

        if new_error != old_error and self.on_error_change:
            new_bits     = {b for b in range(32) if (new_error >> b & 1) and not (old_error >> b & 1)}
            cleared_bits = {b for b in range(32) if (old_error >> b & 1) and not (new_error >> b & 1)}
            self.on_error_change(new_bits, cleared_bits)

    def _handle_ack(self, payload: bytes) -> None:
        _, ack_id, ack_val = struct.unpack(ACK_FORMAT, payload)

        # Heartbeat echo from receiver — ignore
        if ack_id == 254:
            return

        self.last_lora_time = time.time()

        if ack_id == 255:
            # Mode ACK (id=255, val=new_mode)
            self._mode_ack_val = ack_val
            self._mode_event.set()

        elif ack_id == PKT_HOME_SET:
            # Home-set ACK (id=0x08, val=0x01)
            self._home_set_event.set()

        else:
            # Waypoint ACK (id=route_id, val=order)
            self._ack_value = ack_val
            self._ack_event.set()

    def _handle_home_data(self, payload: bytes) -> None:
        _, lat, lon = struct.unpack(HOME_FORMAT, payload)
        self.last_lora_time = time.time()

        if lat != 0.0 or lon != 0.0:
            self.home_lat = lat
            self.home_lon = lon
            self.home_set = True
        else:
            self.home_set = False
            self.home_lat = 0.0
            self.home_lon = 0.0

        self._home_data_event.set()

        if self.on_home_received:
            self.on_home_received(self.home_lat, self.home_lon)

    def _send_time_data(self) -> None:
        if not self.receiver_connected or not self.ser:
            return
        unix_now = int(time.time())
        pkt = struct.pack(TIME_DATA_FORMAT, PKT_TIME_DATA, unix_now)
        try:
            with self._serial_lock:
                self.ser.write(pkt)
            print(f"[TIME] Sent unix time {unix_now} to boat")
        except Exception as e:
            print(f"[TIME] TX error: {e}")
            self.receiver_connected = False

    # ── Internal: periodic TX threads ────────────────────────────────────────

    def _manual_tx_thread(self) -> None:
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
        while self.running:
            if self.receiver_connected and self.ser and self.upload_status != "uploading":
                try:
                    pkt = struct.pack(ACK_FORMAT, PKT_DATA, 254, 0)
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
            time.sleep(2)