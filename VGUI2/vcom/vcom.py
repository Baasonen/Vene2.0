import struct
import threading
import time

from typing import Callable, Dict, List, Optional, Set, Tuple, Any
from VCOM.comms import SerialTransport
from VCOM.protocol import (
    ACK_FORMAT, CONTROL_FORMAT, FAST_FORMAT, HOME_FORMAT,
    RESET_ERRORS_FORMAT, ROUTE_FORMAT, SLOW_FORMAT, TIME_DATA_FORMAT,
    COURSE_SET_FORMAT, COURSE_DATA_FORMAT, THR_SET_FORMAT, THR_DATA_FORMAT,
    BE_TELE_FORMAT,

    PKT_CONTROL, PKT_DATA, PKT_HOME_DATA, PKT_HOME_REQ, PKT_HOME_SET,
    PKT_RESET_ERRORS, PKT_TELE_FAST, PKT_TELE_SLOW, PKT_TIME_DATA, PKT_TIME_REQ,
    PKT_WP_DATA, PKT_COURSE_SET, PKT_COURSE_DATA, PKT_THR_SET, PKT_THR_DATA,
    PKT_BE_TELE,

    HOME_RETRIES, HOME_TIMEOUT_S,
    COURSE_RETRIES, COURSE_TIMEOUT_S, COURSE_SCALE, COURSE_REQ,
    THR_RETRIES, THR_TIMEOUT_S, THR_REQ,
    MODE_NAMES, MODE_RETRIES, MODE_TIMEOUT_S,
    ROUTE_RETRIES, ROUTE_TIMEOUT_S,

    uploadStatus, load_error_defs,
)

_NO_PENDING = object()

class Controller:

    def __init__(self, port: str = "COM4", baud: int = 115200):
        self.error_defs: Dict[int, Tuple[str, str]] = load_error_defs()

        self._telemetry_data: dict = {
            "lat": 0.0,
            "lon": 0.0, 
            "heading": 0.0,
            "mode": 0,
            "target_idx": 0,
            "battery": 0.0,
            "hAcc": 0.0,
            "signal": 0,
            "error": 0,
        }
        self._data_lock = threading.Lock()

        self._home_lat: float = 0.0
        self._home_lon: float = 0.0
        self._home_set: bool = False

        self._target_course: float = 0.0
        self._course_valid: bool = False

        self._target_throttle: float = 0.0
        self._throttle_valid: bool = False

        self._upload_status: uploadStatus = uploadStatus.IDLE
        self._upload_current: int = 0 # Number of WP confirmed
        self._upload_total: int = 0
        self._route_id_counter: int = 0

        # Callbacks
        self.on_telemetry: Optional[Callable[[dict], None]] = None
        self.on_mode_change: Optional[Callable[[int], None]] = None
        self.on_error_change: Optional[Callable[[Set[int], Set[int]], None]] = None
        self.on_home_received: Optional[Callable[[float, float], None]] = None

        # ACK sync
        self._ack_event = threading.Event()
        self._ack_val = -1

        self._mode_event = threading.Event()
        self._mode_ack_val = -1

        self._home_set_event = threading.Event()
        self._home_data_event = threading.Event()

        self._course_data_event = threading.Event()
        self._throttle_data_event = threading.Event()

        self._course_lock = threading.Lock()
        self._course_req_event = threading.Event()
        self._course_pending: Any = _NO_PENDING

        self._throttle_lock = threading.Lock()
        self._throttle_req_event = threading.Event()
        self._throttle_pending: Any = _NO_PENDING
        
        self._lora_was_online: bool = False

        self._t = SerialTransport(port, baud)

        self._t.register_handler(PKT_TELE_FAST, self._handle_fast_tele)
        self._t.register_handler(PKT_TELE_SLOW, self._handle_slow_tele)
        self._t.register_handler(PKT_BE_TELE, self._handle_be_tele)
        self._t.register_handler(PKT_DATA, self._handle_ack)
        self._t.register_handler(PKT_HOME_DATA, self._handle_home_data)
        self._t.register_handler(PKT_TIME_REQ, self._handle_time_req)
        self._t.register_handler(PKT_COURSE_DATA, self._handle_course_data)
        self._t.register_handler(PKT_THR_DATA, self._handle_throttle_data)

        self._t.start()
        threading.Thread(target = self._lora_monitor, daemon = True).start()
        threading.Thread(target = self._course_tx_worker, daemon = True).start()
        threading.Thread(target = self._throttle_tx_worker, daemon = True).start()

    def stop(self) -> None:
        self._t.stop()

    def get_telemetry_data(self) -> dict:
        with self._data_lock:
            return dict(self._telemetry_data)
        
    def get_connection_status(self) -> dict:
        with self._data_lock:
            return {
                "receiver_connected": self._t.connected,
                "lora_online": self._t.lora_online,
                "wifi_online": self._t.wifi_online,
                "home_set": self._home_set,
                "home_lat": self._home_lat,
                "home_lon": self._home_lon,
                "serial_port": self._t.port,
            }
    
    def get_upload_status(self) -> Tuple[uploadStatus, int, int]:
        with self._data_lock:
            return self._upload_status, self._upload_current, self._upload_total
        
    def get_course_status(self) -> Tuple[float, bool]:
        with self._data_lock:
            return self._target_course, self._course_valid

    def get_throttle_status(self) -> Tuple[float, bool]:
        with self._data_lock:
            return self._target_throttle, self._throttle_valid
        
    # Manual control
    @property
    def manual_throttle(self) -> None:
        return self._t.manual_throttle
    
    @manual_throttle.setter
    def manual_throttle(self, value: float) -> None:
        self._t.manual_throttle = value

    @property
    def manual_rudder(self) -> None:
        return self._t.manual_rudder
    
    @manual_rudder.setter
    def manual_rudder(self, value: float) -> None:
        self._t.manual_rudder = value

    # Non-blocking commands
    def set_mode(self, mode: int) -> None:
        if not self._t.connected:
            print("[MODE] Receiver offline")
            return
        
        threading.Thread(target = self._set_mode_task, args = (mode,), daemon = True).start()

    def send_route(self, waypoints: List[Tuple[float, float]], route_id: Optional[int] = None) -> None:
        if not self._t.connected:
            print("[ROUTE] Receiver offline")
            return

        if route_id is None:
            route_id = (self._route_id_counter % 100) + 1
            self._route_id_counter = route_id
        
        threading.Thread(target = self._upload_route_task, args = (waypoints, route_id), daemon = True).start()

    def send_test_route(self) -> None:
        wps: List[Tuple[float, float]] = []
        start_lat, start_lon = 60.2055, 24.6559
        for row in range(5):
            lat  = start_lat + row * 0.00015
            cols = range(10) if row % 2 == 0 else range(9, -1, -1)
            for col in cols:
                lon = start_lon + col * 0.00030
                wps.append((round(lat, 6), round(lon, 6)))
        self.send_route(wps)

    def reset_errors(self) -> None:
        if not self._t.connected:
            print("[ERR_RST] Receiver offline")
            return
        
        threading.Thread(target = self._reset_errors_task, daemon = True).start()

    def set_home(self, lat: float, lon: float) -> None:
        if not self._t.lora_online:
            print("[HOME] LoRa offline")
            return
        
        threading.Thread(target = self._set_home_task, args = (lat, lon), daemon = True).start()
 
    def request_home(self) -> None:
        if not self._t.lora_online:
            print("[HOME] LoRa offline")
            return
        
        threading.Thread(target = self._request_home_task, daemon = True).start()

    def set_course(self, heading_deg: float) -> None:
        if not self._t.lora_online:
            print("[CTRL] LoRa offline")
            return 
        
        self._queue_course(heading_deg)

    def request_course(self) -> None:
        if not self._t.lora_online:
            print("[CTRL] LoRa offline")
            return
        
        self._queue_course(None)

    def set_throttle(self, throttle: float) -> None:
        if not self._t.lora_online:
            print("[CTRL] LoRa offline")
            return

        self._queue_throttle(throttle)

    def request_throttle(self) -> None:
        if not self._t.lora_online:
            print("[CTRL] LoRa offline")
            return

        self._queue_throttle(None)

    # Worker tasks
    def _set_mode_task(self, mode: int) -> None:
        pkt = struct.pack(CONTROL_FORMAT, PKT_CONTROL, mode)

        for attempt in range(1, MODE_RETRIES + 1):
            if not self._t.connected:
                return
            
            self._mode_ack_val = -1
            self._mode_event.clear()

            if not self._t.write(pkt):
                return
            
            deadline = time.time() + MODE_TIMEOUT_S
            while time.time() < deadline:
                if self._mode_event.wait(timeout = 0.02) and self._mode_ack_val == mode:
                    print(f"[MODE] Confirmed: {MODE_NAMES.get(mode, mode)}")
                    return
                
                with self._data_lock:
                    if self._telemetry_data["mode"] == mode:
                        return
            
            print(f"[MODE] Timeout attempt {attempt} / {MODE_RETRIES}")

    def _set_home_task(self, lat: float, lon: float) -> None:
        pkt = struct.pack(HOME_FORMAT, PKT_HOME_SET, lat, lon)

        for attempt in range(1, HOME_RETRIES + 1):
            if not self._t.lora_online:
                return
            
            self._home_set_event.clear()

            if not self._t.write(pkt):
                return
            
            if self._home_set_event.wait(timeout = HOME_TIMEOUT_S):
                with self._data_lock:
                    self._home_lat = lat
                    self._home_lon = lon
                    self._home_set = True
                if self.on_home_received:
                    self.on_home_received(lat, lon)

                return
            
            print(f"[HOME] Set timput attempt {attempt} / {HOME_RETRIES}")

    def _request_home_task(self) -> None:
        pkt = struct.pack("<B", PKT_HOME_REQ)
        
        for attempt in range(1, HOME_RETRIES + 1):
            if not self._t.lora_online:
                return
            self._home_data_event.clear()
 
            if not self._t.write(pkt):
                return
 
            if self._home_data_event.wait(timeout=HOME_TIMEOUT_S):
                return  
            print(f"[HOME] Request timeout attempt {attempt} / {HOME_RETRIES}")

    def _upload_route_task(self, waypoints: List[Tuple[float, float]], route_id: int) -> None:
        total = len(waypoints)

        with self._data_lock:
            self._upload_status  = uploadStatus.UPLOADING
            self._upload_current = 0
            self._upload_total   = total

        self._t.set_upload_status(uploadStatus.UPLOADING)
 
        for order, (lat, lon) in enumerate(waypoints):
            pkt = struct.pack(ROUTE_FORMAT, PKT_WP_DATA, route_id, lat, lon, order, total)
            acked = False
 
            for attempt in range(1, ROUTE_RETRIES + 1):
                if not self._t.connected:
                    self._finish_upload(uploadStatus.FAILED)
                    return
 
                self._ack_val = -1
                self._ack_event.clear()
 
                if not self._t.write(pkt):
                    self._finish_upload(uploadStatus.FAILED)
                    return
 
                if self._ack_event.wait(timeout=ROUTE_TIMEOUT_S) and self._ack_val == order:
                    with self._data_lock:
                        self._upload_current = order + 1
                    acked = True
                    break

                print(f"[ROUTE] WP {order} timeout attempt {attempt} / {ROUTE_RETRIES}")
 
            if not acked:
                self._finish_upload(uploadStatus.FAILED)
                return
 
        self._finish_upload(uploadStatus.DONE, final_count=total)
 
    def _finish_upload(self, status: uploadStatus, final_count: int = 0) -> None:
        with self._data_lock:
            self._upload_status  = status

            if final_count:
                self._upload_current = final_count

        self._t.set_upload_status(status)
 
    def _reset_errors_task(self) -> None:
        pkt = struct.pack(RESET_ERRORS_FORMAT, PKT_RESET_ERRORS)
        self._t.write(pkt)

    def _queue_course(self, heading_deg: Optional[float]) -> None:
        with self._course_lock:
            self._course_pending = heading_deg

        self._course_req_event.set()

    def _course_tx_worker(self) -> None:
        while self._t.running:
            if not self._course_req_event.wait(timeout = 5.0):
                continue

            with self._course_lock:
                heading_deg = self._course_pending
                self._course_pending = _NO_PENDING
                self._course_req_event.clear()

            if heading_deg is _NO_PENDING:
                continue

            self._send_course(heading_deg)

    def _send_course(self, heading_deg: Optional[float]) -> None:
        is_req = heading_deg is None
        raw = COURSE_REQ if is_req else int(round(heading_deg * COURSE_SCALE)) % (360 * COURSE_SCALE)
        pkt = struct.pack(COURSE_SET_FORMAT, PKT_COURSE_SET, raw)

        for attempt in range(1, COURSE_RETRIES + 1):
            if not self._t.lora_online:
                return
            self._course_data_event.clear()

            if not self._t.write(pkt):
                return
            
            if self._course_data_event.wait(timeout = COURSE_TIMEOUT_S):
                return
            
            label = "Request" if is_req else "Set"
            print(f"[CRS] {label} timeout attempt {attempt} / {COURSE_RETRIES}")

    def _queue_throttle(self, throttle: Optional[float]) -> None:
        with self._throttle_lock:
            self._throttle_pending = throttle
        self._throttle_req_event.set()

    def _throttle_tx_worker(self) -> None:
        while self._t.running:
            if not self._throttle_req_event.wait(timeout = 5.0):
                continue

            with self._throttle_lock:
                throttle = self._throttle_pending
                self._throttle_pending = _NO_PENDING
                self._throttle_req_event.clear()

            if throttle is _NO_PENDING:
                continue

            self._send_throttle(throttle)

    def _send_throttle(self, throttle: Optional[float]) -> None:
        is_req = throttle is None
        raw = THR_REQ if is_req else int (round(max(-100, min(100, throttle))))
        pkt = struct.pack(THR_SET_FORMAT, PKT_THR_SET, raw)

        for attempt in range(1, THR_RETRIES + 1):
            if not self._t.lora_online:
                return
            self._throttle_data_event.clear()

            if not self._t.write(pkt):
                return

            if self._throttle_data_event.wait(timeout = THR_TIMEOUT_S):
                return

            label = "Request" if is_req else "Set"
            print(f"[CTRL] {label} timeout attempt {attempt} / {THR_RETRIES}")

    # Pakcet handlers
    def _handle_fast_tele(self, payload: bytes) -> None:
        _, heading_raw, mode, target_idx = struct.unpack(FAST_FORMAT, payload)

        self._t.update_lora_t()

        heading = heading_raw / 100.0

        with self._data_lock:
            old_mode = self._telemetry_data["mode"]

            self._telemetry_data.update({
                "heading": heading,
                "mode": mode,
                "target_idx": target_idx,
            })
            snapshot = dict(self._telemetry_data)

        if self.on_telemetry:
            self.on_telemetry(snapshot)
        
        if mode != old_mode and self.on_mode_change:
            self.on_mode_change(mode)

    def _handle_slow_tele(self, payload: bytes) -> None:
        _, battery, hAcc_raw, signal_raw, lat_raw, lon_raw = struct.unpack(SLOW_FORMAT, payload)

        self._t.update_lora_t()

        lat = lat_raw / 1e7
        lon = lon_raw / 1e7

        with self._data_lock:
            self._telemetry_data.update({
                "battery": battery,
                "hAcc": (hAcc_raw / 10.0) if hAcc_raw != 255 else 0.0,
                "signal": signal_raw - 128,
                "lat" : lat,
                "lon" : lon,
            })

    def _handle_be_tele(self, payload: bytes) -> None:
        _, c1, c2, c3, new_error = struct.unpack(BE_TELE_FORMAT, payload)

        self._t.update_lora_t()

        with self._data_lock:
            old_error = self._telemetry_data["error"]
            self._telemetry_data["error"] = new_error

        if new_error != old_error and self.on_error_change:
            new_bits = {b for b in range(32) if (new_error >> b & 1) and not (old_error >> b & 1)}
            cleared_bits = {b for b in range(32) if not (new_error >> b & 1) and (old_error >> b & 1)}

            self.on_error_change(new_bits, cleared_bits)
    
    def _handle_ack(self, payload: bytes) -> None:
        _, ack_id, ack_val = struct.unpack(ACK_FORMAT, payload)

        if ack_id == 254:
            return # HB echo
        
        self._t.update_lora_t()

        if ack_id == 255:
            self._mode_ack_val = ack_val
            self._mode_event.set()

        elif ack_id == PKT_HOME_SET:
            self._home_set_event.set()

        else:
            self._ack_val = ack_val
            self._ack_event.set()

    def _handle_home_data(self, payload: bytes) -> None:
        _, lat, lon = struct.unpack(HOME_FORMAT, payload)

        self._t.update_lora_t()

        with self._data_lock:
            if lat != 0.0 or lon != 0.0:
                self._home_lat, self._home_lon, self._home_set = lat, lon, True

            else:
                self._home_lat, self._home_lon, self._home_set = 0.0, 0.0, False

            h_lat, h_lon = self._home_lat, self._home_lon

        self._home_data_event.set()

        if self.on_home_received:
            self.on_home_received(h_lat, h_lon)

    def _handle_time_req(self, _payload: bytes) -> None:
        threading.Thread(target = self._send_time_data, daemon = True).start()

    def _send_time_data(self) -> None:
        pkt = struct.pack(TIME_DATA_FORMAT, PKT_TIME_DATA, int(time.time()))
        self._t.write(pkt)
        print("[TIME] Sent current time")

    def _handle_course_data(self, payload: bytes) -> None:
        _, raw_course = struct.unpack(COURSE_DATA_FORMAT, payload)

        self._t.update_lora_t()

        with self._data_lock:
            self._target_course = raw_course / COURSE_SCALE
            self._course_valid = True

        self._course_data_event.set()

    def _handle_throttle_data(self, payload: bytes) -> None:
        _, raw_throttle = struct.unpack(THR_DATA_FORMAT, payload)

        self._t.update_lora_t()

        with self._data_lock:
            self._target_throttle = raw_throttle
            self._throttle_valid = True

        self._throttle_data_event.set()

    # LoRa monito
    def _lora_monitor(self) -> None:
        while self._t.running:
            now_online = self._t.lora_online

            with self._data_lock:
                h_set = self._home_set

            if not self._lora_was_online and now_online and not h_set:
                print(f"[HOME] LoRa onlime, requesting home wp")
                threading.Thread(target = self._request_home_task, daemon = True).start()

            if not self._lora_was_online and now_online:
                self._queue_course(None)
                self._send_throttle(None)

            self._lora_was_online = now_online
            time.sleep(0.5)
