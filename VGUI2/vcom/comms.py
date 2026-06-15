"""
Serial layer for vcom

-Auto reconnect for USB/Lora receiver
-RX pareser
-Manual control thread (10 Hz)
-TX hearbeat thread (0.5 Hz)
-Track LoRa and WiFi staleness

"""

import serial
import struct
import threading
import time

from typing import Callable, Dict, Optional

from vcom.protocol import (
    ACK_FORMAT, ACK_SIZE,

    FAST_SIZE, HOME_SIZE,
    SLOW_SIZE, WIFI_HEARTBEAT_SIZE,

    MANUAL_FORMAT,

    PKT_DATA, PKT_HOME_DATA, PKT_MANUAL,
    PKT_TELE_FAST, PKT_TELE_SLOW,
    PKT_TIME_REQ, PKT_WIFI_HB,

    LORA_STALE_S, WIFI_STALE_S,

    uploadStatus,
)

PacketHandler = Callable[[bytes], None]

class SerialTransport:

    def __init__(self, port: str = "COM4", baud: int = 115200):
        self.port = port
        self.baud = baud

        self.ser: Optional[serial.Serial] = None

        self._serial_lock = threading.Lock()
        self._state_lock = threading.Lock()

        self._connected = False
        self._last_lora_t = 0.0
        self._last_wifi_t = 0.0

        self.manual_throttle: float = 0.0
        self.manual_rudder: float = 0.0

        self._upload_status = uploadStatus.IDLE
        self._upload_lock = threading.Lock()

        self._handlers: Dict[int, PacketHandler] = {}

        self._pkt_sizes: Dict[int, int] = {
            PKT_TELE_FAST: FAST_SIZE,
            PKT_TELE_SLOW: SLOW_SIZE,
            PKT_DATA: ACK_SIZE,
            PKT_WIFI_HB: WIFI_HEARTBEAT_SIZE,
            PKT_HOME_DATA: HOME_SIZE,
            PKT_TIME_REQ: 1,
        }

        self.running = False

    def register_handler(self, pkt_id: int, fn: PacketHandler) -> None:
        self._handlers[pkt_id] = fn

    def start(self) -> None:
        self.running = True

        for target in (
            self._connection_manager,
            self._rx_thread,
            self._manual_tx_thread,
            self._heartbeat_tx_thread,
        ):
            threading.Thread(target = target, daemon = True).start()

    def stop(self) -> None:
        self.running = False

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass

    # Send data to the serial port
    def write(self, data: bytes) -> bool:
        with self._state_lock:
            if not self._connected:
                return False
        
        try:
            with self._serial_lock:
                self.ser.write(data)
            return True
        
        except Exception:
            with self._state_lock:
                self._connected = False
            return False
        
    def set_upload_status(self, status: uploadStatus) -> None:
        with self._upload_lock:
            self._upload_status = status

    # Record new LoRa time
    def update_lora_t(self) -> None:
        with self._state_lock:
            self._last_lora_t = time.time()

    # Status properties
    @property
    def connected(self) -> bool:
        with self._state_lock:
            return self._connected
        
    @property
    def lora_online(self) -> bool:
        with self._state_lock:
            return (
                self._connected
                and self._last_lora_t > 0
                and (time.time() - self._last_lora_t) < LORA_STALE_S
            )
        
    @property
    def wifi_online(self) -> bool:
        with self._state_lock:
            return (
                self._connected
                and self._last_wifi_t > 0
                and (time.time() - self._last_wifi_t) < WIFI_STALE_S
            )
        
    # Backround threads
    def _connection_manager(self) -> None:
        while self.running:
            with self._state_lock:
                conn = self._connected

            if not conn:
                with self._state_lock:
                    self._last_lora_t = 0.0
                    self._last_wifi_t = 0.0

                try:
                    if self.ser:
                        self.ser.close()

                    self.ser = serial.Serial(self.port, self.baud, timeout = 0.1)

                    with self._state_lock:
                        self._connected = True

                    print(f"[RX] Connected on {self.port}")

                except Exception:
                    pass

            time.sleep(1)

    def _rx_thread(self) -> None:
        while self.running:
            with self._state_lock:
                conn = self._connected

            if conn and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        id_byte = self.ser.read(1)
                        pkt_id = id_byte[0]

                        # Drop invalid ID
                        if pkt_id not in self._pkt_sizes:
                            continue
                            
                        total = self._pkt_sizes[pkt_id]
                        to_read = total - 1 # Already red ID 

                        if to_read > 0:
                            rest = self.ser.read(to_read)

                            if len(rest) != to_read:
                                continue 

                            payload = id_byte + rest

                        else:
                            payload = id_byte

                        if pkt_id == PKT_WIFI_HB:
                            with self._state_lock:
                                self._last_wifi_t = time.time()
                            continue

                        handler = self._handlers.get(pkt_id)
                        if handler:
                            handler(payload)

                except Exception:
                    with self._state_lock:
                        self._connected = False
                    
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass

                    self.ser = None

            time.sleep(0.005) # ~200 Hz pollrate

    def _manual_tx_thread(self) -> None:
        while self.running:
            if self.connected and self.ser:
                throttle = int(max(-100, min(100, self.manual_throttle)))
                rudder = int(max(-80, min(80, self.manual_rudder)))

                pkt = struct.pack(MANUAL_FORMAT, PKT_MANUAL, throttle, rudder)
                self.write(pkt)
            
            time.sleep(0.1)

    def _heartbeat_tx_thread(self) -> None:
        while self.running:
            with self._upload_lock:
                uploading = self._upload_status == uploadStatus.UPLOADING

            if self.connected and not uploading and self.ser:
                pkt = struct.pack(ACK_FORMAT, PKT_DATA, 254, 0)
                self.write(pkt)

            time.sleep(2)