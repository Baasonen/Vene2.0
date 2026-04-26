import serial
import struct
import time
import sys
import threading
import tkinter as tk
from tkinter import ttk

# --- CONFIGURATION ---
SERIAL_PORT = 'COM4' 
BAUD_RATE = 115200

# --- STRUCT FORMATS ---
FAST_FORMAT = "<BddfBB"
FAST_SIZE = struct.calcsize(FAST_FORMAT)

SLOW_FORMAT = "<BBBBI" 
SLOW_SIZE = struct.calcsize(SLOW_FORMAT)

ROUTE_FORMAT = "<BBddBB"
ACK_FORMAT = "<BBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)

CONTROL_FORMAT = "<BB" 
MANUAL_FORMAT = "<Bbb"  # ID(B), Throttle(b), Rudder(b) -> Sent to ESP via Serial

WIFI_HEARTBEAT_FORMAT = "<BI" # ID(B), Data(I) -> Rcvd from ESP via Serial
WIFI_HEARTBEAT_SIZE = struct.calcsize(WIFI_HEARTBEAT_FORMAT)

class BoatController:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, serial_port=SERIAL_PORT, baud_rate=BAUD_RATE):
        if hasattr(self, 'initialized'):
            return
        
        self.boat_data = {
            "lat": 0.0, "lon": 0.0, "heading": 0.0,
            "mode": 0, "target_idx": 0, "battery": 0,
            "hdop": 0.0, "signal": 0, "error": 0
        }
        self.ack_status = -1
        self.mode_ack_status = -1
        
        # Connections States
        self.last_lora_time = 0
        self.last_wifi_time = 0
        self.receiver_connected = False
        
        # Manual Controls State
        self.manual_throttle = 0
        self.manual_rudder = 90  # Default neutral rudder
        
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None

        self.running = True
        self.initialized = True

        self._serial_lock = threading.Lock()

        # Start background threads, including the new connection manager
        threading.Thread(target=self._connection_manager, daemon=True).start()
        threading.Thread(target=self._serial_rx_thread, daemon=True).start()
        threading.Thread(target=self._manual_tx_thread, daemon=True).start()
        threading.Thread(target=self._heartbeat_tx_thread, daemon=True).start()

    def _connection_manager(self):
        """Continuously monitors and attempts to reconnect to the Serial port if disconnected."""
        while self.running:
            if not self.receiver_connected:
                try:
                    if self.ser:
                        self.ser.close()
                    self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
                    self.receiver_connected = True
                    print(f"Connected to Receiver ESP on {self.serial_port}")
                except Exception:
                    self.receiver_connected = False
            time.sleep(1) # Check connection status every second

    def set_mode(self, mode):
        if not self.receiver_connected:
            print("Cannot change mode when receiver offline")
            return
        
        threading.Thread(target=self._set_mode_task, args=(mode,), daemon=True).start()


    def _set_mode_task(self, mode):
        success = False
        for attempt in range(3):
            if not self.receiver_connected:
                print("MODE CHANGE ABORTED: Receiver disconnect")
                return

            self.mode_ack_status = -1
            try:
                pkt = struct.pack(CONTROL_FORMAT, 0x04, mode)
                with self._serial_lock:
                    self.ser.write(pkt)
                print(f"Requested mode change to {mode}, attempt {attempt + 1}")
            except Exception:
                self.receiver_connected = False
                return
            
            timeout = time.time() + 2
            while self.mode_ack_status != mode and time.time() < timeout:
                time.sleep(0.05)
            
            if self.mode_ack_status == mode:
                print("Mode ack")
                success = True
                break
            else:
                print("Ack timeout")
        
        if not success:
            print("Mode change failed, max retries reached")

    def send_test_route(self):
        if not self.receiver_connected: 
            print("Cannot upload route: Receiver offline.")
            return
        threading.Thread(target=self._upload_route_task, daemon=True).start()

    def _upload_route_task(self):
        waypoints = []
        start_lat, start_lon = 60.2055, 24.6559
        lat_step, lon_step = 0.00015, 0.00030 

        for row in range(5):
            current_lat = start_lat + (row * lat_step)
            col_range = range(10) if row % 2 == 0 else range(9, -1, -1)
            for col in col_range:
                current_lon = start_lon + (col * lon_step)
                waypoints.append((round(current_lat, 6), round(current_lon, 6)))
        
        ammnt = len(waypoints)
        route_id = 100

        for order, (lat, lon) in enumerate(waypoints):
            success = False
            for attempt in range(5):
                if not self.receiver_connected:
                    print("> ROUTE UPLOAD ABORTED: Receiver disconnected")
                    return

                self.ack_status = -1
                try:
                    pkt = struct.pack(ROUTE_FORMAT, 0x01, route_id, lat, lon, order, ammnt)
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
                    return

                timeout = time.time() + 2
                while self.ack_status != order and time.time() < timeout:
                    time.sleep(0.05)
                
                if self.ack_status == order:
                    print(f"> Waypoint {order + 1}/{ammnt} acknowledged")
                    success = True
                    break
                else:
                    print(f"> Waypoint {order + 1}/{ammnt} timeout, retrying...")

            if not success:
                print("> ROUTE UPLOAD STOPPED: Max retries reached")
                return
        print("> Route upload finished successfully")

    def _serial_rx_thread(self):
        """Listens to ALL incoming serial data (LoRa Telemetry + WiFi Heartbeats) safely."""
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    if self.ser.in_waiting > 0:
                        byte = self.ser.read(1)
                        
                        if byte == b'\x02':  # PKT_TELE_FAST (LoRa)
                            payload = byte + self.ser.read(FAST_SIZE - 1)
                            if len(payload) == FAST_SIZE:
                                unpacked = struct.unpack(FAST_FORMAT, payload)
                                self.boat_data.update({
                                    "lat": unpacked[1], "lon": unpacked[2], "heading": unpacked[3],
                                    "mode": unpacked[4], "target_idx": unpacked[5]
                                })
                                self.last_lora_time = time.time()
                        
                        elif byte == b'\x03':  # PKT_TELE_SLOW (LoRa)
                            payload = byte + self.ser.read(SLOW_SIZE - 1)
                            if len(payload) == SLOW_SIZE:
                                unpacked = struct.unpack(SLOW_FORMAT, payload)
                                self.boat_data.update({
                                    "battery": unpacked[1], "hdop": unpacked[2] / 10.0,
                                    "signal": unpacked[3], "error": unpacked[4]
                                })
                                self.last_lora_time = time.time()

                        elif byte == b'\x10':  # PKT_ACK (LoRa)
                            payload = byte + self.ser.read(ACK_SIZE - 1)
                            if len(payload) == ACK_SIZE:
                                _, ack_id, ack_val = struct.unpack(ACK_FORMAT, payload)

                                if ack_id == 255:
                                    self.mode_ack_status = ack_val
                                else:
                                    self.ack_status = struct.unpack(ACK_FORMAT, payload)[2]

                                self.last_lora_time = time.time()

                        elif byte == b'\x06':  # PKT_WIFI_HEARTBEAT (WiFi -> ESP -> Serial)
                            payload = byte + self.ser.read(WIFI_HEARTBEAT_SIZE - 1)
                            if len(payload) == WIFI_HEARTBEAT_SIZE:
                                self.last_wifi_time = time.time()
                except Exception:
                    # Connection lost or unplugged
                    self.receiver_connected = False
                    if self.ser:
                        self.ser.close()
                    self.ser = None
                    
            time.sleep(0.01)

    def _manual_tx_thread(self):
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    pkt = struct.pack(MANUAL_FORMAT, 0x05, int(self.manual_throttle), int(self.manual_rudder))
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
            time.sleep(0.1)

    def _heartbeat_tx_thread(self):
        while self.running:
            if self.receiver_connected and self.ser:
                try:
                    pkt = struct.pack(ACK_FORMAT, 0x10, 254, 0)
                    with self._serial_lock:
                        self.ser.write(pkt)
                except Exception:
                    self.receiver_connected = False
            time.sleep(2)


class BoatGUI:
    def __init__(self, root, controller):
        self.root = root
        self.controller = controller
        self.root.title("Vene 2.0 Ground Station")
        self.root.geometry("600x480")
        
        self.setup_ui()
        self.update_ui()

    def setup_ui(self):
        # --- Telemetry Frame ---
        tele_frame = tk.LabelFrame(self.root, text="Telemetry", padx=10, pady=10)
        tele_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.lbl_loc = ttk.Label(tele_frame, text="Location: -")
        self.lbl_loc.pack(anchor="w")
        
        self.lbl_head = ttk.Label(tele_frame, text="Heading: -")
        self.lbl_head.pack(anchor="w")
        
        self.lbl_nav = ttk.Label(tele_frame, text="Target WP: - | Mode: -")
        self.lbl_nav.pack(anchor="w")

        self.lbl_stats = ttk.Label(tele_frame, text="Batt: -% | HDOP: - | Err: -")
        self.lbl_stats.pack(anchor="w")

        # --- Status Frame ---
        status_frame = tk.LabelFrame(self.root, text="Connection Status", padx=10, pady=10)
        status_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.lbl_receiver = ttk.Label(status_frame, text="Receiver: OFFLINE", foreground="red")
        self.lbl_receiver.pack(anchor="w")

        self.lbl_lora = ttk.Label(status_frame, text="LoRa: OFFLINE", foreground="red")
        self.lbl_lora.pack(anchor="w")

        self.lbl_wifi = ttk.Label(status_frame, text="WiFi: OFFLINE", foreground="red")
        self.lbl_wifi.pack(anchor="w")

        # --- Control Frame ---
        ctrl_frame = tk.LabelFrame(self.root, text="System Controls", padx=10, pady=10)
        ctrl_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        ttk.Button(ctrl_frame, text="STOP (0)", command=lambda: self.controller.set_mode(0)).grid(row=0, column=0, padx=5)
        ttk.Button(ctrl_frame, text="MANUAL (1)", command=lambda: self.controller.set_mode(1)).grid(row=0, column=1, padx=5)
        ttk.Button(ctrl_frame, text="AUTO (2)", command=lambda: self.controller.set_mode(2)).grid(row=0, column=2, padx=5)
        ttk.Button(ctrl_frame, text="RTL (3)", command=lambda: self.controller.set_mode(3)).grid(row=0, column=3, padx=5)
        ttk.Button(ctrl_frame, text="Upload Test Route", command=self.controller.send_test_route).grid(row=0, column=4, padx=20)

        # --- Manual Steering Frame ---
        man_frame = tk.LabelFrame(self.root, text="Manual Driving (via ESP WiFi)", padx=10, pady=10)
        man_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")

        ttk.Label(man_frame, text="Throttle (-100 to 100)").grid(row=0, column=0)
        self.scale_throttle = ttk.Scale(man_frame, from_=-100, to=100, orient="horizontal", length=200)
        self.scale_throttle.grid(row=0, column=1, padx=10)
        self.scale_throttle.set(0)

        ttk.Label(man_frame, text="Rudder (10 to 170)").grid(row=1, column=0)
        self.scale_rudder = ttk.Scale(man_frame, from_=10, to=170, orient="horizontal", length=200) 
        self.scale_rudder.grid(row=1, column=1, padx=10)
        self.scale_rudder.set(90)

        ttk.Button(man_frame, text="Reset to Neutral", command=self.reset_controls).grid(row=0, column=2, rowspan=2, padx=10)

    def reset_controls(self):
        self.scale_throttle.set(0)
        self.scale_rudder.set(90)

    def update_ui(self):
        data = self.controller.boat_data
        
        self.lbl_loc.config(text=f"Location: {data['lat']:.6f}, {data['lon']:.6f}")
        self.lbl_head.config(text=f"Heading: {data['heading']:.1f}°")
        
        mode_str = {0:"STOP", 1:"MANUAL", 2:"AUTO", 3:"RTL"}.get(data['mode'], "UNKNOWN")
        self.lbl_nav.config(text=f"Target WP: {data['target_idx']} | Mode: {mode_str}")
        self.lbl_stats.config(text=f"Batt: {data['battery']}% | HDOP: {data['hdop']:.1f} | Err: {data['error']}")

        # Receiver Status Update
        if self.controller.receiver_connected:
            self.lbl_receiver.config(text=f"Receiver ({self.controller.serial_port}): ONLINE", foreground="green")
        else:
            self.lbl_receiver.config(text="Receiver: OFFLINE", foreground="red")

        curr_time = time.time()
        
        if curr_time - self.controller.last_lora_time < 2.0:
            self.lbl_lora.config(text="LoRa: ONLINE", foreground="green")
        else:
            self.lbl_lora.config(text="LoRa: OFFLINE", foreground="red")

        if curr_time - self.controller.last_wifi_time < 2.0:
            self.lbl_wifi.config(text="WiFi: ONLINE", foreground="green")
        else:
            self.lbl_wifi.config(text="WiFi: OFFLINE", foreground="red")

        self.controller.manual_throttle = self.scale_throttle.get()
        self.controller.manual_rudder = self.scale_rudder.get()

        self.root.after(100, self.update_ui)

if __name__ == "__main__":
    controller = BoatController()
    root = tk.Tk()
    app = BoatGUI(root, controller)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False