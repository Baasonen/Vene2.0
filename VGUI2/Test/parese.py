import serial
import struct
import time
import sys
import threading

SERIAL_PORT = 'COM4' 
BAUD_RATE = 115200

# < = Little Endian
# B = uint8_t (1 byte)
# d = double (8 bytes)
# f = float (4 bytes)
# I = uint32_t (4 bytes)
# ? = bool (1 byte)

# Format: ID(B), Lat(d), Lon(d), Heading(f), Mode(B), TargetIdx(B)
FAST_FORMAT = "<BddfBB"
FAST_SIZE = struct.calcsize(FAST_FORMAT)

# Format: ID(B), Batt(B), GPS(B), Timeout(?), ErrorCode(I)
SLOW_FORMAT = "<BBB?I"
SLOW_SIZE = struct.calcsize(SLOW_FORMAT)

ROUTE_FORMAT = "<BBddBB"

ACK_FORMAT = "<BBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)

boat_data = {}
ack_status = -1

def send_test_route(ser):
    global ack_status
    waypoints = []
    start_lat, start_lon = 60.2055, 24.6559
    lat_step = 0.00015  
    lon_step = 0.00030 

    for row in range(5):
        current_lat = start_lat + (row * lat_step)

        if row % 2 == 0:
            col_range = range(10)
        else:
            col_range = range(9, -1, -1)

        for col in col_range:
            current_lon = start_lon + (col * lon_step)
            waypoints.append((round(current_lat, 6), round(current_lon, 6)))
        ammnt = len(waypoints)

    pkt_ack_id = 0x10
    pkt_route_id = 0x01
    route_id = 100

    for order, (lat, lon) in enumerate(waypoints):
        max_retries = 5
        success = False
        
        for attempt in range(max_retries):
            ack_status = -1

            pkt = struct.pack(ROUTE_FORMAT, pkt_route_id, route_id, lat, lon, order, ammnt)
            ser.write(pkt)

            timeout = time.time() + 2
            while ack_status != order and time.time() < timeout:
                time.sleep(0.05)
            
            if ack_status == order:
                print(f">>> Waypoin {order + 1} / {ammnt} acknowledged")
                success = True
                break
                
            else:
                print(f">>> Waypoint {order +1 } / {ammnt} timeout, retrying {attempt + 1} / {max_retries}...")

        if not success:
            print(">>> ROUTE UPLOAD STOPPED: Max retries reached")
            return
    
    print(">>> Route upload finished")
    time.sleep(2)

def parse_fast_telemetry(data):
    unpacked = struct.unpack(FAST_FORMAT, data)
    return {
        "lat": unpacked[1],
        "lon": unpacked[2],
        "heading": unpacked[3],
        "mode": unpacked[4],
        "target_idx": unpacked[5]
    }

def parse_slow_telemetry(data):
    unpacked = struct.unpack(SLOW_FORMAT, data)
    return {
        "battery": unpacked[1],
        "hdop": unpacked[2] / 10.0, # Convert back to float
        "timeout": unpacked[3],
        "error": unpacked[4]
    }

def user_input(ser):
    while True:
        cmd = input()
        if cmd.strip().lower() == 'r':
            send_test_route(ser)

def main():
    global ack_status
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    threading.Thread(target=user_input, args=(ser,), daemon=True).start()

    while True:
        if ser.in_waiting > 0:
            byte = ser.read(1)
            
            if byte == b'\x02':  # PKT_TELE_FAST
                payload = byte + ser.read(FAST_SIZE - 1)
                if len(payload) == FAST_SIZE:
                    boat_data.update(parse_fast_telemetry(payload))
            
            elif byte == b'\x03':  # PKT_TELE_SLOW
                payload = byte + ser.read(SLOW_SIZE - 1)
                if len(payload) == SLOW_SIZE:
                    boat_data.update(parse_slow_telemetry(payload))

            elif byte == b'\x10':  # PKT_ACK
                payload = byte + ser.read(ACK_SIZE - 1)
                if len(payload) == ACK_SIZE:
                    ack_data = struct.unpack(ACK_FORMAT, payload)
                    ack_status = ack_data[2]

            # Update the console display
            sys.stdout.write("\033[H\033[J")
            print(f"--- BOAT STATUS --- (Press 'r' + Enter to send test route)")
            print(f"Location:  {boat_data.get('lat', 0):.7f}, {boat_data.get('lon', 0):.7f}")
            print(f"Heading:   {boat_data.get('heading', 0):.1f}°")
            print(f"Target WP: {boat_data.get('target_idx', 0)}")
            print(f"Mode:      {boat_data.get('mode', 0)}")
            print(f"Battery:   {boat_data.get('battery', 0)}%")
            print(f"HDOP:      {boat_data.get('hdop', 0.0)}")
            print(f"Error:     {boat_data.get('error', 0)}")
            sys.stdout.flush()

        time.sleep(0.01) 

if __name__ == "__main__":
    main()