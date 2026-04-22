import serial
import struct
import time
import sys

SERIAL_PORT = 'COM3' 
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

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"Connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    boat_data = {}

    while True:
        if ser.in_waiting > 0:
            # Look for a Start-of-Packet ID
            byte = ser.read(1)
            
            if byte == b'\x02':  # PKT_TELE_FAST
                payload = byte + ser.read(FAST_SIZE - 1)
                if len(payload) == FAST_SIZE:
                    boat_data.update(parse_fast_telemetry(payload))
            
            elif byte == b'\x03':  # PKT_TELE_SLOW
                payload = byte + ser.read(SLOW_SIZE - 1)
                if len(payload) == SLOW_SIZE:
                    boat_data.update(parse_slow_telemetry(payload))

            # Update the console display
            sys.stdout.write("\033[H\033[J")
            print(f"--- BOAT STATUS ---")
            print(f"Location: {boat_data.get('lat', 0):.7f}, {boat_data.get('lon', 0):.7f}")
            print(f"Heading:  {boat_data.get('heading', 0):.1f}°")
            print(f"Target WP: {boat_data.get('target_idx', 0)}")
            print(f"Mode:     {boat_data.get('mode', 0)}")
            print(f"Battery:  {boat_data.get('battery', 0)}%")
            print(f"HDOP:     {boat_data.get('hdop', 0.0)}")
            print(f"Error:    {boat_data.get('error', 0)}")
            sys.stdout.flush()

        time.sleep(0.01) 

if __name__ == "__main__":
    main()