"""

B uin8
b int8

I uint32

d float64
f float32

"""

import os
import struct

from enum import Enum, auto
from typing import Dict, Tuple

# Struct formats

# Fast telemetry (downlink, ~5 Hz): position, heading, mode, target waypoint
FAST_FORMAT = "<BddfBB"
FAST_SIZE = struct.calcsize(FAST_FORMAT)
 
# Slow telemetry (downlink, ~1 Hz): battery, HDOP, RSSI, error bitmask
SLOW_FORMAT = "<BBBBI"
SLOW_SIZE = struct.calcsize(SLOW_FORMAT)
 
# Waypoint data (uplink): route_id, lat, lon, waypoint order, total count
ROUTE_FORMAT = "<BBddBB"
 
# General-purpose ACK / heartbeat (bidirectional)
ACK_FORMAT = "<BBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)
 
# Mode change command (uplink)
CONTROL_FORMAT = "<BB"
 
# Manual throttle / rudder (uplink, 10 Hz)
MANUAL_FORMAT = "<Bbb"
 
# Error-flag reset command (uplink, single byte)
RESET_ERRORS_FORMAT = "<B"
 
# WiFi heartbeat from boat (downlink)
WIFI_HEARTBEAT_FORMAT = "<BI"
WIFI_HEARTBEAT_SIZE = struct.calcsize(WIFI_HEARTBEAT_FORMAT)
 
# Home waypoint: used for PKT_HOME_SET (uplink) and PKT_HOME_DATA (downlink)
HOME_FORMAT = "<Bdd"
HOME_SIZE = struct.calcsize(HOME_FORMAT)
 
# Unix-time response (uplink, ground → boat)
TIME_DATA_FORMAT = "<BI"
TIME_DATA_SIZE = struct.calcsize(TIME_DATA_FORMAT)

# Packet IDs

PKT_WP_DATA = 0x01 # U : WP data during upload
PKT_TELE_FAST = 0x02 # D : High-rate tele
PKT_TELE_SLOW = 0x03 # D : Low-rate tele
PKT_CONTROL = 0x04 # U : Mode change command
PKT_MANUAL = 0x05 # U : Manual steering commands
PKT_WIFI_HB = 0x06 # D : WiFi hb 
PKT_RESET_ERRORS = 0x07 # U : Clear error data 
PKT_HOME_SET = 0x08 # U : Send new home wp
PKT_HOME_DATA = 0x09 # D : Current home wp
PKT_HOME_REQ = 0x0A # U : Request current home wp 
PKT_TIME_REQ = 0x0B # D : Requests unix time
PKT_TIME_DATA = 0x0C # U : Send unix time
PKT_DATA = 0x10 # U/D : ACK payload / hb

# Modes

MODE_STOP = 0
MODE_MANUAL = 1
MODE_AUTO = 2
MODE_RTH = 3

MODE_NAMES: Dict[int, str] ={
    MODE_STOP : "STOP",
    MODE_MANUAL : "MANUAL",
    MODE_AUTO : "A/P",
    MODE_RTH : "RTH",
}

# Retry / timeout limits

MODE_TIMEOUT_S = 1.5 # Seconds to wait for mode-change ACK
MODE_RETRIES = 5

ROUTE_TIMEOUT_S = 2.0 # Seconds to wait for each wp ACK
ROUTE_RETRIES = 5

HOME_TIMEOUT_S = 2.0 # Seconds to wait for a home set or home data ACK
HOME_RETRIES = 5

LORA_STALE_S = 5.0 # Time limit for LoRa timeout
WIFI_STALE_S = 2.0 # Time limit for WiFi timeout

# Upload state

class uploadStatus(Enum):
    IDLE = auto()
    UPLOADING = auto()
    DONE = auto()
    FAILED = auto()

# Error bit definitions

_DEFAULT_ERROR_BITS : Dict[int, Tuple[str, str]] = {
    0: ("ERR_DEF", "Could not load error definitions from file"),
}

_DEFAULT_ERR_PATH = os.path.join(os.path.dirname(__file__), "error_codes.txt")

def load_error_defs(path: str = _DEFAULT_ERR_PATH) -> Dict[int, Tuple[str, str]]:
    defs: Dict[int, Tuple[str, str]] = dict(_DEFAULT_ERROR_BITS)

    if not os.path.isfile(path):
        print("error_codes.txt not found")
        return defs
    
    with open(path, encoding = "utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue

            bit_str, rest = line.split("=", 1)
            bit_str = bit_str.strip().upper()

            if not bit_str.startswith("BIT"):
                continue
            
            try:
                bit_num = int(bit_str[3:])
            
            except ValueError:
                continue

            name, desc = rest.split(":", 1) if ":" in rest else (rest, "")
            defs[bit_num] = (name.strip(), desc.strip())

    return defs