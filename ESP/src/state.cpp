#include "state.h"

uint8_t validateMode(const SystemStatus& status, const GPSData& gps, const MagData& mag)
{
    uint8_t mode = status.mode;

    if (status.initFail || !status.homeSet) {return 0;}
    if (status.loraTimeout && (millis() - status.commTimeoutTriggerTime > 30000)) {mode = 3;}

    //TODO: Check correct logic

    switch (mode)
    {
        case 1:
            // Manual
            if (status.wifiTimeout) {return 0;}
            if (status.loraTimeout) {return 0;}
            return 1;

        case 2:
            // Autopilot
            if (status.loraTimeout) {return 0;}
            if (!gps.valid || !mag.valid) {return 0;}
            if (gps.hdop > GPS_MAX_HDOP) {return 0;}
            if (mag.accuracy < MAG_MIN_ACC) {return 0;}
            if (!status.routeReady) {return 0;}
            return 2;

        case 3:
            if (!status.homeSet) {return 0;}
            if (gps.hdop > GPS_MAX_HDOP) {return 0;}
            if (!gps.valid) {return 0;}
            return 3;

        default:
            return 0;
    }
}