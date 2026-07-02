#include "state.h"

#include "errors.h"

uint8_t validateMode(const SystemStatus& status, const SensorData& sensors)
{
    uint8_t mode = status.mode;

    if (!hasError(ERR_INIT)) {return 0;}
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
            if (hasError(ERR_LORA_TIMEOUT)) {return 0;}
            if (hasError(ERR_GPS_ACC_LOW) || hasError(ERR_GPS_FAIL) || hasError(ERR_MAG_ACC_LOW) || hasError(ERR_MAG_FAIL)) {return 0;}
            if (!status.routeReady) {return 0;}
            return 2;
        
        case 3:
            if (!status.homeSet) {return 0;}
            if (!sensors.gps.valid || !sensors.mag.valid) {return 0;}
            return 3;

        default:
            return 0;
    }
}