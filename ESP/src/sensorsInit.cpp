#include "sensorsInit.h"

bool sensorsInit()
{
    int initFailed = false;

    if (!magInit()) 
    {
        Serial.println("[INIT] Magnetometer init failed");
        initFailed = true;
    }

    if (!GPSInit())
    {
        Serial.println("[INIT] GPS init failed");
        initFailed = true;
    }

    if (!initFailed) {Serial.println("[INIT] Sensors init success");}
    return initFailed;
}