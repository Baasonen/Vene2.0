#include "sensorsInit.h"

bool sensorsInit()
{
    int initFailed = false;

    if (!magInit()) 
    {
        Serial.println("Magnetometer init failed");
        initFailed = true;
    }

    if (!GPSInit())
    {
        Serial.println("GPS init failed");
        initFailed = true;
    }

    return initFailed;
}