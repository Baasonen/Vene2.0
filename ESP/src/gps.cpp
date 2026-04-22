#include "gps.h"

static TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);

int GPSInit()
{
    gpsSerial.begin(9600, SERIAL_8N1, GPSRXPIN, GPSTXPIN);
    return 1;
}

GPSData getGPS()
{
    static uint32_t lastValid;

    static GPSData data = {};

    while (gpsSerial.available() > 0) {gps.encode(gpsSerial.read());}

    if (gps.location.isValid())
    {   
        if (gps.hdop.hdop() < 1.5)
        {
            data.lat = gps.location.lat();
            data.lon = gps.location.lng();
            data.speedKMH = gps.speed.kmph();
            data.hdop=gps.hdop.hdop();
            data.satellites = gps.satellites.value();
            data.time = gps.time.value();

            lastValid = millis();
            data.valid = true;
        }
    }

    if ((millis() - lastValid) > 500) {data.valid = false;}

    return data;
}