#pragma once

#include <TinyGPS++.h>

#define GPSRXPIN 5
#define GPSTXPIN 18

typedef struct 
{
    double lat, lon;
    float hdop;
    int satellites;
    double speedKMH;
    uint32_t time;
    bool valid;
} GPSData;

int GPSInit();
GPSData getGPS();