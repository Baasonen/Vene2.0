#pragma once

#include <stdint.h>

#include "tuning.h"

#define GPSRXPIN 5
#define GPSTXPIN 18

typedef struct 
{
    double lat;
    double lon;

    float speedKMH;
    float headingDeg;

    float velN;
    float velE;
    float velAccM;

    float hAccM;
    uint8_t fixType;
    uint8_t satellites;
    uint32_t unixTime;
    bool utcTimeValid;
    bool accDegraded;
    bool freshFix;
    bool valid;
} GPSData;

int GPSInit();
GPSData getGPS();

void gpsInitAid(double lat, double lon, float altM, uint32_t unix);
