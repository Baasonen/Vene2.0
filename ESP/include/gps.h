#pragma once

#include <stdint.h>

#define GPSRXPIN 5
#define GPSTXPIN 18

typedef struct 
{
    double lat;
    double lon;

    float speedKMH;
    float headingDeg;

    int32_t velN;
    int32_t velE;
    uint32_t velAccM;

    float hAccM;
    uint8_t fixType;
    uint8_t satellites;
    uint32_t unixTime;
    bool utcTimeValid;
    bool accDegraded;
    bool valid;
} GPSData;

int GPSInit();
GPSData getGPS();

void gpsInitAid(double lat, double lon, float altM, uint32_t unix);
