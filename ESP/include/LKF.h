#pragma once

#include <stdint.h>

#include "gps.h"
#include "magnetometer.h"
#include "tuning.h"

typedef struct 
{
    double lat;
    double lon;

    float gpsHeadingDeg;
    float speedKMH;

    float posStdM;
    float velStdMpS;

    float posNIS;
    float velNIS;
    float posNISAvg;

    bool accurate;
    bool valid;
} PosSol;

void LKFInit();
PosSol LKFUpdate(const GPSData& gps, const MagData& mag);