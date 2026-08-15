#pragma once

#include <stdint.h>

#include "magnetometer.h"
#include "gps.h"

typedef struct 
{
    double lat;
    double lon;

    float gpsHeadingDeg;
    float speedKMH;

    bool valid;
} PosSol;

void EKFInit();
PosSol EKFUpdate(const GPSData& gps, const MagData& mag);