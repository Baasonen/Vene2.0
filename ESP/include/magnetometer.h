#pragma once

#include <Adafruit_BNO08x.h>
#include <Arduino.h>

#define SH2_ARVR_STABILIZED_ROTATION_VECTOR 0x28

typedef struct 
{
    float heading;
    unsigned char accuracy;
    bool valid;
} MagData;

int magInit();
MagData getMagnetometer();