#pragma once

#include <Arduino.h>

#include "tuning.h"

#define CELL1PIN 33
#define CELL2PIN 32
#define CELL3PIN 35

typedef struct
{
    uint8_t c1;
    uint8_t c2;
    uint8_t c3;

    float c1F;
    float c2F;
    float c3F;

    float lowest;
} Battery;

Battery getBattery();