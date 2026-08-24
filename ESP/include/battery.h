#pragma once

#include <Arduino.h>

#include "tuning.h"

#define CELL1PIN 1
#define CELL2PIN 2
#define CELL3PIN 3

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