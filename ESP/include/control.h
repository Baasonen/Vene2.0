#pragma once

#include <ESP32Servo.h>

#include "state.h"
#include "tuning.h"

#define RUDDER_PIN 4
#define ESC_PIN 17

void controlInit();
void turnRudder(int8_t angle);
void setThrottle(int8_t);
void steerTo(float targetHeading);
void resetSteering();