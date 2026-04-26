#pragma once

#include <ESP32Servo.h>

#define RUDDER_PIN 4
#define ESC_PIN 17

#define RUDDER_U_LIM 80
#define RUDDER_L_LIM -80

#define ESC_RANGE 5
#define ESC_NEUTRAL 1500

void controlInit();
void turnRudder(int8_t angle);
void setThrottle(int8_t);