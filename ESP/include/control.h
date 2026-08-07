#pragma once

#include <ESP32Servo.h>

#include "state.h"

#define RUDDER_PIN 4
#define ESC_PIN 17

#define RUDDER_U_LIM 80
#define RUDDER_L_LIM -80

#define RUDDER_KP 2.0f
#define RUDDER_KI 0.03f
#define RUDDER_I_LIM 40.0f

#define ESC_RANGE 2.0f
#define ESC_REVERSE_DIV 4
#define ESC_NEUTRAL 1500

void controlInit();
void turnRudder(int8_t angle);
void setThrottle(int8_t);
void steerTo(float targetHeading);
void resetSteering();