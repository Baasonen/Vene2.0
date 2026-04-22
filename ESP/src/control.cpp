#include "control.h"

static Servo rudder;
static Servo motor;

void controlInit()
{
    rudder.attach(RUDDER_PIN);
    rudder.write(90);

    motor.attach(ESC_PIN);
    motor.writeMicroseconds(ESC_NEUTRAL);
}

void turnRudder(uint8_t angle)
{
    if (angle > RUDDER_U_LIM) {angle = RUDDER_U_LIM;}
    if (angle < RUDDER_L_LIM) {angle = RUDDER_L_LIM;}

    rudder.write(angle);
}

void setThrottle(int8_t throttle)
{
    if (abs(throttle) < 10) {throttle = 0;}

    uint32_t throttleMS = (throttle * ESC_RANGE) + ESC_NEUTRAL;
    motor.writeMicroseconds(throttleMS);
}