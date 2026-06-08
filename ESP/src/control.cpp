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

void turnRudder(int8_t angle)
{
    if (angle > RUDDER_U_LIM) {angle = RUDDER_U_LIM;}
    if (angle < RUDDER_L_LIM) {angle = RUDDER_L_LIM;}

    rudder.write(angle + 90);
}

void setThrottle(int8_t throttle)
{
    if (abs(throttle) < 10) {throttle = 0;}

    uint32_t throttleMS = (throttle * ESC_RANGE) + ESC_NEUTRAL;
    motor.writeMicroseconds(throttleMS);
}

void steerTo(float targetHeading)
{
    const float Kp = 2.0;
    const float deadzone = 2.0;

    float currentHeading = 0.0;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        currentHeading = globalState.sensors.mag.heading;
        xSemaphoreGive(stateMutex);
    }

    float error = targetHeading - currentHeading;

    if (error > 180.0) {error -= 360.0;}
    if (error < -180.0) {error += 360.0;}

    if (abs(error) < deadzone) {return;}

    int8_t angle = (int8_t) (error * Kp);

    if (angle > RUDDER_U_LIM) {angle = RUDDER_U_LIM;}
    if (angle < RUDDER_L_LIM) {angle = RUDDER_U_LIM;}

    turnRudder(angle);
}