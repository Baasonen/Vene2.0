#include "control.h"

static Servo rudder;
static Servo motor;

static float iError = 0.0f;
static uint32_t lastSteerTime = 0;

void resetSteering()
{
    iError = 0.0f;
    lastSteerTime = 0;
}

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
    const float Kp = RUDDER_KP;
    const float Ki = RUDDER_KI;
    const float deadzone = 2.0f;
    const float integralLimit = RUDDER_I_LIM;

    float currentHeading = 0.0f;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        currentHeading = globalState.sensors.mag.heading;
        xSemaphoreGive(stateMutex);
    }

    float error = targetHeading - currentHeading;
    if (error > 180.0f) {error -= 360.0f;}
    if (error < -180.0f) {error += 360.0f;}

    uint32_t now = millis();
    float dt = (lastSteerTime == 0) ? 0.05f : (now - lastSteerTime) / 1000.0f;
    lastSteerTime = now;

    if (abs(error) < deadzone) {return;}

    float rawAngle = (error * Kp) + (iError * Ki);
    bool saturated = (rawAngle > RUDDER_U_LIM) || (rawAngle < RUDDER_L_LIM);

    if (!saturated)
    {
        iError += error * dt;
        if (iError > integralLimit) {iError = integralLimit;}
        if (iError < -integralLimit) {iError = -integralLimit;}
    }

    if (rawAngle > RUDDER_U_LIM) {rawAngle = RUDDER_U_LIM;}
    if (rawAngle < RUDDER_L_LIM) {rawAngle = RUDDER_L_LIM;}

    turnRudder((int8_t)rawAngle);
}