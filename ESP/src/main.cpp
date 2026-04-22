#include <Arduino.h>
#include <ESP32Servo.h>

#include "magnetometer.h"
#include "gps.h"
#include "lora.h"
#include "state.h"
#include "control.h"
#include "wifiComm.h"

State globalState = {};
SemaphoreHandle_t stateMutex;

static Servo rudder;

void setup();
void loop();

void setup()
{
    Serial.begin(115200);

    rudder.attach(4);
    rudder.write(90);

    if (!magInit()) {Serial.println("Magnetometer init fail");}
    if (!GPSInit()) {Serial.println("GPS init failed");}

    controlInit();
    WiFiInit();

    stateMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        commsTask,
        "CommsTask",
        4096,
        NULL,
        1,
        NULL,
        0
    );


}

void loop()
{
    MagData mag = getMagnetometer();
    GPSData gps = getGPS();

    WiFiReadIncoming(stateMutex);

    uint8_t mode;
    uint8_t targetIdx;
    double homeLat, homeLon;

    static Route currentRoute = {};
    static SystemStatus localSysStatus = {};
    static ManualControls manual = {};

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        localSysStatus = globalState.status;
        manual = globalState.manual;

        mode = globalState.status.mode;
        targetIdx = globalState.targetIdx;
        homeLat = globalState.home.lat;
        homeLon = globalState.home.lon;

        if (globalState.route.newRouteAvailable)
        {
            currentRoute = globalState.route;
            globalState.route.newRouteAvailable = false;
        }

        xSemaphoreGive(stateMutex);
    }

    switch (mode)
    {
        case 0:
            // STOP

            turnRudder(90);
            setThrottle(0);

            break;

        case 1:
            // MANUAL

            if (localSysStatus.wifiTimeout) {mode = 0;}
            else
            {
                turnRudder(manual.rudder);
                setThrottle(manual.throttle);
            }

            break;

        case 2:
            // AUTOPILOT
            if (localSysStatus.loraTimeout && (millis() - localSysStatus.commTimeoutTriggerTime) < 30000) {mode = 0;}

            break;

        case 3: 
            // RETURN HOME (AUTOPILOT TO HOME WP)
            break;

        default:
            mode = 0;
            break;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        globalState.gps = gps;
        globalState.mag = mag;

        globalState.status.mode = mode;
        globalState.targetIdx = targetIdx;

        globalState.status.mode = mode;

        xSemaphoreGive(stateMutex);
    }

    //Serial.printf("Heading: %.2f Accuracy: %.2i ", mag.heading, mag.accuracy);
    //Serial.printf("Lat: %.2f Lon: %.2f hdop: %.2f satellites: %i\n", gps.lat, gps.lon, gps.hdop, gps.satellites);
    delay(100);
}