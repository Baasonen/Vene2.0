#include <Arduino.h>

#include "magnetometer.h"
#include "gps.h"
#include "lora.h"
#include "state.h"

State globalState = {};
SemaphoreHandle_t stateMutex;

void setup();
void loop();

void setup()
{
    Serial.begin(115200);

    if (!magInit()) {Serial.println("Magnetometer init fail");}
    if (!GPSInit()) {Serial.println("GPS init failed");}

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

    uint8_t mode;
    uint8_t targetIdx;
    double homeLat, homeLon;
    static Route currentRoute = {};

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
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
            break;

        case 1:
            // MANUAL
            if (globalState.status.commTimeout) {mode = 0;}

            break;

        case 2:
            // AUTOPILOT
            if (globalState.status.commTimeout) {mode = 0;}

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

        xSemaphoreGive(stateMutex);
    }

    //Serial.printf("Heading: %.2f Accuracy: %.2i ", mag.heading, mag.accuracy);
    //Serial.printf("Lat: %.2f Lon: %.2f hdop: %.2f satellites: %i\n", gps.lat, gps.lon, gps.hdop, gps.satellites);
    delay(100);
}