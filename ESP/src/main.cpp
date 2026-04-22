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

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        globalState.gps = gps;
        globalState.mag = mag;

        xSemaphoreGive(stateMutex);
    }

    //Serial.printf("Heading: %.2f Accuracy: %.2i ", mag.heading, mag.accuracy);
    //Serial.printf("Lat: %.2f Lon: %.2f hdop: %.2f satellites: %i\n", gps.lat, gps.lon, gps.hdop, gps.satellites);
    delay(100);
}