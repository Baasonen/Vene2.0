#include <Arduino.h>
#include <ESP32Servo.h>

#include "esp_system.h"
#include "esp_task_wdt.h"

#include "magnetometer.h"
#include "gps.h"
#include "lora.h"
#include "state.h"
#include "control.h"
#include "wifiComm.h"
#include "sensorsInit.h"

#define DEBUG true

State globalState = {0};
SemaphoreHandle_t stateMutex;

void setup();
void loop();

void setup()
{
    Serial.begin(115200);
    Serial.println("BOOT...");

    stateMutex = xSemaphoreCreateMutex();

    bool initFail = false;

    initFail = sensorsInit();

    if (DEBUG)
    {
        globalState.status.homeSet = true;
    }

    controlInit();
    WiFiInit();

    xTaskCreatePinnedToCore(commsTask, "CommsTask", 8192, NULL, 1, NULL, 0);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        globalState.status.initFail = initFail;

        xSemaphoreGive(stateMutex);
    }

    esp_task_wdt_init(5, true);
    esp_task_wdt_add(NULL);
}

void loop()
{
    static uint32_t lastLoopStartTime = 0;
    uint32_t loopDuration = millis() - lastLoopStartTime;
    lastLoopStartTime = millis();

    esp_task_wdt_reset();

    MagData mag = getMagnetometer();
    GPSData gps = getGPS();

    WiFiReadIncoming(stateMutex);

    static Route currentRoute = {};
    static SystemStatus localSysStatus = {};
    static ManualControls manual = {};

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(30)) == pdTRUE)
    {
        localSysStatus = globalState.status;
        manual = globalState.manual;

        if (globalState.route.newRouteAvailable)
        {
            currentRoute = globalState.route;
            globalState.route.newRouteAvailable = false;
        }

        xSemaphoreGive(stateMutex);
    }

    uint8_t mode = localSysStatus.mode;
    mode = validateMode(localSysStatus, gps, mag);

    switch (mode)
    {
        case 0:
            // STOP

            turnRudder(0);
            setThrottle(0);

            break;

        case 1:
            // MANUAL
            turnRudder(manual.rudder);
            setThrottle(manual.throttle);

            break;

        case 2:
            // AUTOPILOT

            break;

        case 3: 
            // RETURN HOME (AUTOPILOT TO HOME WP)
            break;

        default:
            mode = 0;
            break;
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(15)) == pdTRUE)
    {
        globalState.gps = gps;
        globalState.mag = mag;

        globalState.status.mode = mode;
        globalState.status.targetIdx = localSysStatus.targetIdx;

        xSemaphoreGive(stateMutex);
    }

    static uint32_t lastStatusPrintTime = millis();
    if (millis() - lastStatusPrintTime > 2000)
    {
        Serial.println("\n--- VENE 2.0 SYSTEM STATUS ---");
        
        Serial.printf("Mode: %u | Loop Time: %u ms\n", mode, loopDuration);
        
        Serial.printf("Controls: Throttle: %d | Rudder: %d\n", manual.throttle, manual.rudder);
        
        Serial.printf("GPS: [%s] Lat: %.6f, Lon: %.6f | Sats: %d | HDOP: %.2f\n", 
                      gps.valid ? "VALID" : "INVALID", 
                      gps.lat, gps.lon, gps.satellites, gps.hdop);
        
        Serial.printf("MAG: [%s] Heading: %.2f | Accuracy: %u (0-3)\n", 
                      mag.valid ? "VALID" : "INVALID", 
                      mag.heading, mag.accuracy);
        
        Serial.printf("Comms: LoRa TO: %s | WiFi TO: %s\n", 
                      localSysStatus.loraTimeout ? "YES" : "NO", 
                      localSysStatus.wifiTimeout ? "YES" : "NO");
                      
        Serial.println("-------------------------------");
        lastStatusPrintTime = millis();
    }

    delay(10);
}