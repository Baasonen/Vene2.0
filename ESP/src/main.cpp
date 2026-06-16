#include <Arduino.h>
#include <ESP32Servo.h>
#include <Preferences.h>

#include "esp_system.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "magnetometer.h"
#include "gps.h"
#include "lora.h"
#include "state.h"
#include "control.h"
#include "wifiComm.h"
#include "sensorsInit.h"
#include "navigation.h"

#define WDT_TIMEOUT 5
#define DEBUG true

#define AP_THROTTLE 50
#define WP_TRESHOLD 3

Preferences prefs;

State globalState = {0};
SemaphoreHandle_t stateMutex;
QueueHandle_t sensorQueue;

// CORE 1 | PRIORITY 4 | 50HZ
void sensorTask(void* pv)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        SensorData data;
        data.gps = getGPS();
        data.mag = getMagnetometer();

        xQueueOverwrite(sensorQueue, &data);
    }
}

// CORE 1 | PRIORITY 3 | 20HZ
void controlTask(void* pv)
{
    esp_task_wdt_add(NULL);
 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50); 
    SensorData sensors = {};
    SystemStatus status = {};
    ManualControls manual = {};
    Route route  = {};
 
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        esp_task_wdt_reset();
 
        xQueuePeek(sensorQueue, &sensors, 0);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            status = globalState.status;
            manual = globalState.manual;
 
            route = globalState.route;
            globalState.route.newRouteAvailable = false;
 
            xSemaphoreGive(stateMutex);
        }
 
        uint8_t mode = validateMode(status, sensors);
 
        switch (mode)
        {
            case 0: // STOP
                turnRudder(0);
                setThrottle(0);
                break;
 
            case 1: // MANUAL
                turnRudder(manual.rudder);
                setThrottle(manual.throttle);
                break;
 
            case 2: // AUTOPILOT
                if (status.targetWaypoint == 0) {status.targetWaypoint = 1;}

                if (distanceToPoint(sensors.gps.lat, sensors.gps.lon, route.waypoints[status.targetWaypoint].lat, route.waypoints[status.targetWaypoint].lon) <= WP_TRESHOLD)
                {
                    status.targetWaypoint++;

                    if (status.targetWaypoint > route.length) {status.mode = 3;}
                }

                setThrottle(AP_THROTTLE);
                steerTo(headingToPoint(sensors.gps.lat, sensors.gps.lon, route.waypoints[status.targetWaypoint].lat, route.waypoints[status.targetWaypoint].lon));

                break;
 
            case 3: // RETURN HOME
                
                if (distanceToPoint(status.home.lat, status.home.lon, sensors.gps.lat, sensors.gps.lon) > 10.0)
                {
                    setThrottle(AP_THROTTLE);
                    steerTo(headingToPoint(sensors.gps.lat, sensors.gps.lon, status.home.lat, status.home.lon));
                }
                else
                {
                    turnRudder(0);
                    setThrottle(0);
                }

                break;
 
            default:
                turnRudder(0);
                setThrottle(0);
                break;
        }
 
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.sensors = sensors;
            globalState.status.mode = mode;
            globalState.status.targetWaypoint = status.targetWaypoint;
            xSemaphoreGive(stateMutex);
        }
    }
}

// CORE 1 | PRIORITY 2 | 20HZ
void wifiTask(void* pv)
{
    for(;;)
    {
        WiFiReadIncoming(stateMutex);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// CORE 0 | PRIORITY 1 | 0.5 HZ
void diagTask(void* pd)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));

        SensorData sensors = {};
        SystemStatus status = {};

        xQueuePeek(sensorQueue, &sensors, 0);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            status = globalState.status;
            xSemaphoreGive(stateMutex);
        }

        Serial.println("\n--- VENE 2.0 ---");
        Serial.printf("Mode:    %u\n", status.mode);
        Serial.printf("GPS:     [%s]  %.6f, %.6f  Sats: %d  HDOP: %.2f\n",
                      sensors.gps.valid ? "OK " : "BAD",
                      sensors.gps.lat, sensors.gps.lon,
                      sensors.gps.satellites, sensors.gps.hdop);
        Serial.printf("Heading: [%s]  %.1f deg  Acc: %u/3\n",
                      sensors.mag.valid ? "OK " : "BAD",
                      sensors.mag.heading, sensors.mag.accuracy);
        Serial.printf("Comms:   LoRa TO: %s  WiFi TO: %s\n",
                      status.loraTimeout ? "YES" : "NO",
                      status.wifiTimeout ? "YES" : "NO");
        Serial.printf("LoRa RSSI: %i dBm\n", status.loraRSSI);
        Serial.println("----------------");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("BOOT...");

    stateMutex = xSemaphoreCreateMutex();
    sensorQueue = xQueueCreate(1, sizeof(SensorData));

    prefs.begin("home", false);
    double savedLat = prefs.getDouble("hLat", 0.0);
    double savedLon = prefs.getDouble("hLon", 0.0);
    prefs.end();

    if (savedLat != 0.0 && savedLon != 0.0) {
        globalState.status.home.lat = savedLat;
        globalState.status.home.lon = savedLon;
        globalState.status.homeSet = true;
        Serial.printf("[PREFS] Loaded Home: %.6f, %.6f\n", savedLat, savedLon);
    }

    bool initFail = sensorsInit();

    if (initFail) {Serial.println("[ERR] Sensor init failed");}

    if (DEBUG) {globalState.status.homeSet = true;}

    controlInit();
    WiFiInit();

    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    // CORE 1
    xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(controlTask, "Control", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(wifiTask, "WiFi", 4096, NULL, 2, NULL, 1);

    // CORE 0
    xTaskCreatePinnedToCore(commsTask, "Comms", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(diagTask, "Diag", 4096, NULL, 1, NULL, 0);

    esp_task_wdt_delete(NULL);
    //vTaskDelete(NULL);
}

void loop()
{
    if (globalState.status.homeNeedsSave)
    {
        double hLat = 0.0;
        double hLon = 0.0;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            hLat = globalState.status.home.lat;
            hLon = globalState.status.home.lon;
            globalState.status.homeNeedsSave = false;
            xSemaphoreGive(stateMutex);

            prefs.begin("home", false);
            prefs.putDouble("hLat", hLat);
            prefs.putDouble("hLon", hLon);
            prefs.end();

            Serial.println("[PREFS] Saved home wp");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}