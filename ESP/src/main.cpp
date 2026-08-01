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
#include "navigation.h"
#include "errors.h"
#include "autopilot.h"
#include "led.h"

#define WDT_TIMEOUT 5

#define HOME_TRESHOLD 5

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

    uint8_t lastMode = 255;
 
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

        if (mode != lastMode) {resetSteering();} // Reset steering PI controller on mode change 

        bool justArmed = (mode != 0) && !status.ctrlArmed;
 
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
                if (runAutopilot(status, sensors, route)) {mode = 0;}

                break;
 
            case 3: // RETURN HOME
                
                if (distanceToPoint(status.home.lat, status.home.lon, sensors.gps.lat, sensors.gps.lon) > HOME_TRESHOLD)
                {
                    setThrottle(status.APThrottle);
                    steerTo(headingToPoint(sensors.gps.lat, sensors.gps.lon, status.home.lat, status.home.lon));
                }
                else
                {
                    turnRudder(0);
                    setThrottle(0);
                }

                break;

            case 4: // COURSE HOLD
                steerTo(status.targetCourse);
                setThrottle(status.APThrottle);

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

            if (justArmed) {globalState.status.ctrlArmed = true;}
            
            xSemaphoreGive(stateMutex);
        }

        lastMode = mode;
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
void diagTask(void* pv)
{
    uint32_t startMillis = millis();

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

        if (millis() - startMillis > 15000)
        {
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
            Serial.printf("LoRa RSSI: %i dBm\n\n", status.loraRSSI);
            Serial.printf("Errors: 0x%08X\n", (unsigned long)status.errorCode);
            //printActiveErrors();
            Serial.println("----------------");
        }
    }
}

void ledTask(void* pv)
{
    // LED 0: Top
    // LED 1: Bottom

    static LedDoubleFlash bottomBlink;
    static LedPulse bottomCritBlink;
    static LedPulse topSlowBlink;

    static bool dBlink = false;
    static bool critBlinkStarted = false;
    static bool topBlinkStarted = false;
    static bool critErr = false;
    static bool wasCritErr = false;
    static int lastMode = -1;

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        SystemStatus status = {};

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            status = globalState.status;

            xSemaphoreGive(stateMutex);
        }

        if (critErr)
        {
            if (!critBlinkStarted || !wasCritErr)
            {
                bottomCritBlink.start(CRGB::Red, 0, 100, 100); // fast blink, 5 Hz
                critBlinkStarted = true;
            }

            leds[1] = bottomCritBlink.update();
        }
        else
        {
            if (!dBlink || wasCritErr)
            {
                bottomBlink.start(CRGB::White, 100, 150, 1600); 
                dBlink = true;
            }

            leds[1] = bottomBlink.update();
        }

        wasCritErr = critErr;

        if (status.mode != lastMode)
        {
            topBlinkStarted = false; // reset blink phase on mode change
        }

        switch (status.mode)
        {
            case 0: // slow blink
                if (!topBlinkStarted)
                {
                    topSlowBlink.start(CRGB::Red, 0, 1500, 500); // 2s period
                    topBlinkStarted = true;
                }

                leds[0] = topSlowBlink.update();
                break;

            case 1:
                leds[0] = CRGB::Red;
                break;

            case 2:
                leds[0] = CRGB::Green;
                break;

            case 3:
                leds[0] = CRGB::Blue;
                break;

            case 4:
                leds[0] = CRGB::Pink;
                break;

            default:
                leds[0] = CRGB::Black;
                break;
        }

        lastMode = status.mode;

        FastLED.show();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("BOOT...");
    delay(500);

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

    bool sensorInitFail = false;

    if (!magInit()) {Serial.println("[INIT] Mag init fail"); sensorInitFail = true;}
    if (!GPSInit()) {Serial.println("[INIT] GPS init failed"); sensorInitFail = true;}

    if (!sensorInitFail) {Serial.println("[INIT] Sensor init OK");}

    ledSetup();

    controlInit();
    WiFiInit();

    setError(ERR_INIT);

    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    // CORE 1
    xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(controlTask, "Control", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(wifiTask, "WiFi", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(ledTask, "LED", 4096, NULL, 2, NULL, 1);

    // CORE 0
    xTaskCreatePinnedToCore(commsTask, "Comms", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(diagTask, "Diag", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(errorTask, "Error", 4096, NULL, 2, NULL, 0);

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