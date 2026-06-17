#include "errors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

#define ERROR_TASK_PERIOD_MS 200
#define DEBOUNCE_THRESHOLD 5
#define BATT_LOW_TRESHOLD 20

struct Debounce
{
    uint8_t counter = 0;
};

static bool debounce(Debounce& d, bool condition, uint8_t treshold = DEBOUNCE_THRESHOLD)
{
    if (condition)
    {
        if (d.counter < treshold) {d.counter++;}
    }
    else
    {
        if (d.counter > 0) {d.counter--;}
    }
    return d.counter >= treshold;
}

void errorTask(void* pv)
{
    esp_task_wdt_add(NULL);

    static Debounce gpsFailDb, gpsAccDb, magFailDb, magAccDb, battDb;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(ERROR_TASK_PERIOD_MS);

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        esp_task_wdt_reset();

        SensorData sensors = {};
        SystemStatus status = {};
        Route route = {};

        xQueuePeek(sensorQueue, &sensors, 0);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            status = globalState.status;
            route = globalState.route;
            xSemaphoreGive(stateMutex);
        }

        setErrorIf(ERR_GPS_FAIL, debounce(gpsFailDb, !sensors.gps.valid));
        setErrorIf(ERR_MAG_FAIL, debounce(magFailDb, !sensors.mag.valid));
        setErrorIf(ERR_GPS_ACC_LOW, debounce(gpsAccDb, sensors.gps.hdop > GPS_MAX_HDOP));
        setErrorIf(ERR_MAG_ACC_LOW, debounce(magAccDb, sensors.mag.accuracy < MAG_MIN_ACC));
        setErrorIf(ERR_BATT_LOW, debounce(battDb, status.battery < BATT_LOW_TRESHOLD));

        setErrorIf(ERR_LORA_TIMEOUT, status.loraTimeout);
        setErrorIf(ERR_WIFI_TIMEOUT, status.wifiTimeout);

        setErrorIf(ERR_NO_ROUTE, route.length == 0);
        setErrorIf(ERR_NO_HOME, !status.homeSet);

        clearError(ERR_HDG_A1);        
        clearError(ERR_HDG_A2);
        clearError(ERR_HDG_A3);
        switch(sensors.mag.accuracy)
        {
            case 1: setError(ERR_HDG_A1); break;
            case 2: setError(ERR_HDG_A2); break;
            case 3: setError(ERR_HDG_A3); break;
            default: break;
        }
    }
}