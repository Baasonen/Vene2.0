#include "lora.h"

#include "errors.h"
#include "packet_handlers.h"

#define LORA_TIMEOUT_MS 20000

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);
static int8_t lastRSSI = 0;
static uint32_t txStartTime = 0;

#define DUTY_CYCLE_WINDOW_MS 60000UL  // 60s
#define DUTY_CYCLE_LIMIT_PCT 10.0f
static uint32_t dutyWindowStart = 0;
static uint32_t dutyAirtimeMs = 0;

#define SLOW_TELE_PERIOD_MS 5700
#define FAST_TELE_PERIOD_MS 2000
#define BE_TELE_PERIOD_MS 10300

enum LoRaDir {LORA_DIR_RX, LORA_DIR_TX};
static volatile LoRaDir loraDir = LORA_DIR_RX;

SemaphoreHandle_t rxPacketSem;
SemaphoreHandle_t txDoneSem;

void IRAM_ATTR onLoraDIO0Rise()
{
    BaseType_t woken = pdFALSE;

    if (loraDir == LORA_DIR_RX)
    {
        xSemaphoreGiveFromISR(rxPacketSem, &woken);
    }
    else
    {
        xSemaphoreGiveFromISR(txDoneSem, &woken);
    }

    if (woken) {portYIELD_FROM_ISR();}
}

void beginTransmit(uint8_t* data, size_t len)
{
    txStartTime = millis();
    loraDir = LORA_DIR_TX;
    radio.startTransmit(data, len);
}

void completeTransmit()
{
    dutyAirtimeMs += millis() - txStartTime;
    loraDir = LORA_DIR_RX;
    radio.startReceive();
}

void printDutyCycle()
{
    uint32_t elapsed = millis() - dutyWindowStart;
    if (elapsed < DUTY_CYCLE_WINDOW_MS) {return;}

    float dutyPct = (dutyAirtimeMs * 100.0f) / elapsed;

    Serial.printf("[LORA] Duty cycle: %.2f%% of %.0f%% limit (%lu ms TX / %lu ms window)\n",
                  dutyPct, DUTY_CYCLE_LIMIT_PCT, dutyAirtimeMs, elapsed);

    dutyWindowStart = millis();
    dutyAirtimeMs = 0;
}

void resetLoRa()
{
    Serial.println("[LORA] Reseting due to TX stuck");

    detachInterrupt(digitalPinToInterrupt(LORA_DIO0));

    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[LORA] Reinit failed: ");
        Serial.println(state);
    }

    xSemaphoreTake(txDoneSem, 0);
    xSemaphoreTake(rxPacketSem, 0);

    loraDir = LORA_DIR_RX;
    radio.startReceive();

    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);
}

int LoRaInit()
{
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    if (state == RADIOLIB_ERR_NONE) {return 1;}

    else
    {
        Serial.print("[LORA] LoRa Init Error");
        Serial.println(state);
        return 0;
    }
}

void rxTask(uint32_t &lastPacketReceivedTime, uint32_t &lastRoutePacketTime,
            Route &tempRoute, bool* wpReceived, uint8_t &receivedCount)
{
    int len = radio.getPacketLength();
    if (len == 0) {return;}

    uint8_t rxBuffer[256];
    int rxState = radio.readData(rxBuffer, sizeof(rxBuffer));
    if (rxState != RADIOLIB_ERR_NONE) {return;}

    lastRSSI = (int8_t)constrain((int)radio.getRSSI(), INT8_MIN, INT8_MAX);

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
    {
        globalState.status.loraRSSI = lastRSSI;
        xSemaphoreGive(stateMutex);
    }

    uint8_t packetID = rxBuffer[0];

    switch (packetID)
    {
        case PKT_WP_DATA:
            if (len == sizeof(routePacket)) 
            {
                handleRoutePacket(rxBuffer, lastPacketReceivedTime, lastRoutePacketTime, tempRoute, wpReceived, receivedCount);
            }
            break;

        case PKT_CONTROL:
            if (len == sizeof(controlPacket)) 
            {
                handleControlPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_DATA:
            if (len == sizeof(dataPacket)) 
            {
                handleDataPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_RESET_ERRORS:
            if (len == sizeof(resetErrorsPacket)) 
            {
                handleResetErrorsPacket();
            }
            break;

        case PKT_HOME_SET:
            if (len == sizeof(homeSetPacket)) 
            {
                handleHomeSetPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_HOME_REQ:
            if (len == 1) 
            {
                handleHomeReqPacket(lastPacketReceivedTime);
            }
            break;

        case PKT_TIME_DATA:
            if (len == sizeof(timeDataPacket)) 
            {
                handleTimeDataPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_COURSE_SET:
            if (len == sizeof(courseSetPacket)) 
            {
                handleCourseSetPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_THR_SET:
            if (len == sizeof(throttlePacket))
            {
                handleThrottleSetPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_UPLOAD_BEGIN:
            if (len == 1) {
                lastRoutePacketTime = millis();
            }
            break;
    }
}

void txTask(uint32_t &lastFastTele, uint32_t &lastSlowTele, uint32_t &lastBETele)
{   
    static uint32_t lastTimeReq = 0;
    static uint32_t lastErrorCode = 0;

    if (!globalState.status.timeSet && (millis() - lastTimeReq) > 6500)
    {
        Serial.println("[TIME] Requesting current time");
        uint8_t timeReqPacket = PKT_TIME_REQ;
        beginTransmit(&timeReqPacket, 1);
        lastTimeReq = millis();
        return;
    }
    
    if (millis() - lastFastTele > FAST_TELE_PERIOD_MS)
    {
        telemetryFastPacket fastPkt = {};
        fastPkt.packetID = PKT_TELE_FAST;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            fastPkt.heading = (uint16_t)(globalState.sensors.mag.heading * 100);
            fastPkt.targetIdx = globalState.status.targetWaypoint;
            fastPkt.mode = globalState.status.mode;
            
            xSemaphoreGive(stateMutex);

            beginTransmit((uint8_t*)&fastPkt, sizeof(fastPkt));
            lastFastTele = millis();

            return;
        }
    }

    if (millis() - lastSlowTele > SLOW_TELE_PERIOD_MS)
    {   
        telemetrySlowPacket slowPkt = {};
        slowPkt.packetID = PKT_TELE_SLOW;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            slowPkt.batt = globalState.status.battery;
            slowPkt.gps = (uint8_t)(globalState.sensors.gps.hdop * 10);
            slowPkt.signalStrength = (uint8_t)(lastRSSI + 128);
            slowPkt.lat = (int32_t)(globalState.sensors.gps.lat * 1e7);
            slowPkt.lon = (int32_t)(globalState.sensors.gps.lon * 1e7);

            xSemaphoreGive(stateMutex);

            beginTransmit((uint8_t*)&slowPkt, sizeof(slowPkt));
            lastSlowTele = millis();
            //Serial.println("[LORA] Slow tele sent");

            return;
        }
    }

    bool sendBE = false;
    beTelemetryPacket bePkt = {};
    bePkt.packetID = PKT_BE_TELE;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        bePkt.errorCode = globalState.status.errorCode;
        bePkt.c1 = (uint8_t)100;

        xSemaphoreGive(stateMutex);

        sendBE = ((millis() - lastBETele) > BE_TELE_PERIOD_MS) || (bePkt.errorCode != lastErrorCode);

        if (sendBE)
        {
            beginTransmit((uint8_t*)&bePkt, sizeof(beTelemetryPacket));

            lastBETele = millis();
            lastErrorCode = bePkt.errorCode;

            return;
        }
    }
}

void commsTask(void* pvParameters)
{
    rxPacketSem = xSemaphoreCreateBinary();
    txDoneSem = xSemaphoreCreateBinary();
    esp_task_wdt_add(NULL);

    uint32_t lastRoutePacketTime = 0;

    while (!LoRaInit())
    {
        Serial.println("[LORA] Init failed, retrying in 5s...");

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.loraTimeout = true;
            xSemaphoreGive(stateMutex);
        }

        for (int i = 0; i < 50; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_task_wdt_reset();
        }
    }

    
    radio.startReceive();
    pinMode(LORA_DIO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);

    uint32_t lastFastTele = 0;
    uint32_t lastSlowTele = 0;
    uint32_t lastBETele = 0;
    uint32_t lastPacketReceivedTime = millis();

    static Route tempRoute = {};
    static bool wpReceived[WP_AMMNT_LIM] = {false};
    static uint8_t receivedCount = 0;

    dutyWindowStart = millis();

    for (;;)
    {
        esp_task_wdt_reset();

        // Handle TX completion (hig priority)
        if (xSemaphoreTake(txDoneSem, 0) == pdTRUE)
        {
            completeTransmit();
        }
        else if (loraDir == LORA_DIR_TX && txStartTime != 0 && (millis() - txStartTime) > LORA_TX_TIMEOUT_MS)
        {
            Serial.println("[LORA] TX timeout, forcing RX");
            resetLoRa();
            txStartTime = 0;
        }

        // Handle RX (only when not transmitting)
        if (loraDir == LORA_DIR_RX)
        {
            if (xSemaphoreTake(rxPacketSem, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                rxTask(lastPacketReceivedTime, lastRoutePacketTime,
                        tempRoute, wpReceived, receivedCount);
                
                if (loraDir == LORA_DIR_RX) {radio.startReceive();}
            }
        }
        else // Yield to TX
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        bool isUploading = ((millis() - lastRoutePacketTime) < 1500);

        if (!isUploading && loraDir == LORA_DIR_RX && uxSemaphoreGetCount(rxPacketSem) == 0)
        {
            txTask(lastFastTele, lastSlowTele, lastBETele);
        }

        printDutyCycle();

        if ((millis() - lastPacketReceivedTime) > LORA_TIMEOUT_MS)
        {
            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                if (!globalState.status.loraTimeout)
                {
                    globalState.status.commTimeoutTriggerTime = millis();
                    globalState.status.loraTimeout = true;
                }

                xSemaphoreGive(stateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
