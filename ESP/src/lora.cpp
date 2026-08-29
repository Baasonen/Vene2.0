#include "lora.h"

#include "errors.h"
#include "packet_handlers.h"

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, -1);
static int8_t lastRSSI = 0;
static uint32_t txStartTime = 0;

static uint32_t dutyWindowStart = 0;
static uint32_t dutyAirtimeMs = 0;

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

static bool channelClear()
{
    detachInterrupt(digitalPinToInterrupt(LORA_DIO0));

    xSemaphoreTake(rxPacketSem, 0);
    xSemaphoreTake(txDoneSem, 0);

    int16_t cad = radio.scanChannel();

    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);

    return (cad == RADIOLIB_CHANNEL_FREE);
}

void beginTransmit(uint8_t* data, size_t len)
{
    for (int attempt = 0; attempt < CAD_MAX_ATTEMPTS; attempt++)
    {
        if (channelClear()) {break;}

        loraDir = LORA_DIR_RX;
        radio.startReceive();

        uint32_t backoff = CAD_BACKOFF_MIN_MS + (esp_random() % CAD_BACKOFF_MAX_MS);
        vTaskDelay(pdMS_TO_TICKS(backoff));
    }

    radio.standby();
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

    SPI.end(); // Fix LoRa unable to recoved due to broken SPI state
    delay(100);
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);

    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    xSemaphoreTake(txDoneSem, 0);
    xSemaphoreTake(rxPacketSem, 0);
    loraDir = LORA_DIR_RX;

    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("[LORA] Reinit failed: ");
        Serial.println(state);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (!globalState.status.loraTimeout)
            {
                globalState.status.loraTimeoutTriggerTime = millis();
            }
            globalState.status.loraTimeout = true;
            xSemaphoreGive(stateMutex);
        }
        // Leave interrupt detached to prevent garbage data reseting timeout
        return;
    }

    pinMode(LORA_DIO0, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);
    radio.startReceive();

    // If DIO0 already high RISING won't trigger
    if (digitalRead(LORA_DIO0) == HIGH) {xSemaphoreGive(rxPacketSem);}
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
            if (len == sizeof(RoutePacket)) 
            {
                handleRoutePacket(rxBuffer, lastPacketReceivedTime, lastRoutePacketTime, tempRoute, wpReceived, receivedCount);
            }
            break;

        case PKT_CONTROL:
            if (len == sizeof(ControlPacket)) 
            {
                handleControlPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_DATA:
            if (len == sizeof(DataPacket)) 
            {
                handleDataPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_RESET_ERRORS:
            if (len == sizeof(ResetErrorsPacket)) 
            {
                handleResetErrorsPacket();
            }
            break;

        case PKT_HOME_SET:
            if (len == sizeof(HomeSetPacket)) 
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
            if (len == sizeof(TimeDataPacket)) 
            {
                handleTimeDataPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_COURSE_SET:
            if (len == sizeof(CourseSetPacket)) 
            {
                handleCourseSetPacket(rxBuffer, lastPacketReceivedTime);
            }
            break;

        case PKT_THR_SET:
            if (len == sizeof(ThrottlePacket))
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
        TelemetryFastPacket fastPkt = {};
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
        TelemetrySlowPacket slowPkt = {};
        slowPkt.packetID = PKT_TELE_SLOW;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            uint8_t kfStatus;

            if (!globalState.sensors.kf.valid || isnan(globalState.sensors.kf.posStdM)) {kfStatus = 255;}
            else
            {
                uint32_t scaled = (uint32_t)(globalState.sensors.kf.posStdM * 10.0f);
                kfStatus = (uint8_t)min(scaled, (uint32_t)100);
            }
            
            slowPkt.gps = kfStatus;
            slowPkt.signalStrength = (uint8_t)(lastRSSI + 128);
            slowPkt.lat = (int32_t)(globalState.sensors.kf.lat * 1e7);
            slowPkt.lon = (int32_t)(globalState.sensors.kf.lon * 1e7);

            xSemaphoreGive(stateMutex);
            beginTransmit((uint8_t*)&slowPkt, sizeof(slowPkt));
            lastSlowTele = millis();
            //Serial.println("[LORA] Slow tele sent");

            return;
        }
    }

    bool sendBE = false;
    BeTelemetryPacket bePkt = {};
    bePkt.packetID = PKT_BE_TELE;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        bePkt.errorCode = globalState.status.errorCode;
        bePkt.c1 = globalState.battery.c1;
        bePkt.c2 = globalState.battery.c2;
        bePkt.c3 = globalState.battery.c3;

        xSemaphoreGive(stateMutex);

        sendBE = ((millis() - lastBETele) > BE_TELE_PERIOD_MS) || (bePkt.errorCode != lastErrorCode);

        if (sendBE)
        {
            beginTransmit((uint8_t*)&bePkt, sizeof(BeTelemetryPacket));

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
            if (!globalState.status.loraTimeout)
            {
                globalState.status.loraTimeoutTriggerTime = millis();
            }

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
    pinMode(LORA_DIO0, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);

    uint32_t lastFastTele = 0;
    uint32_t lastSlowTele = 0;
    uint32_t lastBETele = 0;
    uint32_t lastPacketReceivedTime = millis();

    static Route tempRoute = {};
    static bool wpReceived[WP_AMMOUNT_LIM] = {false};
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
                    globalState.status.loraTimeoutTriggerTime = millis();
                    globalState.status.loraTimeout = true;
                }

                xSemaphoreGive(stateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
