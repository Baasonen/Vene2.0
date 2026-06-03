#include "lora.h"

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);
static int8_t lastRSSI = 0;

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
    loraDir = LORA_DIR_TX;
    radio.startTransmit(data, len);
}

void completeTransmit()
{
    loraDir = LORA_DIR_RX;
    radio.startReceive();
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

    uint8_t packetID = rxBuffer[0];

    // Route Packet
    if ((packetID == PKT_WP_DATA) && (len == sizeof(routePacket)))
    {
        lastPacketReceivedTime = millis();
        lastRoutePacketTime = millis();

        routePacket* rp = (routePacket*)rxBuffer;
        Serial.printf("Received Route Packet %i\n", rp->order);

        if (rp->order == 0)
        {
            tempRoute.id = rp->id;
            tempRoute.length = rp->ammnt;
            receivedCount = 0;
            memset(wpReceived, 0, 50 * sizeof(bool));
        }

        if ((rp->id == tempRoute.id) && (rp->order < 50))
        {
            dataPacket ack = {PKT_DATA, rp->id, rp->order};
            beginTransmit((uint8_t*)&ack, sizeof(ack));

            if (!wpReceived[rp->order])
            {
                tempRoute.waypoints[rp->order + 1].lat = rp->lat;
                tempRoute.waypoints[rp->order + 1].lon = rp->lon;
                wpReceived[rp->order] = true;
                receivedCount++;

                if (receivedCount == tempRoute.length)
                {
                    tempRoute.newRouteAvailable = true;

                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                    {
                        globalState.status.loraTimeout = false;
                        globalState.status.routeReady = true;

                        tempRoute.waypoints[0].lat = globalState.status.home.lat;
                        tempRoute.waypoints[0].lon = globalState.status.home.lon;
                        globalState.route = tempRoute;

                        xSemaphoreGive(stateMutex);
                    }
                }
            }
        }
    }

    else if ((packetID == PKT_CONTROL) && (len == sizeof(controlPacket)))
    {
        lastPacketReceivedTime = millis();

        controlPacket* cp = (controlPacket*)rxBuffer;

        dataPacket ack = {PKT_DATA, 255, cp->mode};
        beginTransmit((uint8_t*)&ack, sizeof(ack));

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.mode = cp->mode;
            globalState.status.loraTimeout = false;
            xSemaphoreGive(stateMutex);
        }
    }

    else if ((packetID == PKT_DATA) && (len == sizeof(dataPacket)))
    {
        dataPacket* hb = (dataPacket*)rxBuffer;
        if (hb->id == 254) 
        {
            lastPacketReceivedTime = millis();

            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                globalState.status.loraTimeout = false;
                xSemaphoreGive(stateMutex);
            }
        }
    }

    else if ((packetID == PKT_RESET_ERRORS) && (len == sizeof(resetErrorsPacket)))
    {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.errorCode = 0;
            globalState.status.loraTimeout = false;
            xSemaphoreGive(stateMutex);
        }

        dataPacket ack = {PKT_DATA, 254, PKT_RESET_ERRORS};
        beginTransmit((uint8_t*)&ack, sizeof(ack));
    }
}

void txTask(uint32_t &lastFastTele, uint32_t &lastSlowTele)
{
    if (millis() - lastFastTele > 900)
    {
        telemetryFastPacket fastPkt = {};
        fastPkt.packetID = PKT_TELE_FAST;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            fastPkt.lat = globalState.sensors.gps.lat;
            fastPkt.lon = globalState.sensors.gps.lon;
            fastPkt.heading = globalState.sensors.mag.heading;
            fastPkt.targetIdx = globalState.status.targetIdx;
            fastPkt.mode = globalState.status.mode;
            
            xSemaphoreGive(stateMutex);

            beginTransmit((uint8_t*)&fastPkt, sizeof(fastPkt));
            lastFastTele = millis();

            return;
        }
    }

    if (millis() - lastSlowTele > 3100)
    {   
        telemetrySlowPacket slowPkt = {};
        slowPkt.packetID = PKT_TELE_SLOW;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            slowPkt.batt = globalState.status.battery;
            slowPkt.gps = (uint8_t)(globalState.sensors.gps.hdop * 10);
            slowPkt.errorCode = globalState.status.errorCode;
            slowPkt.signalStrength = (uint8_t)(lastRSSI + 128);

            xSemaphoreGive(stateMutex);

            beginTransmit((uint8_t*)&slowPkt, sizeof(slowPkt));
            lastSlowTele = millis();
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
    uint32_t lastPacketReceivedTime = millis();

    static Route tempRoute = {};
    static bool wpReceived[50] = {false};
    static uint8_t receivedCount = 0;

    for (;;)
    {
        esp_task_wdt_reset();

        // Handle TX completion (hig priority)
        if (xSemaphoreTake(txDoneSem, 0) == pdTRUE)
        {
            completeTransmit();
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
            txTask(lastFastTele, lastSlowTele);
        }

        if ((millis() - lastPacketReceivedTime) > 10000)
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
