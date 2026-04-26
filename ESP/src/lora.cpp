#include "lora.h"

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);
SemaphoreHandle_t packetSem;

void IRAM_ATTR onLoraDIO0Rise()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(packetSem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {portYIELD_FROM_ISR();}
}

int LoRaInit()
{
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    if (state == RADIOLIB_ERR_NONE) {return 1;}

    else
    {
        Serial.print("Lora error");
        Serial.println(state);
        return 0;
    }
}

void rxTask(uint32_t &lastPacketReceivedTime, Route &tempRoute, bool* wpReceived, uint8_t &receivedCount)
{
    int len = radio.getPacketLength();
    if (len == 0) {return;}

    uint8_t rxBuffer[256];
    int rxState = radio.readData(rxBuffer, sizeof(rxBuffer));
    if ((rxState != RADIOLIB_ERR_NONE) || (radio.getPacketLength() == 0)) {return;}

    uint8_t packetID = rxBuffer[0];

    // Route Packet
    if ((packetID == PKT_WP_DATA) && (len == sizeof(routePacket)))
    {
        lastPacketReceivedTime = millis();

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
            radio.transmit((uint8_t*)&ack, sizeof(ack));
            radio.startReceive();

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
    else if ((packetID == PKT_CONTROL) && (radio.getPacketLength() == sizeof(controlPacket)))
    {
        lastPacketReceivedTime = millis();

        controlPacket* cp = (controlPacket*)rxBuffer;

        dataPacket ack = {PKT_DATA, 255, cp->mode};
        radio.transmit((uint8_t*)&ack, sizeof(ack));

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.mode = cp->mode;
            globalState.status.loraTimeout = false;
            xSemaphoreGive(stateMutex);
        }

        radio.startReceive();
    }

    else if ((packetID == PKT_DATA) && (radio.getPacketLength() == sizeof(dataPacket)))
    {
        dataPacket* hb = (dataPacket*)rxBuffer;
        if (hb->id == 254) 
        {
            lastPacketReceivedTime = millis();

            if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)))
            {
                globalState.status.loraTimeout = false;
                xSemaphoreGive(stateMutex);
            }
        }
    }
}

void txTask(uint32_t &lastFastTele, uint32_t &lastSlowTele)
{
    if (millis() - lastFastTele > 900)
    {
        telemetryFastPacket fastPkt;
        fastPkt.packetID = PKT_TELE_FAST;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            fastPkt.lat = globalState.gps.lat;
            fastPkt.lon = globalState.gps.lon;
            fastPkt.heading = globalState.mag.heading;
            fastPkt.targetIdx = globalState.status.targetIdx;
            fastPkt.mode = globalState.status.mode;
            
            xSemaphoreGive(stateMutex);

            radio.transmit((uint8_t*)&fastPkt, sizeof(fastPkt));
            radio.startReceive();
        }
        lastFastTele = millis();
    }

    if (millis() - lastSlowTele > 3100)
    {   
        telemetrySlowPacket slowPkt;
        slowPkt.packetID = PKT_TELE_SLOW;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            slowPkt.batt = globalState.status.battery;
            slowPkt.gps = (uint8_t)(globalState.gps.hdop * 10);
            slowPkt.errorCode = globalState.status.errorCode;

            xSemaphoreGive(stateMutex);

            radio.transmit((uint8_t*)&slowPkt, sizeof(slowPkt));
            radio.startReceive();
        }
        lastSlowTele = millis();
    }
}

void commsTask(void* pvParameters)
{
    packetSem = xSemaphoreCreateBinary();
    esp_task_wdt_add(NULL);

    if (!LoRaInit()) {Serial.println("LoraInit ERR");}

    pinMode(LORA_DIO0, INPUT);
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDIO0Rise, RISING);

    uint32_t lastFastTele = 0;
    uint32_t lastSlowTele = 0;
    uint32_t lastPacketReceivedTime = millis();

    static Route tempRoute = {};
    static bool wpReceived[50] = {false};
    static uint8_t receivedCount = 0;

    radio.startReceive();

    for (;;)
    {
        esp_task_wdt_reset();

        if (xSemaphoreTake(packetSem, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            rxTask(lastPacketReceivedTime, tempRoute, wpReceived, receivedCount);
            radio.startReceive();
        }

        bool isUploading = ((millis() - lastPacketReceivedTime) < 1500);
        if (!isUploading)
        {
            if (uxSemaphoreGetCount(packetSem) == 0) {txTask(lastFastTele, lastSlowTele);}
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
    }
}
