#include "lora.h"

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);

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

void commsTask(void* pvParameters)
{
    if (!LoRaInit()) {Serial.print("LoraInit Err");}

    uint32_t lastFastTele = 0;
    uint32_t lastSlowTele = 0;
    uint32_t lastPacketReceivedTime = 0;

    static Route tempRoute = {}; // Hold route when receiving
    static bool wpReceived[50] = {false};
    static uint8_t receivedCount = 0;

    radio.startReceive();

    for (;;)
    {
        // Read packets
        if (digitalRead(LORA_DIO0))
        {
            uint8_t rxBuffer[256];
            int rxState = radio.readData(rxBuffer, sizeof(rxBuffer));

            if (rxState == RADIOLIB_ERR_NONE && radio.getPacketLength() > 0)
            {
                uint8_t packetID = rxBuffer[0];
                if (packetID == PKT_WP_DATA && radio.getPacketLength() == sizeof(routePacket))
                {
                    lastPacketReceivedTime = millis();
                    routePacket* rp = (routePacket*)rxBuffer;
                    Serial.printf("Received Route Packet %i\n", rp->order);

                    if (rp->order == 0)
                    {
                        tempRoute.id = rp->id;
                        tempRoute.length = rp->ammnt;
                        receivedCount = 0;
                        memset(wpReceived, 0, sizeof(wpReceived));
                    }

                    if (rp->id == tempRoute.id && rp->order < rp->ammnt && rp->order < 50)
                    {
                        routeAckPacket ack;
                        ack.packetID = PKT_ACK;
                        ack.id = rp->id;
                        ack.order = rp->order;

                        vTaskDelay(pdMS_TO_TICKS(50));
                        radio.transmit((uint8_t*)&ack, sizeof(ack));
                        radio.startReceive();

                        if (!wpReceived[rp->order])
                        {
                            tempRoute.waypoints[rp->order].lat = rp->lat;
                            tempRoute.waypoints[rp->order].lon = rp->lon;
                            wpReceived[rp->order] = true;
                            receivedCount++;

                            if (receivedCount == tempRoute.length)
                            {
                                tempRoute.newRouteAvailable = true;

                                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                                {
                                    globalState.route = tempRoute;
                                    xSemaphoreGive(stateMutex);

                                    Serial.println("New route received");
                                }

                                receivedCount = 0;
                            }
                        }
                    }
                }
            }

            radio.startReceive();
        }

        bool isUploading = (millis() - lastPacketReceivedTime < 2000);

        if (!isUploading)
        {
            if (millis() - lastFastTele > 500)
            {
                telemetryFastPacket fastPkt;
                fastPkt.packetID = PKT_TELE_FAST;

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    fastPkt.lat = globalState.gps.lat;
                    fastPkt.lon = globalState.gps.lon;
                    fastPkt.heading = globalState.mag.heading;
                    fastPkt.targetIdx = globalState.targetIdx;
                    fastPkt.mode = globalState.status.mode;

                    xSemaphoreGive(stateMutex);

                    radio.transmit((uint8_t*)&fastPkt, sizeof(fastPkt));
                    //Serial.println("SentFastTelemetry");
                }
                lastFastTele = millis();
                radio.startReceive();
            }

            if (millis() - lastSlowTele > 3100)
            {
                telemetrySlowPacket slowPkt;
                slowPkt.packetID = PKT_TELE_SLOW;

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    slowPkt.batt = globalState.status.battery;
                    slowPkt.gps = (uint8_t)globalState.gps.hdop * 10;
                    slowPkt.commTimeout = globalState.status.commTimeout;
                    slowPkt.errorCode = globalState.status.errorCode;

                    xSemaphoreGive(stateMutex);

                    radio.transmit((uint8_t*)&slowPkt, sizeof(slowPkt));
                }
                lastSlowTele = millis();
                radio.startReceive();
            }
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}
