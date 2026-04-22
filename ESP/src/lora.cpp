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

    for (;;)
    {
        // Read packets

        if (millis() - lastFastTele > 500)
        {
            TelemetryFastPacket fastPkt;
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
                Serial.println("SentFastTelemetry");
            }
            lastFastTele = millis();
        }

        if (millis() - lastSlowTele > 3100)
        {
            TelemetrySlowPacket slowPkt;
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
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}
