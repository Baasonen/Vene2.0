#include <Arduino.h>

#include "setup.h"

void loop();
void setup();

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);

void setup()
{
    Serial.begin(115200);

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("SYSTEM_READY");
    }
    else
    {
        Serial.print("LORA_INIT_ERROR: ");
        Serial.println(state);
        while (true);
    }
}

void loop()
{
    uint8_t rxBuffer[256];
    int rxState = radio.receive(rxBuffer, 0);

    if (rxState == RADIOLIB_ERR_NONE)
    {
        int len = radio.getPacketLength();

        Serial.write(rxBuffer, len);
    }
}
