#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "setup.h"

const char* ssid = "VENE2.0";
const char* password = "123456789";
const char* ip = "192.168.4.1";
const int udpTxPort = 4210;
const int udpRxPort = 4211;

#define LORA_TX_TIMEOUT_MS 750

WiFiUDP udp;
bool udpReady = false;

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);

enum LoRaDir {LORA_DIR_RX, LORA_DIR_TX};
volatile LoRaDir loraDir = LORA_DIR_RX;
static uint32_t txStartTime = 0;

SemaphoreHandle_t rxPacketSem;
SemaphoreHandle_t txDoneSem;

void IRAM_ATTR onLoraDio0Rise()
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
    loraDir = LORA_DIR_RX;
    radio.startReceive();
}

void forceRecoverTxStuck()
{
    Serial.println("[LORA] TX stuck, forcing RX");

    xSemaphoreTake(txDoneSem, 0);
    xSemaphoreTake(rxPacketSem, 0);

    radio.standby();
    loraDir = LORA_DIR_RX;
    radio.startReceive();
    txStartTime = 0;
}

void setup()
{
    Serial.begin(115200);

    rxPacketSem = xSemaphoreCreateBinary();
    txDoneSem = xSemaphoreCreateBinary();

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, -1);
    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);

    Serial.print("[LORA] LoRa init: ");
    Serial.print(state == RADIOLIB_ERR_NONE ? "OK" : "FAIL");
    Serial.println(state);

    if (state == RADIOLIB_ERR_NONE)
    {
        radio.setDio0Action(onLoraDio0Rise, RISING);
        loraDir = LORA_DIR_RX;
        radio.startReceive();
    }
    else
    {
        while (true);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("[REC] Init ready");
}

void loop()
{
    static uint8_t lastWifiStatus = 255;
    uint8_t wifiStatus = WiFi.status();

    if (wifiStatus != lastWifiStatus)
    {
        Serial.print("[WIFI] WiFi status: ");
        Serial.println(wifiStatus);
        lastWifiStatus = wifiStatus;
    }

    if (!udpReady && (wifiStatus == WL_CONNECTED))
    {
        udp.begin(udpRxPort);
        udpReady = true;
        Serial.print("UDP ready, IP: ");
        Serial.println(WiFi.localIP());
    }

    if (xSemaphoreTake(txDoneSem, 0) == pdTRUE)
    {
        completeTransmit();
    }
    else if (loraDir == LORA_DIR_TX && txStartTime != 0 && (millis() - txStartTime) > LORA_TX_TIMEOUT_MS)
    {
        forceRecoverTxStuck();
    }

    if (loraDir == LORA_DIR_RX && xSemaphoreTake(rxPacketSem, 0) == pdTRUE)
    {
        int len = radio.getPacketLength();
        uint8_t rxBuffer[256];
        int state = radio.readData(rxBuffer, len);

        if (state == RADIOLIB_ERR_NONE && len > 0) {Serial.write(rxBuffer, len);}

        if (loraDir == LORA_DIR_RX) {radio.startReceive();}
    }

    if (udpReady)
    {
        int packetSize = udp.parsePacket();

        if (packetSize > 0)
        {
            uint8_t udpData[4];
            udp.read(udpData, sizeof(udpData));

            Serial.write(PKT_WIFI_HEARTBEAT);
            Serial.write(udpData, sizeof(udpData));
        }
    }

    if (Serial.available() > 0)
    {
        uint8_t id = Serial.peek();
        int expectedLen = 0;

        if (id == PKT_WP_DATA) {expectedLen = ROUTE_PACKET_SIZE;}
        else if (id == PKT_CONTROL) {expectedLen = CONTROL_PACKET_SIZE;}
        else if (id == PKT_MANUAL) {expectedLen = MANUAL_SERIAL_SIZE;}
        else if (id == PKT_DATA) {expectedLen = PKT_DATA_SIZE;}
        else if (id == PKT_RESET_ERRORS) {expectedLen = RESET_ERRORS_SIZE;}
        else if (id == PKT_HOME_SET) {expectedLen = HOME_SET_SIZE;}
        else if (id == PKT_HOME_REQ) {expectedLen = HOME_REQ_SIZE;}
        else if (id == PKT_TIME_DATA) {expectedLen = TIME_DATA_SIZE;}
        else if (id == PKT_COURSE_SET) {expectedLen = COURSE_SET_SIZE;}
        else if (id == PKT_UPLOAD_BEGIN) {expectedLen = UPLOAD_BEGIN_SIZE;}
        else if (id == PKT_THR_SET) {expectedLen = THR_SET_SIZE;}
        else
        {
            Serial.read(); // Discard unknown
            return;
        }

        bool isLoraPacket = (id == PKT_WP_DATA || id == PKT_CONTROL || id == PKT_DATA ||
                              id == PKT_RESET_ERRORS || id == PKT_HOME_SET || id == PKT_HOME_REQ ||
                              id == PKT_TIME_DATA || id == PKT_COURSE_SET || id == PKT_THR_SET ||
                              id == PKT_UPLOAD_BEGIN);

        if (isLoraPacket && loraDir == LORA_DIR_TX) {return;} // If already transmiting leave bytes in serial buffer

        if (Serial.available() >= expectedLen)
        {
            uint8_t txBuffer[64];
            Serial.readBytes(txBuffer, expectedLen);

            if (isLoraPacket)
            {
                static uint32_t lastLoraTx = 0;

                if (id == PKT_DATA && (millis() - lastLoraTx) < 400)
                {
                    // Discard too early hb
                }
                else
                {
                    lastLoraTx = millis();
                    beginTransmit(txBuffer, expectedLen);
                }
            }

            else if (id == PKT_MANUAL)
            {
                if (WiFi.status() == WL_CONNECTED)
                {
                    udp.beginPacket(ip, udpTxPort);
                    udp.write(&txBuffer[1], 2);
                    udp.endPacket();
                }
            }
        }
    }
}