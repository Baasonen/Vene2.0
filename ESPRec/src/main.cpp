#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "setup.h"

const char* ssid = "VENE2.0";             
const char* password = "123456789"; 
const char* ip = "192.168.4.1";    
const int udpTxPort = 4210;
const int udpRxPort = 4211;

WiFiUDP udp;
bool udpReady = false;

SX1276 radio = new Module(LORA_CS, LORA_DIO0, -1, -1);

volatile bool receivedFlag = false;

void IRAM_ATTR setFlag() {
    receivedFlag = true;
}

void setup()
{
    Serial.begin(115200);

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    int state = radio.begin(LORA_FREQ, LORA_BANDWIDTH, LORA_SF, LORA_CODING_RATE, RADIOLIB_SX127X_SYNC_WORD, LORA_POWER);
    
    Serial.print("LoRa init: ");
    Serial.println(state == RADIOLIB_ERR_NONE ? "OK" : "FAIL");
    Serial.print("State code: ");
    Serial.println(state);

    if (state == RADIOLIB_ERR_NONE) {
        radio.setDio0Action(setFlag, RISING); 
        radio.startReceive();
    } else {
        while (true);
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("SYSTEM_READY");
}

void loop()
{
    static uint8_t lastWifiStatus = 255;
    uint8_t wifiStatus = WiFi.status();
    
    if (wifiStatus != lastWifiStatus) 
    { 
        Serial.print("WiFi status: ");
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

    if (receivedFlag)
    {
        receivedFlag = false;
        int len = radio.getPacketLength();
        uint8_t rxBuffer[256];
        int state = radio.readData(rxBuffer, len);

        if (state == RADIOLIB_ERR_NONE && len > 0)
        {
            Serial.write(rxBuffer, len);
        }
        radio.startReceive();
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
        int expected_len = 0;

        if (id == PKT_WP_DATA) {expected_len = ROUTE_PACKET_SIZE;} 
        else if (id == PKT_CONTROL) {expected_len = CONTROL_PACKET_SIZE;} 
        else if (id == PKT_MANUAL) {expected_len = MANUAL_SERIAL_SIZE;}
        else if (id == PKT_DATA) {expected_len = PKT_DATA_SIZE;}

        else 
        {
            Serial.read(); 
            return;
        }

        if (Serial.available() >= expected_len)
        {
            uint8_t txBuffer[64];
            Serial.readBytes(txBuffer, expected_len);

            if (id == PKT_WP_DATA || id == PKT_CONTROL || id == PKT_DATA) 
            {
                static uint32_t lastLoRaTx = 0;

                if (id == PKT_DATA && (millis() - lastLoRaTx) < 400)
                {
                    // Discard HB
                }
                else 
                {
                    receivedFlag = false;
                    lastLoRaTx = millis();
                    radio.transmit(txBuffer, expected_len);
                    radio.startReceive();
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