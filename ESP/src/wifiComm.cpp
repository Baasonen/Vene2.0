#include "wifiComm.h"

#include <WiFi.h>

const char* ssid = "VENE2.0";
const char* password = "123456789";

WiFiUDP udp;

void WiFiInit()
{
    WiFi.softAP(ssid, password);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    udp.begin(WIFI_RX_PORT);

    Serial.print("Wifi AP started on: ");
    Serial.println(WiFi.softAPIP());
}

void WiFiReadIncoming(SemaphoreHandle_t stateMutex)
{
    int32_t packetSize = udp.parsePacket();
    static uint32_t lastWifiTime = 0;
    bool hasNewData = false;
    ManualControls inbound = {};

    while ((packetSize = udp.parsePacket()) > 0)
    {
        if (packetSize == sizeof(ManualControls))
        {
            udp.read((uint8_t*)&inbound, sizeof(ManualControls));
            hasNewData = true;
        }
        else
        {
            udp.flush();
        }
    }

    if (hasNewData)
    {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            globalState.status.wifiTimeout = false;
            globalState.manual = inbound;

            lastWifiTime = millis();
            xSemaphoreGive(stateMutex);
        }
    }

    if (millis() - lastWifiTime > 2000)
    {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            globalState.status.wifiTimeout = true;

            xSemaphoreGive(stateMutex);
        }
    }
}