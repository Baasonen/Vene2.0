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
    static uint32_t lastWifiTime = 0;
    static uint32_t lastWifiHeartbeat = 0;
    static IPAddress lastIP;
    
    static uint32_t connectionStartTime = 0;
    static bool stationWasConnected = false;

    bool hasNewData = false;
    ManualControls inbound = {};
    int32_t packetSize = 0;

    int stationCount = WiFi.softAPgetStationNum();
    if ((stationCount > 0) && !stationWasConnected)
    {
        connectionStartTime = millis();
        stationWasConnected = true;
    }
    else if (stationCount == 0)
    {
        connectionStartTime = 0;
        stationWasConnected = false;
    }

    while ((packetSize = udp.parsePacket()) > 0)
    {
        if (packetSize == sizeof(ManualControls))
        {
            udp.read((uint8_t*)&inbound, sizeof(ManualControls));
            hasNewData = true;
            lastIP = udp.remoteIP();
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

    bool isNewConnectionTimeoutOver = ((millis() - connectionStartTime) > WIFI_NEW_CONNECTION_TIMEOUT);

    if ((lastIP != IPAddress()) && (WiFi.softAPgetStationNum() > 0) && (millis() - lastWifiHeartbeat > 1000) && isNewConnectionTimeoutOver)
    {
        WifiHeartbeat data = {0};
        data.data = lastWifiTime;

        udp.beginPacket(lastIP, WIFI_TX_PORT);
        udp.write((uint8_t*)&data, sizeof(WifiHeartbeat));
        udp.endPacket();

        lastWifiHeartbeat = millis();
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