#pragma once

#include <WiFi.h>
#include <WiFiUdp.h> 

#include "state.h"

#define WIFI_RX_PORT 4210
#define WIFI_TX_PORT 4211

#define WIFI_NEW_CONNECTION_TIMEOUT 5000

typedef struct 
{
    uint32_t data;
} WifiHeartbeat;

extern WiFiUDP udp;

void WiFiInit();
void WiFiReadIncoming(SemaphoreHandle_t stateMutex);