#pragma once

#include <WiFi.h>
#include <WiFiUdp.h> 

#include "state.h"

#define WIFI_RX_PORT 4210

extern WiFiUDP udp;

void WiFiInit();
void WiFiReadIncoming(SemaphoreHandle_t stateMutex);