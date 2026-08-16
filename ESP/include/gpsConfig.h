#pragma once

#include <Arduino.h>

void sendUBX(Stream &port, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
void gpsRunOneTimeSetup(Stream &port);
void ubxEnableSBAS(Stream &port);
void ubxEnableNavMessages(Stream &port);
void ubxDisableNMEA(Stream &port);
void ubxSaveConfig(Stream &port);
void ubxSetNavRate(Stream &port, uint16_t measRateMs);
void ubxSetBaudRate(Stream &port, uint32_t baud);

// Manual reset
void ubxClearConfig(Stream &port);
void ubxColdReset(Stream &port);