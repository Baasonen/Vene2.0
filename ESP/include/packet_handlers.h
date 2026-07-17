#pragma once

#include "state.h"

void handleRoutePacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime, uint32_t &lastRoutePacketTime, 
                       Route &tempRoute, bool* wpReceived, uint8_t &receivedCount);

void handleControlPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime);

void handleDataPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime);

void handleResetErrorsPacket();

void handleHomeSetPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime);

void handleHomeReqPacket(uint32_t &lastPacketReceivedTime);

void handleTimeDataPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime);

void handleCourseSetPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime);