#pragma once

#include <RadioLib.h>
#include <SPI.h>

#include "esp_system.h"
#include "esp_task_wdt.h"

#include "state.h"

#define LORA_SCK 14
#define LORA_MISO 26
#define LORA_MOSI 13
#define LORA_CS 15
#define LORA_DIO0 12

#define LORA_FREQ 868.5
#define LORA_BANDWIDTH 125.0
#define LORA_SF 9
#define LORA_CODING_RATE 7
#define LORA_POWER 15

#define PKT_WP_DATA 0x01
#define PKT_TELE_FAST 0x02
#define PKT_TELE_SLOW 0x03
#define PKT_CONTROL 0x04
#define PKT_RESET_ERRORS 0x07
#define PKT_DATA 0x10

#define PKT_HOME_SET 0x08
#define PKT_HOME_DATA 0x09
#define PKT_HOME_REQ 0x0A
#define PKT_TIME_REQ 0x0B
#define PKT_TIME_DATA 0x0C

#pragma pack(push, 1)

struct telemetryFastPacket
{
    uint8_t packetID; // 0x02
    double lat;
    double lon;
    float heading;
    uint8_t mode;
    uint8_t targetIdx; 
};

struct telemetrySlowPacket
{
    uint8_t packetID; // 0x03
    uint8_t batt;
    uint8_t gps;
    uint8_t signalStrength;
    uint32_t errorCode;
};

struct routePacket
{
    uint8_t packetID; // 0x01
    uint8_t id;
    double lat;
    double lon;
    uint8_t order;
    uint8_t ammnt;
};

struct resetErrorsPacket
{
    uint8_t packetID; // 0x07
};

struct dataPacket
{
    uint8_t packetID; // 0x10
    uint8_t id;
    uint8_t data;
};

struct controlPacket
{
    uint8_t packetID;
    uint8_t mode;
};

struct homeSetPacket // From GUI: Set homeWP
{
    uint8_t packedID; // 0x08
    double lat;
    double lon;
};

struct homeDataPacket // Respond to PKT_HOME_REQ from GUI
{
    uint8_t packetID; // 0x09
    double lat;
    double lon;
};

struct timeDataPacket
{
    uint8_t packetID; // 0x0C
    uint32_t unixTime;
};

#pragma pack(pop)

extern SemaphoreHandle_t stateMutex;

int LoRaInit();
void commsTask(void* pvParameters);