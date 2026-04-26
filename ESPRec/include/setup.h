#pragma once

#include <RadioLib.h>
#include <SPI.h>

#define LORA_SCK 14
#define LORA_MISO 12
#define LORA_MOSI 13
#define LORA_CS 15
#define LORA_DIO0 26

#define LORA_FREQ 868.5
#define LORA_BANDWIDTH 125.0
#define LORA_SF 9
#define LORA_CODING_RATE 7
#define LORA_POWER 15

#define PKT_WP_DATA 0x01
#define ROUTE_PACKET_SIZE 20

#define PKT_CONTROL 0x04
#define CONTROL_PACKET_SIZE 2

#define PKT_MANUAL 0x05
#define MANUAL_SERIAL_SIZE 3 

#define PKT_DATA 0x10
#define PKT_DATA_SIZE 3

#define PKT_WIFI_HEARTBEAT 0x06