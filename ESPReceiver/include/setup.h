#pragma once

#include <RadioLib.h>
#include <SPI.h>

#define LORA_SCK 14
#define LORA_MISO 12
#define LORA_MOSI 13
#define LORA_CS 17
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

#define PKT_RESET_ERRORS 0x07
#define RESET_ERRORS_SIZE 1

#define PKT_WIFI_HEARTBEAT 0x06

#define PKT_HOME_SET 0x08
#define HOME_SET_SIZE 17

#define PKT_HOME_REQ 0x0A
#define HOME_REQ_SIZE 1

#define PKT_TIME_DATA 0x0C
#define TIME_DATA_SIZE 5

#define PKT_HOME_DATA 0x09
#define HOME_DATA_SIZE 17

#define PKT_TIME_REQ 0x0B
#define TIME_REQ_SIZE 1