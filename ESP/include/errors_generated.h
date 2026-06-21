#pragma once

#include <stdint.h>

// =========================================================
// AUTO-GENERATED FILE -- DO NOT EDIT BY HAND
// Source: errors.def
// =========================================================

enum ErrorBit : uint8_t
{
    ERR_INIT          = 0,  // Init successful
    ERR_GPS_FAIL      = 1,  // No valid gps fix
    ERR_MAG_FAIL      = 2,  // Cannot connect to the magnetometer
    ERR_GPS_ACC_LOW   = 3,  // Gps accuracy low
    ERR_MAG_ACC_LOW   = 4,  // Magnetometer accuracy low
    ERR_BATT_LOW      = 5,  // Battery voltage low
    ERR_LORA_TIMEOUT  = 6,
    ERR_WIFI_TIMEOUT  = 7,
    ERR_MOTOR_FAULT   = 8,  // Motor / ESC fault
    ERR_NO_ROUTE      = 9,  // No route loaded, cannot enable A/P
    ERR_NO_HOME       = 10,  // Home WP not set
    ERR_WATER_LOW     = 11,  // Water ingress sensor detected LOW water level
    ERR_WATER_MID     = 12,  // Water ingress sensor detected MEDIUM water level
    ERR_WATER_HIGH    = 13,  // Water ingress sensor detected HIGH water level
    ERR_HDG_A1        = 14,  // Heading accuracy 1
    ERR_HDG_A2        = 15,  // Heading accuracy 2
    ERR_HDG_A3        = 16,  // Heading accuracy 3
};

#define ERROR_BIT_COUNT 17

struct ErrorBitInfo
{
    uint8_t bit;
    const char* name;
    const char* description;
};

static const ErrorBitInfo ERROR_BIT_TABLE[ERROR_BIT_COUNT] = {
    { 0, "INIT", "Init successful" },
    { 1, "GPS_FAIL", "No valid gps fix" },
    { 2, "MAG_FAIL", "Cannot connect to the magnetometer" },
    { 3, "GPS_ACC_LOW", "Gps accuracy low" },
    { 4, "MAG_ACC_LOW", "Magnetometer accuracy low" },
    { 5, "BATT_LOW", "Battery voltage low" },
    { 6, "LORA_TIMEOUT", "" },
    { 7, "WIFI_TIMEOUT", "" },
    { 8, "MOTOR_FAULT", "Motor / ESC fault" },
    { 9, "NO_ROUTE", "No route loaded, cannot enable A/P" },
    { 10, "NO_HOME", "Home WP not set" },
    { 11, "WATER_LOW", "Water ingress sensor detected LOW water level" },
    { 12, "WATER_MID", "Water ingress sensor detected MEDIUM water level" },
    { 13, "WATER_HIGH", "Water ingress sensor detected HIGH water level" },
    { 14, "HDG_A1", "Heading accuracy 1" },
    { 15, "HDG_A2", "Heading accuracy 2" },
    { 16, "HDG_A3", "Heading accuracy 3" },
};

