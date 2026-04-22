#pragma once

#include "gps.h"
#include "magnetometer.h"

struct wp
{
    double lat, lon;
};

typedef struct 
{
    wp waypoints[50];
    uint8_t id;
    uint8_t length;
    bool newRouteAvailable;
} Route;

typedef struct 
{
    uint8_t mode;
    uint8_t battery;
    bool gpsFix;
    bool commTimeout;

    uint32_t errorCode;
} SystemStatus;

typedef struct 
{
    GPSData gps;
    MagData mag;

    Route route;
    wp home;
    bool homeSet;
    uint8_t targetIdx;

    SystemStatus status;
} State;

extern State globalState;