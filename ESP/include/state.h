#pragma once

#include "gps.h"
#include "magnetometer.h"

struct wp
{
    double lat, lon;
};

typedef struct 
{
    wp waypoints[51];
    uint8_t id;
    uint8_t length;
    bool routeReady;
    bool newRouteAvailable;
} Route;

typedef struct 
{
    uint8_t mode;
    uint8_t battery;
    bool gpsFix;
    bool loraTimeout;
    bool wifiTimeout;
    uint32_t commTimeoutTriggerTime;

    uint32_t errorCode;
} SystemStatus;

typedef struct 
{
    int8_t throttle;
    int8_t rudder;
} ManualControls;

typedef struct 
{
    GPSData gps;
    MagData mag;

    Route route;
    wp home;
    bool homeSet;
    uint8_t targetIdx;

    SystemStatus status;
    ManualControls manual;
} State;

extern State globalState;