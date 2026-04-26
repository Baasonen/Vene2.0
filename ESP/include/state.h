#pragma once

#include "gps.h"
#include "magnetometer.h"

#define GPS_MAX_HDOP 1.5
#define MAG_MIN_ACC 2

struct wp
{
    double lat, lon;
};

typedef struct 
{
    wp waypoints[51];
    uint8_t id;
    uint8_t length;
    bool newRouteAvailable;
} Route;

typedef struct 
{
    uint8_t mode;
    uint8_t battery;
    bool loraTimeout;
    bool wifiTimeout;
    uint32_t commTimeoutTriggerTime;

    wp home;
    bool homeSet;

    bool routeReady;
    uint8_t targetIdx;

    uint32_t errorCode;
    bool initFail;
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

    SystemStatus status;
    ManualControls manual;
} State;

extern State globalState;

uint8_t validateMode(const SystemStatus& status, const GPSData& gps, const MagData& mag);