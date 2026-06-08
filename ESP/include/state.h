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
    uint8_t targetWaypoint;

    bool loraTimeout;
    bool wifiTimeout;
    uint32_t commTimeoutTriggerTime;

    wp home;
    bool homeSet;
    bool homeNeedsSave;

    bool routeReady;
    uint8_t targetIdx;

    uint32_t errorCode;
    uint32_t unixTime;
    bool timeSet;
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
} SensorData;

typedef struct 
{
    SensorData sensors;
    Route route;
    SystemStatus status;
    ManualControls manual;
} State;

extern State globalState;
extern SemaphoreHandle_t stateMutex;
extern QueueHandle_t sensorQueue;

uint8_t validateMode(const SystemStatus& status, const SensorData& sensors);