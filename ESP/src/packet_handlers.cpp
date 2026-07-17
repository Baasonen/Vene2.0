#include "packet_handlers.h"

#include "lora.h"
#include "errors.h"

void handleRoutePacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime, uint32_t &lastRoutePacketTime, 
                       Route &tempRoute, bool* wpReceived, uint8_t &receivedCount)
{
    lastPacketReceivedTime = millis();
    lastRoutePacketTime = millis();

    routePacket* rp = (routePacket*)rxBuffer;
    Serial.printf("[LORA] Received Route Packet %i\n", rp->order);

    if (rp->order == 0)
    {
        tempRoute.id = rp->id;
        tempRoute.length = rp->ammnt;
        receivedCount = 0;
        memset(wpReceived, 0, 50 * sizeof(bool));
    }

    if ((rp->id == tempRoute.id) && (rp->order < 50))
    {
        dataPacket ack = {PKT_DATA, rp->id, rp->order};
        beginTransmit((uint8_t*)&ack, sizeof(ack));

        if (!wpReceived[rp->order])
        {
            tempRoute.waypoints[rp->order].lat = rp->lat;
            tempRoute.waypoints[rp->order].lon = rp->lon;
            wpReceived[rp->order] = true;
            receivedCount++;

            if (receivedCount == tempRoute.length)
            {
                tempRoute.newRouteAvailable = true;

                if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    globalState.status.loraTimeout = false;
                    globalState.status.routeReady = true;

                    globalState.route = tempRoute;

                    xSemaphoreGive(stateMutex);
                }
            }
        }
    }
}

void handleControlPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime)
{
    lastPacketReceivedTime = millis();
    controlPacket* cp = (controlPacket*)rxBuffer;

    dataPacket ack = {PKT_DATA, 255, cp->mode};

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        globalState.status.mode = cp->mode;
        globalState.status.loraTimeout = false;
        
        xSemaphoreGive(stateMutex);
    }
}

void handleDataPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime)
{
    dataPacket* dp = (dataPacket*)rxBuffer;

    if (dp->id == 254)
    {
        lastPacketReceivedTime = millis();

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            globalState.status.loraTimeout = false;

            xSemaphoreGive(stateMutex);
        }
    }
}

void handleResetErrorsPacket()
{
    clearAllErrors();

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        globalState.status.loraTimeout = false;
        
        xSemaphoreGive(stateMutex);
    }

    dataPacket ack = {PKT_DATA, 254, PKT_RESET_ERRORS};
    beginTransmit((uint8_t*)&ack, sizeof(ack));
}

void handleHomeSetPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime)
{
    lastPacketReceivedTime = millis();
    homeSetPacket* hp = (homeSetPacket*)rxBuffer;

    if (hp->lat != 0.0 || hp->lon != 0.0)
    {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.home.lat = hp->lat;
            globalState.status.home.lon = hp->lon;
            globalState.status.homeSet = true;
            globalState.status.homeNeedsSave = true;
            globalState.status.loraTimeout = false;

            xSemaphoreGive(stateMutex);
        }

        Serial.printf("[LORA] Home set: %.6f, %.6f\n", hp->lat, hp->lon);
    }

    dataPacket ack = {PKT_DATA, PKT_HOME_SET, 0x01};
    beginTransmit((uint8_t*)&ack, sizeof(ack));
}

void handleHomeReqPacket(uint32_t &lastPacketReceivedTime)
{
    lastPacketReceivedTime = millis();

    homeDataPacket resp = {};
    resp.packetID = PKT_HOME_DATA;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        resp.lat = globalState.status.homeSet ? globalState.status.home.lat : 0.0;
        resp.lon = globalState.status.homeSet ? globalState.status.home.lon : 0.0;

        globalState.status.loraTimeout = false;

        xSemaphoreGive(stateMutex);
    }

    Serial.println("[LORA] Home requested, responding...");
    beginTransmit((uint8_t*)&resp, sizeof(resp));
}

void handleTimeDataPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime)
{
    lastPacketReceivedTime = millis();
    timeDataPacket* tp = (timeDataPacket*)rxBuffer;

    bool homeSetLocal = false;
    double homeLatLocal = 0.0;
    double homeLonLocal = 0.0;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (!globalState.status.timeSet)
        {
            globalState.status.unixTime = tp->unixTime;
            globalState.status.timeSet = true;

            Serial.printf("[LORA] Unix time received: %lu\n", (unsigned long)tp->unixTime);

            if (globalState.status.homeSet)
            {
                homeSetLocal = true;
                homeLatLocal = globalState.status.home.lat;
                homeLonLocal = globalState.status.home.lon;
            }
        }
        
        globalState.status.loraTimeout = false;
        xSemaphoreGive(stateMutex);
    }

    if (homeSetLocal)
    {
        gpsInitAid(homeLatLocal, homeLonLocal, 0.0, tp->unixTime);

        Serial.println("[GPS] UBX aiding data injected");
    }
}

void handleCourseSetPacket(uint8_t* rxBuffer, uint32_t &lastPacketReceivedTime)
{
    lastPacketReceivedTime = millis();
    courseSetPacket* cp = (courseSetPacket*)rxBuffer;

    courseDataPacket response = {};
    response.packetID = PKT_COURSE_DATA;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) 
    {
        if (cp->course >= 0.0 && cp->course < 360.0f) 
        {
            globalState.status.targetCourse = cp->course;
        }

        response.course = globalState.status.targetCourse;
        globalState.status.loraTimeout = false;
        xSemaphoreGive(stateMutex);
    }

    beginTransmit((uint8_t*)&response, sizeof(response));
}