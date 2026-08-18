#include <string.h>

#include "gps.h"
#include "gpsConfig.h"
#include "errors.h"

static HardwareSerial gpsSerial(2);

// UBX NAV parser

#define GPS_DEBUG false

#if GPS_DEBUG
static uint32_t dbgFrames = 0;
static uint32_t dbgChecksumFail = 0;
static uint32_t dbgPosllhCount = 0;
static uint32_t dbgVelnedCount = 0;
static uint32_t dbgSolCount = 0;
static uint32_t dbgTimeutcCount = 0;
static uint32_t dbgLastPrintMs = 0;
#endif

enum class UbxState : uint8_t { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };

static UbxState ubxState = UbxState::SYNC1;
static uint8_t ubxClass, ubxId;
static uint16_t ubxLen, ubxPayloadIdx;
static uint8_t ubxPayload[64];
static uint8_t ubxCkA, ubxCkB;

static bool solFixOk = false;
static uint32_t lastSolMs = 0;
static uint32_t lastPollMs = 0;
static uint32_t lastGoodFixMs = 0;

static uint32_t currentITOW = 0;
static uint32_t lastConsumedITOW = 0;

static GPSData data = {};

static void ubxChecksumStep(uint8_t b) {ubxCkA += b; ubxCkB += ubxCkA;}

static void handlePOSLLH()
{
    int32_t lat, lon;
    uint32_t hAcc;
    memcpy(&lon, ubxPayload + 4, 4);
    memcpy(&lat, ubxPayload + 8, 4);
    memcpy(&hAcc, ubxPayload + 20, 4);

    data.lat = lat * 1e-7;
    data.lon = lon * 1e-7;
    data.hAccM = hAcc / 1000.0f; 

    lastPollMs = millis();

    #if GPS_DEBUG
    dbgPosllhCount++;
    #endif
}

static uint32_t utcToUnix(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t sec)
{
    int32_t y = year;
    if (month <= 2) y -= 1;
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    uint32_t yoe = (uint32_t)(y - era * 400);
    uint32_t doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int32_t days = era * 146097 + (int32_t)doe - 719468; // Days since 1.1.1970
 
    return (uint32_t)days * 86400UL + hour * 3600UL + minute * 60UL + sec;
}

static void handleVELNED()
{
    uint32_t gSpeed;
    int32_t heading;
    int32_t velN_cms, velE_cms;
    uint32_t sAcc_cms;

    memcpy(&gSpeed, ubxPayload + 20, 4);
    memcpy(&heading, ubxPayload + 24, 4);
    memcpy(&velN_cms, ubxPayload + 4, 4);
    memcpy(&velE_cms, ubxPayload + 8, 4);
    memcpy(&sAcc_cms, ubxPayload + 28, 4);
    
    data.speedKMH = gSpeed * 0.036f; // cm/s -> kmh/h
    data.headingDeg = heading * 1e-5f; // 1e-5 deg -> deg

    data.velN = velN_cms / 100.0f;
    data.velE = velE_cms / 100.0f;
    data.velAccM = sAcc_cms / 100.0f;

    #if GPS_DEBUG
    dbgVelnedCount++;
    #endif
}

static void handleSOL()
{
    uint32_t iTOW;
    memcpy(&iTOW, ubxPayload + 0, 4);

    uint8_t gpsFix = ubxPayload[10];
    uint8_t numSV = ubxPayload[47];

    data.fixType = gpsFix;
    data.satellites = numSV;
    currentITOW = iTOW;

    solFixOk = (gpsFix == 0x03 || gpsFix == 0x04);

    lastSolMs = millis();

    #if GPS_DEBUG
    dbgSolCount++;
    #endif
}

static void handleTIMEUTC()
{
    uint16_t year;
    uint8_t month, day, hour, minute, sec, valid;
    
    memcpy(&year, ubxPayload + 12, 2);
    month = ubxPayload[14];
    day = ubxPayload[15];
    hour = ubxPayload[16];
    minute = ubxPayload[17];
    sec = ubxPayload[18];
    valid = ubxPayload[19];

    data.utcTimeValid = (valid & 0x04) != 0;

    if (data.utcTimeValid) {data.unixTime = utcToUnix(year, month, day, hour, minute, sec);}

    #if GPS_DEBUG
    dbgTimeutcCount++;
    #endif
}

static void ubxDispathc()
{
    if (ubxClass != 0x01) {return;}

    switch (ubxId)
    {
        case 0x02: 
            if (ubxLen >= 28) {handlePOSLLH();}
            break;

        case 0x12: 
            if (ubxLen >= 36) {handleVELNED();}
            break;

        case 0x06: 
            if (ubxLen >= 52) {handleSOL();}
            break;

        case 0x21: 
            if (ubxLen >= 20) {handleTIMEUTC();}
            break;

        default: 
            break;
    }
}

static void ubxFeed(uint8_t b)
{
    static uint8_t rxCkA, rxCkB;

    switch (ubxState)
    {
        case UbxState::SYNC1:
            ubxState = (b == 0xB5) ? UbxState::SYNC2 : UbxState::SYNC1;
            break;

        case UbxState::SYNC2:
            ubxState = (b == 0x62) ? UbxState::CLASS : UbxState::SYNC1;
            break;

        case UbxState::CLASS:
            ubxClass = b;
            ubxCkA = ubxCkB = 0;
            ubxChecksumStep(b);
            ubxState = UbxState::ID;
            break;

        case UbxState::ID:
            ubxId = b;
            ubxChecksumStep(b);
            ubxState = UbxState::LEN1;
            break;

        case UbxState::LEN1:
            ubxLen = b;
            ubxChecksumStep(b);
            ubxState = UbxState::LEN2;
            break;

        case UbxState::LEN2:
            ubxLen |= (uint16_t)b << 8;
            ubxChecksumStep(b);
            ubxPayloadIdx = 0;

            if (ubxLen == 0) {ubxState = UbxState::CK_A;}
            else if (ubxLen > sizeof(ubxPayload)) {ubxState = UbxState::SYNC1;}
            else {ubxState = UbxState::PAYLOAD;}
            break;

        case UbxState::PAYLOAD:
            ubxPayload[ubxPayloadIdx++] = b;
            ubxChecksumStep(b);

            if (ubxPayloadIdx >= ubxLen) {ubxState = UbxState::CK_A;}
            break;

        case UbxState::CK_A:
            rxCkA = b;
            ubxState = UbxState::CK_B;
            break;
        
        case UbxState::CK_B:
            rxCkB = b;

            if (rxCkA == ubxCkA && rxCkB == ubxCkB) 
            {
                #if GPS_DEBUG
                dbgFrames++;
                #endif

                ubxDispathc();
            }

            #if GPS_DEBUG
            else {dbgChecksumFail++;}
            #endif

            ubxState = UbxState::SYNC1; 
            break;
    }
}

#if GPS_DEBUG
static void printGPSDebug()
{
    uint32_t now = millis();
    if (now - dbgLastPrintMs < 1000) {return;}
    dbgLastPrintMs = now;

    Serial.printf(
        "[GPS] frames=%lu ckFail=%lu | POSLLH=%lu VELNED=%lu SOL=%lu TIMEUTC=%lu | "
        "fix=%u sats=%u hAcc=%.1fm valid=%d\n",
        (unsigned long)dbgFrames, (unsigned long)dbgChecksumFail,
        (unsigned long)dbgPosllhCount, (unsigned long)dbgVelnedCount,
        (unsigned long)dbgSolCount, (unsigned long)dbgTimeutcCount,
        data.fixType, data.satellites, data.hAccM, (int)data.valid);
}
#endif

int GPSInit()
{
    gpsSerial.setRxBufferSize(1024);
    gpsSerial.begin(9600, SERIAL_8N1, GPSRXPIN, GPSTXPIN); // always start at factory default
    delay(1000);

    ubxDisableNMEA(gpsSerial);
    ubxEnableSBAS(gpsSerial);
    ubxSetNavRate(gpsSerial, 200); // 5 Hz
    ubxEnableNavMessages(gpsSerial);

    ubxSetBaudRate(gpsSerial, 38400);
    gpsSerial.flush(); // make sure CFG-PRT bytes go out at 9600 before we switch
    delay(50);
    gpsSerial.updateBaudRate(38400);
    delay(100);

    uint32_t start = millis();
    while (millis() - start < 3000)
    {
        if (gpsSerial.available() > 10) {return 1;}
    }

    setError(ERR_GPS_INIT);
    return 0;
}

GPSData getGPS()
{
    while (gpsSerial.available() > 0) {ubxFeed(gpsSerial.read());}

    #if GPS_DEBUG
    printGPSDebug();
    #endif

    data.freshFix = solFixOk && (currentITOW != lastConsumedITOW);
    lastConsumedITOW = currentITOW;

    uint32_t now = millis();
    bool solFresh = (now - lastSolMs) < 1500;
    bool posFresh = (now - lastPollMs) < 1500;
    bool accOk = data.hAccM < GPS_HACC_GATE;
    data.accDegraded = true;

    if (solFixOk && solFresh && posFresh && accOk && data.satellites >= MIN_SAT_COUNT)
    {
        lastGoodFixMs = now;
        data.valid = true;

        if (data.hAccM < GPS_HACC_ACCURATE) {data.accDegraded = false;}
    }

    if ((now - lastGoodFixMs) > 3000) {data.valid = false;}

    if (data.utcTimeValid)
    {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            globalState.status.unixTime = data.unixTime;
            xSemaphoreGive(stateMutex);
        }
    }

    return data;
}

static void unixToGPSTime(uint32_t unix, uint16_t &week, uint32_t &towMS)
{
    const uint32_t GPS_EPOCH = 315964800UL;
    const uint32_t LEAP_SEC = 18;

    if (unix < GPS_EPOCH) 
    {
        week = 0;
        towMS = 0;
        return;
    }

    uint32_t gpsSec = unix - GPS_EPOCH + LEAP_SEC;
    week = (uint16_t)(gpsSec / 604800UL);
    towMS = (gpsSec % 604800UL) * 1000UL;
}

void ubxInitAid(Stream &port, double lat, double lon, float altM, uint32_t unix)
{
    uint16_t gpsWeek = 0;
    uint32_t gpsTOW_ms = 0;
    unixToGPSTime(unix, gpsWeek, gpsTOW_ms);

    int32_t latI = (int32_t)(lat * 1e7f);
    int32_t lonI = (int32_t)(lon * 1e7f);
    int32_t altCm = (int32_t)(altM * 100.0f);
    uint32_t posAcc = 500000; // 5km in cm
    uint16_t tmCfg = 0; 
    int32_t towNs = 0;
    uint32_t towAccMs = 2000; // 2s
    uint32_t towAccNs = 0;
    int32_t clkD = 0;
    uint32_t clkDAcc = 0;
    uint32_t flags = 0x23;

    uint8_t payload[48] = {};
    memcpy(payload + 0, &latI, 4);
    memcpy(payload + 4, &lonI, 4);
    memcpy(payload + 8, &altCm, 4);
    memcpy(payload + 12, &posAcc, 4);

    memcpy(payload + 16, &tmCfg, 2);
    memcpy(payload + 18, &gpsWeek, 2);
    memcpy(payload + 20, &gpsTOW_ms, 4);

    memcpy(payload + 24, &towNs, 4);
    memcpy(payload + 28, &towAccMs, 4);
    memcpy(payload + 32, &towAccNs, 4);
    memcpy(payload + 36, &clkD, 4);
    memcpy(payload + 40, &clkDAcc, 4);
    memcpy(payload + 44, &flags, 4);

    sendUBX(port, 0x0B, 0x01, payload, 48);
}

void gpsInitAid(double lat, double lon, float altM, uint32_t unix)
{
    ubxInitAid(gpsSerial, lat, lon, altM, unix);
}