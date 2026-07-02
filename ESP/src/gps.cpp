#include "gps.h"

static TinyGPSPlus gps;
static HardwareSerial gpsSerial(2);

void ubxEnableSBAS(Stream &port);
void ubxSaveConfig(Stream &port);

void ubxClearConfig(Stream &port);
void ubxColdReset(Stream &port);

int GPSInit()
{
    gpsSerial.begin(9600, SERIAL_8N1, GPSRXPIN, GPSTXPIN);
    delay(1000);

    // One-time config
    //ubxEnableSBAS(gpsSerial);
    //ubxSaveConfig(gpsSerial);

    // Uncomment for reset
    //ubxClearConfig(gpsSerial);
    //ubxColdReset(gpsSerial);

    uint32_t start = millis(); // Wait 3s and check if communication
    while (millis() - start < 3000)
    {
        if (gpsSerial.available() > 10) {return 1;}
    }

    return 0;
}

GPSData getGPS()
{
    static uint32_t lastValid = millis();

    static GPSData data = {};

    while (gpsSerial.available() > 0) {gps.encode(gpsSerial.read());}

    if (gps.location.isValid())
    {   
        if (gps.hdop.hdop() < 3)
        {
            data.lat = gps.location.lat();
            data.lon = gps.location.lng();
            data.speedKMH = gps.speed.kmph();
            data.hdop = gps.hdop.hdop();
            data.satellites = gps.satellites.value();
            data.time = gps.time.value();

            lastValid = millis();
            data.valid = true;
        }
    }

    if ((millis() - lastValid) > 3000) {data.valid = false;}

    return data;
}

// Neo 6M config
void sendUBX(Stream &port, uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    auto ck = [&](uint8_t b) {ck_a += b; ck_b += ck_a;};

    // UBX Sync
    port.write(0xB5);
    port.write(0x62);

    port.write(cls);
    ck(cls);
    port.write(id);
    ck(id);

    uint8_t lo = len & 0xFF;
    uint8_t hi = len >> 8;
    port.write(lo);
    ck(lo);
    port.write(hi);
    ck(hi);

    for (uint16_t i = 0; i < len; i++)
    {
        port.write(payload[i]);
        ck(payload[i]);
    }

    port.write(ck_a);
    port.write(ck_b);
}

void ubxClearConfig(Stream &port)
{
    const uint8_t payload[] = {
        0xFF, 0xFF, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0x00, 0x00,
        0x07
    };

    sendUBX(port, 0x06, 0x09, payload, sizeof(payload));
    delay(500);
}

void ubxColdReset(Stream &port)
{
    const uint8_t payload[] = {
        0xFF, 0xFF, // Clear all nav data
        0x01, // Software rst
        0x00
    };

    sendUBX(port, 0x06, 0x04, payload, sizeof(payload));
    delay(1000);
}

void ubxEnableSBAS(Stream &port)
{
    const uint8_t payload[] = {
        0x01, //Mode : Enabled | Disabled
        0x03, // Usage : Ranging | Diff. corr. | Integrity corr.
        0x03, //MaxSBAS : 0 - 3
        0x00, // Scanmode2 
        0x00, 0x00, 0x00, 0x00 // Scanmode1
    };

    sendUBX(port, 0x06, 0x16, payload, sizeof(payload));
    delay(400);
}

void ubxSaveConfig(Stream &port)
{
    const uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, // Clear mask
        0xFF, 0xFF, 0x00, 0x00, // Save mask, all config
        0x00, 0x00, 0x00, 0x00, // Load mask
        0x07 // Device mask : BBR | Flash | EEPROM
    };

    sendUBX(port, 0x06, 0x09, payload, sizeof(payload));
    delay(500);
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
    uint32_t towAccMs = 2000; // 15s
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