#include "gpsConfig.h"

void sendUBX(Stream &port, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len)
{
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    auto ck = [&](uint8_t b) { ck_a += b; ck_b += ck_a; };

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

static void ubxSetMsgRate(Stream &port, uint8_t msgClass, uint8_t msgID, uint8_t rate)
{
    const uint8_t payload[] = { msgClass, msgID, rate };
    sendUBX(port, 0x06, 0x01, payload, sizeof(payload));
    delay(100);
}

void ubxSetNavRate(Stream &port, uint16_t measRateMs)
{
    uint16_t navCycles = 1;
    uint16_t timeRef = 0;

    uint8_t payload[6];
    memcpy(payload, &measRateMs, 2);
    memcpy(payload + 2, &navCycles, 2);
    memcpy(payload + 4, &timeRef, 2);

    sendUBX(port, 0x06, 0x08, payload, sizeof(payload));
    delay(100);
}

void ubxSetBaudRate(Stream &port, uint32_t baud)
{
    uint8_t  portID = 1;
    uint8_t  reserved0 = 0;
    uint16_t txReady = 0;
    uint32_t mode = 0x000008D0;
    uint16_t inProtoMask = 0x0001;
    uint16_t outProtoMask = 0x0001;
    uint16_t flags = 0;
    uint16_t reserved5 = 0;

    uint8_t payload[20] = {};
    payload[0] = portID;
    payload[1] = reserved0;
    memcpy(payload + 2, &txReady, 2);
    memcpy(payload + 4, &mode, 4);
    memcpy(payload + 8, &baud, 4);
    memcpy(payload + 12, &inProtoMask, 2);
    memcpy(payload + 14, &outProtoMask, 2);
    memcpy(payload + 16, &flags, 2);
    memcpy(payload + 18, &reserved5, 2);

    sendUBX(port, 0x06, 0x00, payload, sizeof(payload));
    delay(100);
}

void ubxEnableNavMessages(Stream &port)
{
    ubxSetMsgRate(port, 0x01, 0x02, 1); // NAV-POSLLH
    ubxSetMsgRate(port, 0x01, 0x12, 1); // NAV-VELNED
    ubxSetMsgRate(port, 0x01, 0x06, 1); // NAV-SOL
    ubxSetMsgRate(port, 0x01, 0x21, 1); // NAV-TIMEUTC
}

void ubxDisableNMEA(Stream &port)
{
    // Default NEO-6 NMEA set: GGA, GLL, GSA, GSV, RMC, VTG.
    const uint8_t ids[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05 };
    for (uint8_t id : ids) { ubxSetMsgRate(port, 0xF0, id, 0); }
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
        0x01,       // Software rst
        0x00
    };

    sendUBX(port, 0x06, 0x04, payload, sizeof(payload));
    delay(1000);
}

void ubxEnableSBAS(Stream &port)
{
    const uint8_t payload[] = {
        0x01, // Mode : Enabled | Disabled
        0x03, // Usage : Ranging | Diff. corr. | Integrity corr.
        0x03, // MaxSBAS : 0 - 3
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

void gpsRunOneTimeSetup(Stream &port)
{
    ubxEnableSBAS(port);
    ubxEnableNavMessages(port);
    ubxDisableNMEA(port);
    ubxSaveConfig(port);
}