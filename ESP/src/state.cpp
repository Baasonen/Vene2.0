#include "state.h"

#include "errors.h"

#define FAILSAFE_DELAY_MS 10000

enum : uint8_t
{
    MODE_STOP = 0,
    MODE_MANUAL = 1,
    MODE_AP = 2,
    MODE_RTH = 3,
    MODE_CRS = 4,
    MODE_COUNT
};

struct ModeRequirements
{
    bool reqWifi;
    bool reqLora;
    bool reqMag;
    bool reqPosSol;
    bool reqHome;
    bool reqRoute;
};

static const ModeRequirements MODE_REQS[MODE_COUNT] = {
    {false, false, false, false, false, false}, // Stop
    {true, false, false, false, false, false}, // Manual
    {false, true, true, true, true, true}, // AP
    {false, false, true, true, true, false}, // RTH
    {false, true, true, false, false, false}, // CRS
};

static bool modeValid(uint8_t m)
{
    if (m >= MODE_COUNT) {return false;}
    const ModeRequirements& r = MODE_REQS[m];

    if (r.reqWifi && hasError(ERR_WIFI_TIMEOUT)) {return false;}
    if (r.reqLora && hasError(ERR_LORA_TIMEOUT)) {return false;}
    if (r.reqPosSol && hasError(ERR_KF_UNINIT)) {return false;}
    if (r.reqMag && (hasError(ERR_MAG_FAIL) || hasError(ERR_MAG_ACC_LOW))) {return false;}
    if (r.reqHome && hasError(ERR_NO_HOME)) {return false;}
    if (r.reqRoute && hasError(ERR_NO_ROUTE)) {return false;}

    return true;
}

uint8_t validateMode(const SystemStatus& status, const SensorData& sensors)
{
    if (status.ctrlArmed && hasError(ERR_LORA_TIMEOUT) && 
        (millis() - status.commTimeoutTriggerTime > FAILSAFE_DELAY_MS))
    {
        return modeValid(MODE_RTH) ? MODE_RTH : MODE_STOP;
    }

    uint8_t requested = status.mode;

    if (modeValid(requested)) {return requested;}

    return MODE_STOP;
}