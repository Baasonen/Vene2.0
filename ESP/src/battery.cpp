#include "battery.h"

static float c1F = 0;
static float c2F = 0;
static float c3F = 0;

static bool batInitialized = false;
static uint32_t batLastSampleMs = 0;

void batteryInit()
{
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
}

uint8_t uClamp8(int a) {return (uint8_t)constrain(a, 0, 255);}

Battery readBattery()
{
    uint32_t now = millis();

    if (!batInitialized || (now - batLastSampleMs) >= BAT_SAMPLE_INTERVAL_MS)
    {
        float dt = batInitialized ? (now - batLastSampleMs) / 1000.0f : 0.0f;
        batLastSampleMs = now;

        float c1Gain = 1.0f;
        float c2Gain = 1.0f;
        float c3Gain = 1.0f;

        float cell1 = analogReadMilliVolts(CELL1PIN) / 1000.0f;
        float cell2 = analogReadMilliVolts(CELL2PIN) / 1000.0f;
        float cell3 = analogReadMilliVolts(CELL3PIN) / 1000.0f;

        cell1 *= c1Gain;
        cell2 *= c2Gain;
        cell3 *= c3Gain;

        cell3 -= cell2;
        cell2 -= cell1;

        if (!batInitialized)
        {
            c1F = cell1;
            c2F = cell2;
            c3F = cell3;

            batInitialized = true;
        }
        else
        {
            float a = 1.0f - expf(-dt / BAT_TIME_CONST_S);

            c1F += a * (cell1 - c1F);
            c2F += a * (cell2 - c2F);
            c3F += a * (cell3 - c3F);
        }
    }

    Battery output;

    output.c1F = c1F;
    output.c2F = c2F;
    output.c3F = c3F;
    
    output.lowest = min(output.c1F, min(output.c2F, output.c3F));

    output.c1 = uClamp8((int)((output.c1F - 2.5f) * 100));
    output.c2 = uClamp8((int)((output.c2F - 2.5f) * 100));
    output.c3 = uClamp8((int)((output.c3F - 2.5f) * 100));

    return output;
}

