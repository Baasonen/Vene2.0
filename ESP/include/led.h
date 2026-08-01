#pragma once

#include <FastLED.h>

#define LED_D_PIN 19
#define NUMBER_LEDS 2

extern CRGB leds[NUMBER_LEDS];

void ledSetup();

struct LedPulse
{
    CRGB color;
    uint16_t onMs;
    uint16_t offMs;
    uint16_t pulsesLeft;
    bool infinite;
    bool on = false;
    uint32_t lastChange = 0;

    void start(CRGB c, uint16_t count, uint16_t onDur, uint16_t offDur)
    {
        color = c;
        infinite = (count == 0);
        pulsesLeft = count;
        onMs = onDur;
        offMs = offDur;
        on = true;
        lastChange = millis();
    }

    CRGB update()
    {
        if (!infinite && pulsesLeft == 0) {return CRGB::Black;}

        if (millis() - lastChange >= (on ? onMs : offMs))
        {
            lastChange = millis();

            if (on && !infinite) {pulsesLeft--;}

            on = !on;
        }

        return on ? color : CRGB::Black;
    }
};

struct LedDoubleFlash
{
    CRGB color;
    uint16_t onMs;
    uint16_t gapMs;
    uint16_t pauseMs;
    uint8_t phase = 0;
    uint32_t lastChange = 0;

    void start(CRGB c, uint16_t onDur, uint16_t gapDur, uint16_t pauseDur)
    {
        color = c;
        onMs = onDur;
        gapMs = gapDur;
        pauseMs = pauseDur;
        phase = 0;
        lastChange = millis();
    }

    CRGB update()
    {
        uint16_t duration = (phase == 0 || phase == 2) ? onMs : (phase == 1) ? gapMs : pauseMs;

        if (millis() - lastChange >= duration)
        {
            lastChange = millis();
            phase = (phase + 1) % 4;
        }

        return (phase == 0 || phase == 2) ? color : CRGB::Black;
    }
};