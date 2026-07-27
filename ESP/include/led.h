#pragma once

#include <FastLED.h>

#define LED_D_PIN 19
#define NUMBER_LEDS 2

CRGB leds[NUMBER_LEDS];

void ledSetup()
{
    FastLED.addLeds<APA106, LED_D_PIN, RGB>(leds, NUMBER_LEDS);

    Serial.println("[LED] Led Init");
}