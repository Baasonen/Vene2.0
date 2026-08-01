#include "led.h"

CRGB leds[NUMBER_LEDS];

void ledSetup()
{
    FastLED.addLeds<APA106, LED_D_PIN, RGB>(leds, NUMBER_LEDS);

    Serial.println("[LED] Led Init");
}