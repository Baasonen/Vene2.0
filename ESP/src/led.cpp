#include "led.h"
#include "driver/rmt.h"

const CRGB CRGB::Black = CRGB(0, 0, 0);
const CRGB CRGB::White = CRGB(255, 255, 255);
const CRGB CRGB::Red = CRGB(255, 0, 0);
const CRGB CRGB::Green = CRGB(0, 255, 0);
const CRGB CRGB::Blue = CRGB(0, 0, 255);
const CRGB CRGB::Yellow = CRGB(255, 255, 0);
const CRGB CRGB::Orange = CRGB(255, 128, 0);
const CRGB CRGB::Purple = CRGB(128, 0, 128);
const CRGB CRGB::Cyan = CRGB(0, 255, 255);

CRGB leds[NUMBER_LEDS];

// APA106 bit timing
// 0: 350ns Hi, 1360ns Lo
// 1: 1360ns Hi, 350ns Lo
// Latch / Rst: >= 50us Lo
namespace
{
    constexpr rmt_channel_t LED_RMT_CHANNEL = RMT_CHANNEL_0;
    constexpr uint8_t RMT_CLK_DIV = 4; // 80 MHz / 4 = 20 MHz -> 50ns per tick
    constexpr float NS_PER_TICK = 50.0f;
    constexpr uint32_t RESET_NS = 60000; // >= 50 us

    uint16_t ticks(uint32_t ns) {return (uint16_t)(ns / NS_PER_TICK + 0.5f);}

    rmt_item32_t makeItem(uint16_t highTicks, uint16_t lowTicks)
    {
        rmt_item32_t item;
        item.duration0 = highTicks;
        item.level0 = 1;
        item.duration1 = lowTicks;
        item.level1 = 0;
        
        return item;
    }

    const rmt_item32_t RMT_BIT0 = makeItem(ticks(350), ticks(1360));
    const rmt_item32_t RMT_BIT1 = makeItem(ticks(1360), ticks(350));

    void encodeColor(rmt_item32_t *out, const CRGB &color)
    {
        const uint8_t bytes[3] = {color.r, color.g, color.b};

        int idx = 0;
        for (uint8_t byteVal : bytes)
        {
            for (int8_t bit = 7; bit >= 0; bit--)
            {
                out[idx++] = (byteVal & (1 << bit)) ? RMT_BIT1 : RMT_BIT0;
            }
        }
    }
}

void ledSetup()
{
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = LED_RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)LED_D_PIN;
    config.mem_block_num = 1;
    config.clk_div = RMT_CLK_DIV;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);

    Serial.println("[LED] Led Init");
}

void ledShow()
{
    static rmt_item32_t items[NUMBER_LEDS * 24 + 1];

    for (int i = 0; i < NUMBER_LEDS; i++) {encodeColor(&items[i * 24], leds[i]);}

    items[NUMBER_LEDS * 24] = makeItem(0, 0);
    items[NUMBER_LEDS * 24].duration0 = ticks(RESET_NS);
    items[NUMBER_LEDS * 24].level0 = 0;

    rmt_write_items(LED_RMT_CHANNEL, items, NUMBER_LEDS * 24 + 1, true);
}