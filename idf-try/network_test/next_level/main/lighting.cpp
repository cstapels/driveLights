#include "lighting.h"
#include "effects.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

led_strip_handle_t neopixel_strip = nullptr;

void initialize_neopixels()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = NEOPIXEL_DATA_PIN;
    strip_config.max_leds = NEOPIXEL_NUM_LEDS;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.mem_block_symbols = 64;
    rmt_config.flags.with_dma = false;

    ESP_ERROR_CHECK(
        led_strip_new_rmt_device(
            &strip_config,
            &rmt_config,
            &neopixel_strip
        )
    );

    ESP_ERROR_CHECK(
        led_strip_clear(neopixel_strip)
    );

    gpio_set_level(NEOPIXEL_POWER_PIN, 0);
}

static void get_solid_rgb(const ThingSpeakData *data, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    // Handle both packed 24-bit 0xRRGGBB in color1 and individual R, G, B in color1, color2, color3
    if (data->color1 > 255U) {
        *red = (uint8_t)((data->color1 >> 16) & 0xFFU);
        *green = (uint8_t)((data->color1 >> 8) & 0xFFU);
        *blue = (uint8_t)(data->color1 & 0xFFU);
    } else {
        *red = (uint8_t)(data->color1 & 0xFFU);
        *green = (uint8_t)(data->color2 & 0xFFU);
        *blue = (uint8_t)(data->color3 & 0xFFU);
    }
}

void update_neopixels(const ThingSpeakData *data)
{
    if (data->brightness <= 1) {
        ESP_ERROR_CHECK(
            led_strip_clear(neopixel_strip)
        );
        gpio_set_level(NEOPIXEL_POWER_PIN, 0);
        return;
    }

    gpio_set_level(NEOPIXEL_POWER_PIN, 1);

    if (data->pattern >= 1 && data->pattern <= 14) {
        apply_effect(data);
        return;
    }

    if (data->pattern != 0) {
        printf(
            "NeoPixel pattern %u is not implemented yet\n",
            data->pattern
        );
        return;
    }

    uint8_t red, green, blue;
    get_solid_rgb(data, &red, &green, &blue);

    red = (uint8_t)(((uint32_t)red * data->brightness) / 255U);
    green = (uint8_t)(((uint32_t)green * data->brightness) / 255U);
    blue = (uint8_t)(((uint32_t)blue * data->brightness) / 255U);

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                i,
                red,
                green,
                blue
            )
        );
    }

    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel_strip)
    );
}

void led_update_task(void *parameter)
{
    MeshMessage update;
    ThingSpeakData current_data = {};
    bool has_data = false;
    TickType_t wait_ticks = portMAX_DELAY;

    while (1) {
        if (
            xQueueReceive(
                led_queue,
                &update,
                wait_ticks
            ) == pdTRUE
        ) {
            // Drain any pending queue items to guarantee running the latest command
            while (xQueueReceive(led_queue, &update, 0) == pdTRUE) {
            }

            reset_effects();
            current_data = update.thingspeak;
            has_data = true;
            update_neopixels(&current_data);
            send_update_ack(update.message_id);
        } else if (
            has_data &&
            current_data.pattern >= 1 &&
            current_data.pattern <= 14 &&
            current_data.brightness > 1
        ) {
            update_neopixels(&current_data);
        }

        if (
            has_data &&
            current_data.pattern >= 1 &&
            current_data.pattern <= 14 &&
            current_data.brightness > 1
        ) {
            uint16_t speed = current_data.fxSpeed;
            if (speed > 100) {
                speed = 100;
            }

            wait_ticks = pdMS_TO_TICKS(308U - ((speed * 14U) / 5U));
        } else {
            wait_ticks = portMAX_DELAY;
        }
    }
}
