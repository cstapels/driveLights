#include "lighting.h"
#include "effects.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

led_strip_handle_t neopixel_strip = nullptr;

static void flash_startup_test_sequence()
{
    static const uint8_t startup_colors[][3] = {
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255}
    };

    gpio_set_level(NEOPIXEL_POWER_PIN, 1);

    for (size_t i = 0; i < sizeof(startup_colors) / sizeof(startup_colors[0]); ++i) {
        for (int led = 0; led < NEOPIXEL_NUM_LEDS; ++led) {
            ESP_ERROR_CHECK(
                led_strip_set_pixel(
                    neopixel_strip,
                    led,
                    startup_colors[i][0],
                    startup_colors[i][1],
                    startup_colors[i][2]
                )
            );
        }

        ESP_ERROR_CHECK(
            led_strip_refresh(neopixel_strip)
        );

        vTaskDelay(pdMS_TO_TICKS(120));
    }

    ESP_ERROR_CHECK(
        led_strip_clear(neopixel_strip)
    );
    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel_strip)
    );
    gpio_set_level(NEOPIXEL_POWER_PIN, 0);
}

void initialize_neopixels() {
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = NEOPIXEL_DATA_PIN;
    strip_config.max_leds = NEOPIXEL_NUM_LEDS;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    // Larger buffer reduces ISR refills so WiFi/ESP-NOW interrupt latency
    // can't starve the RMT channel and corrupt pixels mid-transmission.
    rmt_config.mem_block_symbols = 256;
    rmt_config.flags.with_dma = false;

    //SPI CONFIGS
    led_strip_spi_config_t spi_config = {
        .clk_src = SPI_CLK_SRC_DEFAULT,
        .spi_bus = SPI2_HOST,          // Uses the ESP32's SPI2 peripheral
        .flags={.with_dma = true,},    // DMA bypasses the CPU completely!
    };

    // FIXED: Removed the internal semicolon and properly closed the macro
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &neopixel_strip));

    ESP_ERROR_CHECK(led_strip_clear(neopixel_strip));
    
    flash_startup_test_sequence();
}

static void get_solid_rgb(const ThingSpeakData *data, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    // Solid color (pattern 0) always uses color1 as a packed 0xRRGGBB value.
    // color2/color3 are reserved for multi-color effects, not solid mode.
    uint32_t packed_color = data->color1 & 0x00FFFFFFU;
    *red = (uint8_t)((packed_color >> 16) & 0xFFU);
    *green = (uint8_t)((packed_color >> 8) & 0xFFU);
    *blue = (uint8_t)(packed_color & 0xFFU);
}

void update_neopixels(const ThingSpeakData *data)
{
    printf(
        "update_neopixels: brightness=%u pattern=%u\n",
        data->brightness,
        data->pattern
    );

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

    printf(
        "update_neopixels: solid rgb=(%u,%u,%u)\n",
        red,
        green,
        blue
    );

    ESP_ERROR_CHECK(
        led_strip_clear(neopixel_strip)
    );

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

    printf("update_neopixels: refresh done\n");
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

            printf("led_update_task: dequeued message_id=%lu\n", (unsigned long)update.message_id);

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
