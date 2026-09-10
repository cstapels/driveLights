#include "effects.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FIRE_COOLING 55
#define FIRE_SPARKING 120
#define FLARE_SPARK_COUNT 5
#define EXPLOSION_SPARK_COUNT (NEOPIXEL_NUM_LEDS / 2)
#define FLARE_GRAVITY (-0.004F)

static uint8_t fire_heat[NEOPIXEL_NUM_LEDS];
static uint8_t rainbow_hue = 0;
static uint8_t confetti_red[NEOPIXEL_NUM_LEDS];
static uint8_t confetti_green[NEOPIXEL_NUM_LEDS];
static uint8_t confetti_blue[NEOPIXEL_NUM_LEDS];
static uint8_t sinelon_red[NEOPIXEL_NUM_LEDS];
static uint8_t sinelon_green[NEOPIXEL_NUM_LEDS];
static uint8_t sinelon_blue[NEOPIXEL_NUM_LEDS];
static uint8_t sinelon_phase = 0;
static uint8_t juggle_red[NEOPIXEL_NUM_LEDS];
static uint8_t juggle_green[NEOPIXEL_NUM_LEDS];
static uint8_t juggle_blue[NEOPIXEL_NUM_LEDS];
static uint8_t juggle_phase = 0;
static uint8_t fade_level = 0;
static bool fade_rising = true;
static float flare_position = 0.0F;
static float flare_velocity = 0.0F;
static float flare_brightness = 1.0F;
static float flare_spark_position[FLARE_SPARK_COUNT];
static float flare_spark_velocity[FLARE_SPARK_COUNT];
static float flare_spark_brightness[FLARE_SPARK_COUNT];
static float explosion_spark_position[EXPLOSION_SPARK_COUNT];
static float explosion_spark_velocity[EXPLOSION_SPARK_COUNT];
static float explosion_spark_brightness[EXPLOSION_SPARK_COUNT];
static int explosion_spark_count = 0;
static bool flare_exploding = false;

static uint8_t add_saturated(uint8_t value, uint8_t amount)
{
    uint16_t result = (uint16_t)value + amount;
    return (uint8_t)(result > 255U ? 255U : result);
}

static void set_scaled_color(
    int position,
    uint32_t color,
    uint8_t brightness,
    uint8_t level
)
{
    if (position < 0 || position >= NEOPIXEL_NUM_LEDS) {
        return;
    }

    uint32_t scale = (uint32_t)brightness * level;
    uint8_t red = (uint8_t)((((color >> 16) & 0xFFU) * scale) / (255U * 255U));
    uint8_t green = (uint8_t)((((color >> 8) & 0xFFU) * scale) / (255U * 255U));
    uint8_t blue = (uint8_t)(((color & 0xFFU) * scale) / (255U * 255U));

    ESP_ERROR_CHECK(
        led_strip_set_pixel(
            neopixel_strip,
            position,
            red,
            green,
            blue
        )
    );
}

static void hsv_to_rgb(
    uint8_t hue,
    uint8_t saturation,
    uint8_t value,
    uint8_t *red,
    uint8_t *green,
    uint8_t *blue
)
{
    uint8_t sector = hue / 43U;
    uint8_t offset = (uint8_t)((hue - (sector * 43U)) * 6U);
    uint8_t descending = (uint8_t)((value * (255U - ((saturation * offset) / 255U))) / 255U);
    uint8_t ascending = (uint8_t)((value * (255U - ((saturation * (255U - offset)) / 255U))) / 255U);
    uint8_t saturated = (uint8_t)((value * (255U - saturation)) / 255U);

    switch (sector) {
        case 0:
            *red = value;
            *green = ascending;
            *blue = saturated;
            break;
        case 1:
            *red = descending;
            *green = value;
            *blue = saturated;
            break;
        case 2:
            *red = saturated;
            *green = value;
            *blue = ascending;
            break;
        case 3:
            *red = saturated;
            *green = descending;
            *blue = value;
            break;
        case 4:
            *red = ascending;
            *green = saturated;
            *blue = value;
            break;
        default:
            *red = value;
            *green = saturated;
            *blue = descending;
            break;
    }
}

static uint8_t rgb_to_hue(uint32_t color)
{
    uint8_t red = (uint8_t)((color >> 16) & 0xFFU);
    uint8_t green = (uint8_t)((color >> 8) & 0xFFU);
    uint8_t blue = (uint8_t)(color & 0xFFU);
    uint8_t maximum = red > green ? red : green;
    maximum = maximum > blue ? maximum : blue;
    uint8_t minimum = red < green ? red : green;
    minimum = minimum < blue ? minimum : blue;

    if (maximum == minimum) {
        return 0;
    }

    int16_t hue;
    if (maximum == red) {
        hue = 43 * (green - blue) / (maximum - minimum);
    } else if (maximum == green) {
        hue = 85 + 43 * (blue - red) / (maximum - minimum);
    } else {
        hue = 171 + 43 * (red - green) / (maximum - minimum);
    }

    return (uint8_t)hue;
}

static uint8_t heat_to_red(uint8_t heat)
{
    if (heat < 85U) {
        return 0;
    }
    if (heat < 170U) {
        return (uint8_t)((heat - 85U) * 3U);
    }
    return 255U;
}

static uint8_t heat_to_green(uint8_t heat)
{
    if (heat < 85U) {
        return 0;
    }
    if (heat < 170U) {
        return (uint8_t)((heat - 85U) * 3U);
    }
    return (uint8_t)((255U - (heat - 170U) * 2U));
}

static uint8_t heat_to_blue(uint8_t heat)
{
    if (heat < 85U) {
        return 0;
    }
    if (heat < 170U) {
        return (uint8_t)(255U - (heat - 85U) * 3U);
    }
    return 0;
}

static void hue_to_rgb(uint8_t hue, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    if (hue < 85U) {
        *red = (uint8_t)(255U - hue * 3U);
        *green = (uint8_t)(hue * 3U);
        *blue = 0;
    } else if (hue < 170U) {
        hue = (uint8_t)(hue - 85U);
        *red = 0;
        *green = (uint8_t)(255U - hue * 3U);
        *blue = (uint8_t)(hue * 3U);
    } else {
        hue = (uint8_t)(hue - 170U);
        *red = (uint8_t)(hue * 3U);
        *green = 0;
        *blue = (uint8_t)(255U - hue * 3U);
    }
}

void fire_effect_step(const ThingSpeakData *data)
{
    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        fire_heat[i] = (uint8_t)(fire_heat[i] - (uint8_t)(rand() % ((FIRE_COOLING * 10U / NEOPIXEL_NUM_LEDS) + 2U)));
    }

    for (int k = NEOPIXEL_NUM_LEDS - 1; k >= 2; k--) {
        fire_heat[k] = (uint8_t)((fire_heat[k - 1] + fire_heat[k - 2] + fire_heat[k - 2]) / 3U);
    }

    if ((rand() & 0xFFU) < FIRE_SPARKING) {
        int y = rand() % 7;
        uint8_t spark = (uint8_t)(rand() % 96U + 160U);
        if (fire_heat[y] < 255U - spark) {
            fire_heat[y] = (uint8_t)(fire_heat[y] + spark);
        } else {
            fire_heat[y] = 255U;
        }
    }

    for (int j = 0; j < NEOPIXEL_NUM_LEDS; j++) {
        uint8_t heat = fire_heat[j];
        uint8_t r = heat_to_red(heat);
        uint8_t g = heat_to_green(heat);
        uint8_t b = heat_to_blue(heat);

        r = (uint8_t)((r * data->brightness) / 255U);
        g = (uint8_t)((g * data->brightness) / 255U);
        b = (uint8_t)((b * data->brightness) / 255U);

        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                j,
                r,
                g,
                b
            )
        );
    }

    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel_strip)
    );
}

static void rainbow_effect_step(const ThingSpeakData *data, bool sparkle)
{
    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        hue_to_rgb(
            (uint8_t)(rainbow_hue + (i * 7)),
            &red,
            &green,
            &blue
        );

        red = (uint8_t)((red * data->brightness) / 255U);
        green = (uint8_t)((green * data->brightness) / 255U);
        blue = (uint8_t)((blue * data->brightness) / 255U);

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

    if (sparkle && ((rand() & 0xFFU) < 80U)) {
        int position = rand() % NEOPIXEL_NUM_LEDS;
        uint8_t sparkle_red;
        uint8_t sparkle_green;
        uint8_t sparkle_blue;

        hue_to_rgb(
            (uint8_t)(rainbow_hue + (position * 7)),
            &sparkle_red,
            &sparkle_green,
            &sparkle_blue
        );

        sparkle_red = (uint8_t)((sparkle_red * data->brightness) / 255U);
        sparkle_green = (uint8_t)((sparkle_green * data->brightness) / 255U);
        sparkle_blue = (uint8_t)((sparkle_blue * data->brightness) / 255U);

        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                position,
                add_saturated(sparkle_red, 255U),
                add_saturated(sparkle_green, 255U),
                add_saturated(sparkle_blue, 255U)
            )
        );
    }

    rainbow_hue++;

    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel_strip)
    );
}

static void fade_pixels(
    uint8_t *red,
    uint8_t *green,
    uint8_t *blue,
    uint8_t fade_amount
)
{
    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        red[i] = (uint8_t)((red[i] * (255U - fade_amount)) / 255U);
        green[i] = (uint8_t)((green[i] * (255U - fade_amount)) / 255U);
        blue[i] = (uint8_t)((blue[i] * (255U - fade_amount)) / 255U);
    }
}

static void render_pixels(
    const ThingSpeakData *data,
    const uint8_t *red,
    const uint8_t *green,
    const uint8_t *blue
)
{
    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                i,
                (uint8_t)((red[i] * data->brightness) / 255U),
                (uint8_t)((green[i] * data->brightness) / 255U),
                (uint8_t)((blue[i] * data->brightness) / 255U)
            )
        );
    }

    ESP_ERROR_CHECK(
        led_strip_refresh(neopixel_strip)
    );
}

static void confetti_effect_step(const ThingSpeakData *data)
{
    uint8_t base_hue = rgb_to_hue(data->color1);
    fade_pixels(confetti_red, confetti_green, confetti_blue, 10);

    int position = rand() % NEOPIXEL_NUM_LEDS;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    hsv_to_rgb(
        (uint8_t)(base_hue + (rand() & 0x3FU)),
        200,
        255,
        &red,
        &green,
        &blue
    );
    confetti_red[position] = add_saturated(confetti_red[position], red);
    confetti_green[position] = add_saturated(confetti_green[position], green);
    confetti_blue[position] = add_saturated(confetti_blue[position], blue);

    render_pixels(data, confetti_red, confetti_green, confetti_blue);
}

static void sinelon_effect_step(const ThingSpeakData *data)
{
    uint8_t base_hue = rgb_to_hue(data->color1);
    fade_pixels(sinelon_red, sinelon_green, sinelon_blue, 20);

    float radians = ((float)sinelon_phase / 255.0F) * 6.2831853F;
    int position = (int)(((sinf(radians) + 1.0F) * 0.5F) * (NEOPIXEL_NUM_LEDS - 1));
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    hsv_to_rgb(base_hue, 255, 192, &red, &green, &blue);
    sinelon_red[position] = add_saturated(sinelon_red[position], red);
    sinelon_green[position] = add_saturated(sinelon_green[position], green);
    sinelon_blue[position] = add_saturated(sinelon_blue[position], blue);
    sinelon_phase = (uint8_t)(sinelon_phase + 8U);

    render_pixels(data, sinelon_red, sinelon_green, sinelon_blue);
}

static void bpm_effect_step(const ThingSpeakData *data)
{
    uint8_t base_hue = rgb_to_hue(data->color1);
    uint8_t beat = (uint8_t)(128U + (uint8_t)(sinf(((float)rainbow_hue / 255.0F) * 6.2831853F) * 64.0F));

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        uint8_t value = (uint8_t)(beat - rainbow_hue + (i * 10));
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        hsv_to_rgb((uint8_t)(base_hue + rainbow_hue + (i * 2)), 220, value, &red, &green, &blue);
        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                i,
                (uint8_t)((red * data->brightness) / 255U),
                (uint8_t)((green * data->brightness) / 255U),
                (uint8_t)((blue * data->brightness) / 255U)
            )
        );
    }

    rainbow_hue++;
    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void juggle_effect_step(const ThingSpeakData *data)
{
    uint8_t base_hue = rgb_to_hue(data->color1);
    fade_pixels(juggle_red, juggle_green, juggle_blue, 20);

    for (int dot = 0; dot < 8; dot++) {
        uint8_t phase = (uint8_t)(juggle_phase * (dot + 7));
        float radians = ((float)phase / 255.0F) * 6.2831853F;
        int position = (int)(((sinf(radians) + 1.0F) * 0.5F) * (NEOPIXEL_NUM_LEDS - 1));
        uint8_t red;
        uint8_t green;
        uint8_t blue;

        hsv_to_rgb(
            (uint8_t)(base_hue + (dot * 32)),
            200,
            255,
            &red,
            &green,
            &blue
        );

        juggle_red[position] = add_saturated(juggle_red[position], red);
        juggle_green[position] = add_saturated(juggle_green[position], green);
        juggle_blue[position] = add_saturated(juggle_blue[position], blue);
    }

    juggle_phase = (uint8_t)(juggle_phase + 5U);
    render_pixels(data, juggle_red, juggle_green, juggle_blue);
}

static void fade_effect_step(const ThingSpeakData *data)
{
    uint8_t red = (uint8_t)((data->color1 >> 16) & 0xFFU);
    uint8_t green = (uint8_t)((data->color1 >> 8) & 0xFFU);
    uint8_t blue = (uint8_t)(data->color1 & 0xFFU);

    uint32_t scale = (uint32_t)data->brightness * fade_level;
    red = (uint8_t)(((uint32_t)red * scale) / (255U * 255U));
    green = (uint8_t)(((uint32_t)green * scale) / (255U * 255U));
    blue = (uint8_t)(((uint32_t)blue * scale) / (255U * 255U));

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

    if (fade_rising) {
        if (fade_level >= 250U) {
            fade_level = 255U;
            fade_rising = false;
        } else {
            fade_level = (uint8_t)(fade_level + 5U);
        }
    } else if (fade_level <= 5U) {
        fade_level = 0;
        fade_rising = true;
    } else {
        fade_level = (uint8_t)(fade_level - 5U);
    }

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void clear_flare_pixels()
{
    ESP_ERROR_CHECK(led_strip_clear(neopixel_strip));
}

static void start_flare()
{
    flare_position = 0.0F;
    flare_velocity = (float)(50 + (rand() % 41)) / 100.0F;
    flare_brightness = 1.0F;
    flare_exploding = false;

    for (int i = 0; i < FLARE_SPARK_COUNT; i++) {
        flare_spark_position[i] = 0.0F;
        flare_spark_velocity[i] = ((float)(rand() & 0xFFU) / 255.0F) * (flare_velocity / 5.0F);
        flare_spark_brightness[i] = flare_spark_velocity[i] * 1000.0F;
        if (flare_spark_brightness[i] > 255.0F) {
            flare_spark_brightness[i] = 255.0F;
        }
    }
}

static void start_explosion()
{
    explosion_spark_count = (int)(flare_position / 2.0F);
    if (explosion_spark_count < 1) {
        explosion_spark_count = 1;
    }
    if (explosion_spark_count > EXPLOSION_SPARK_COUNT) {
        explosion_spark_count = EXPLOSION_SPARK_COUNT;
    }

    for (int i = 0; i < explosion_spark_count; i++) {
        explosion_spark_position[i] = flare_position;
        explosion_spark_velocity[i] = ((float)(rand() % 20001) / 10000.0F) - 1.0F;
        explosion_spark_brightness[i] = fabsf(explosion_spark_velocity[i]) * 500.0F;
        if (explosion_spark_brightness[i] > 255.0F) {
            explosion_spark_brightness[i] = 255.0F;
        }
    }

    flare_exploding = true;
}

static void flare_effect_step(const ThingSpeakData *data)
{
    if (flare_velocity == 0.0F && !flare_exploding) {
        start_flare();
    }

    clear_flare_pixels();

    if (!flare_exploding) {
        for (int i = 0; i < FLARE_SPARK_COUNT; i++) {
            flare_spark_position[i] += flare_spark_velocity[i];
            flare_spark_velocity[i] += FLARE_GRAVITY;
            flare_spark_brightness[i] -= 0.8F;

            set_scaled_color(
                (int)flare_spark_position[i],
                data->color2,
                data->brightness,
                (uint8_t)(flare_spark_brightness[i] > 0.0F ? flare_spark_brightness[i] : 0.0F)
            );
        }

        set_scaled_color(
            (int)flare_position,
            data->color1,
            data->brightness,
            (uint8_t)(flare_brightness * 255.0F)
        );

        flare_position += flare_velocity;
        flare_velocity += FLARE_GRAVITY;
        flare_brightness *= 0.985F;

        if (flare_velocity < -0.2F) {
            start_explosion();
        }
    } else {
        bool sparks_alive = false;

        for (int i = 0; i < explosion_spark_count; i++) {
            explosion_spark_position[i] += explosion_spark_velocity[i];
            explosion_spark_velocity[i] += FLARE_GRAVITY;
            explosion_spark_brightness[i] *= 0.99F;

            if (explosion_spark_brightness[i] > 1.0F) {
                sparks_alive = true;
            }

            set_scaled_color(
                (int)explosion_spark_position[i],
                data->color2,
                data->brightness,
                (uint8_t)(explosion_spark_brightness[i] > 0.0F ? explosion_spark_brightness[i] : 0.0F)
            );
        }

        if (!sparks_alive) {
            start_flare();
        }
    }

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

void apply_effect(const ThingSpeakData *data)
{
    switch (data->pattern) {
        case 2:
            fire_effect_step(data);
            break;

        case 3:
            rainbow_effect_step(data, false);
            break;

        case 4:
            rainbow_effect_step(data, true);
            break;

        case 5:
            confetti_effect_step(data);
            break;

        case 6:
            sinelon_effect_step(data);
            break;

        case 7:
            bpm_effect_step(data);
            break;

        case 8:
            juggle_effect_step(data);
            break;

        case 9:
            fade_effect_step(data);
            break;

        case 10:
            flare_effect_step(data);
            break;

        default:
            break;
    }
}
