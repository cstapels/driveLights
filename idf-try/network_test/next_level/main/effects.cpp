#include "effects.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

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
static uint8_t mesh_fade_phase = 0;
static uint8_t candy_cane_offset = 0;
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
static float center_glow_phase = 0.0F;
static float ocean_swell_phase = 0.0F;
static float color_drift_phase = 0.0F;
static float northern_lights_phase = 0.0F;
static uint8_t ember_glow[NEOPIXEL_NUM_LEDS];
typedef enum {
    RAIN_DIR_DOWN,
    RAIN_DIR_UP
} RainDirection;

#define MAX_RAIN_DROPS 10
#define TOTAL_RAIN_DROPS_PER_DIR 10

typedef struct {
    bool active;
    int pos;         // leading pixel position
    int size;        // 1 or 2 pixels
} RainDrop;

static bool rain_initialized = false;
static RainDirection rain_direction = RAIN_DIR_DOWN;
static RainDrop rain_drops[MAX_RAIN_DROPS];
static int rain_drops_spawned = 0;
static int rain_drops_landed = 0;
static int rain_pile_height = 0;
static int rain_spawn_timer = 0;
static int rain_frame_counter = 0;
static const int RAIN_FRAMES_PER_STEP = 2; // lower = faster fall
static const int RAIN_SPAWN_INTERVAL = 14; // steps between drop spawns
//three color  fade
typedef enum {
    FADE_TO_COLOR,
    FADE_TO_BLACK
} FadePhase;

static bool fade_initialized = false;
static FadePhase fade_phase = FADE_TO_COLOR;
static int fade_color_index = 0;
static uint8_t fade_brightness = 0;      // 0-255
static int fade_frame_counter = 0;

static const int FADE_STEP_SIZE = 3;          // brightness change per tick
static const int FADE_FRAMES_PER_STEP = 2;    // lower = faster fade

void reset_effects()
{
    memset(fire_heat, 0, sizeof(fire_heat));
    rainbow_hue = 0;
    memset(confetti_red, 0, sizeof(confetti_red));
    memset(confetti_green, 0, sizeof(confetti_green));
    memset(confetti_blue, 0, sizeof(confetti_blue));
    memset(sinelon_red, 0, sizeof(sinelon_red));
    memset(sinelon_green, 0, sizeof(sinelon_green));
    memset(sinelon_blue, 0, sizeof(sinelon_blue));
    sinelon_phase = 0;
    memset(juggle_red, 0, sizeof(juggle_red));
    memset(juggle_green, 0, sizeof(juggle_green));
    memset(juggle_blue, 0, sizeof(juggle_blue));
    juggle_phase = 0;
    fade_level = 0;
    fade_rising = true;
    mesh_fade_phase = 0;
    candy_cane_offset = 0;
    flare_position = 0.0F;
    flare_velocity = 0.0F;
    flare_brightness = 1.0F;
    memset(flare_spark_position, 0, sizeof(flare_spark_position));
    memset(flare_spark_velocity, 0, sizeof(flare_spark_velocity));
    memset(flare_spark_brightness, 0, sizeof(flare_spark_brightness));
    memset(explosion_spark_position, 0, sizeof(explosion_spark_position));
    memset(explosion_spark_velocity, 0, sizeof(explosion_spark_velocity));
    memset(explosion_spark_brightness, 0, sizeof(explosion_spark_brightness));
    explosion_spark_count = 0;
    flare_exploding = false;
    center_glow_phase = 0.0F;
    ocean_swell_phase = 0.0F;
    color_drift_phase = 0.0F;
    northern_lights_phase = 0.0F;
    memset(ember_glow, 0, sizeof(ember_glow));
    rain_initialized = false;
    memset(rain_drops, 0, sizeof(rain_drops));
    rain_drops_spawned = 0;
    rain_drops_landed = 0;
    rain_pile_height = 0;
    rain_spawn_timer = 0;
    rain_frame_counter = 0;
    fade_initialized = false;
}

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
    // Fade the existing trail pixels down
    fade_pixels(sinelon_red, sinelon_green, sinelon_blue, 30);

    // Calculate fractional dot position across the strip
    float radians = ((float)sinelon_phase / 255.0F) * 6.2831853F;
    float pos = ((sinf(radians) + 1.0F) * 0.5F) * (float)(NEOPIXEL_NUM_LEDS - 1);

    // Get RGB components from color1 (or rainbow if color1 is 0)
    uint8_t red, green, blue;
    if (data->color1 != 0) {
        red = (uint8_t)((data->color1 >> 16) & 0xFFU);
        green = (uint8_t)((data->color1 >> 8) & 0xFFU);
        blue = (uint8_t)(data->color1 & 0xFFU);
    } else {
        hsv_to_rgb(sinelon_phase, 255, 255, &red, &green, &blue);
    }

    // Anti-aliased / multi-pixel dot centered at `pos` (width ~ 3 pixels)
    int center = (int)pos;
    for (int offset = -2; offset <= 2; offset++) {
        int idx = center + offset;
        if (idx >= 0 && idx < NEOPIXEL_NUM_LEDS) {
            float dist = fabsf((float)idx - pos);
            if (dist < 2.0F) {
                // Smooth bell-curve intensity dropoff from center
                float intensity = 1.0F - (dist * 0.5F);
                uint8_t r = (uint8_t)((float)red * intensity);
                uint8_t g = (uint8_t)((float)green * intensity);
                uint8_t b = (uint8_t)((float)blue * intensity);

                sinelon_red[idx] = add_saturated(sinelon_red[idx], r);
                sinelon_green[idx] = add_saturated(sinelon_green[idx], g);
                sinelon_blue[idx] = add_saturated(sinelon_blue[idx], b);
            }
        }
    }

    sinelon_phase = (uint8_t)(sinelon_phase + 4U);

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

static void mesh_fade_effect_step(const ThingSpeakData *data)
{
    uint8_t red = (uint8_t)((data->color1 >> 16) & 0xFFU);
    uint8_t green = (uint8_t)((data->color1 >> 8) & 0xFFU);
    uint8_t blue = (uint8_t)(data->color1 & 0xFFU);

    // Calculate phase offset based on DEVICE_ID across the mesh
    // Distribute 256 units of phase across NUM_DEVICES
    uint8_t device_phase_offset = (uint8_t)(((uint32_t)DEVICE_ID * 256U) / (NUM_DEVICES > 0 ? NUM_DEVICES : 1));
    uint8_t current_phase = (uint8_t)(mesh_fade_phase - device_phase_offset);

    // Sine wave pulsation: sinf ranges [-1, 1] -> [0, 1] -> [0, 255]
    float radians = ((float)current_phase / 255.0F) * 6.2831853F;
    float sin_norm = (sinf(radians) + 1.0F) * 0.5F; // 0.0 to 1.0
    uint8_t current_fade = (uint8_t)(sin_norm * 255.0F);

    uint32_t scale = (uint32_t)data->brightness * current_fade;
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

    // Step the global phase forward each frame
    mesh_fade_phase = (uint8_t)(mesh_fade_phase + 4U);

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void mesh_strip_fade_effect_step(const ThingSpeakData *data)
{
    uint8_t base_red = (uint8_t)((data->color1 >> 16) & 0xFFU);
    uint8_t base_green = (uint8_t)((data->color1 >> 8) & 0xFFU);
    uint8_t base_blue = (uint8_t)(data->color1 & 0xFFU);

    int total_leds = (NUM_DEVICES > 0 ? NUM_DEVICES : 1) * NEOPIXEL_NUM_LEDS;

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        int global_index = DEVICE_ID * NEOPIXEL_NUM_LEDS + i;
        uint8_t offset = (uint8_t)(((uint32_t)global_index * 256U) / (uint32_t)total_leds);
        uint8_t current_phase = (uint8_t)(mesh_fade_phase - offset);

        float radians = ((float)current_phase / 255.0F) * 6.2831853F;
        float sin_norm = (sinf(radians) + 1.0F) * 0.5F;
        uint8_t current_fade = (uint8_t)(sin_norm * 255.0F);

        uint32_t scale = (uint32_t)data->brightness * current_fade;
        uint8_t red = (uint8_t)(((uint32_t)base_red * scale) / (255U * 255U));
        uint8_t green = (uint8_t)(((uint32_t)base_green * scale) / (255U * 255U));
        uint8_t blue = (uint8_t)(((uint32_t)base_blue * scale) / (255U * 255U));

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

    mesh_fade_phase = (uint8_t)(mesh_fade_phase + 4U);

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void mesh_rainbow_solid_effect_step(const ThingSpeakData *data)
{
    // Each device is rendered as a solid color from the rainbow
    uint8_t device_hue_offset = (uint8_t)(((uint32_t)DEVICE_ID * 256U) / (NUM_DEVICES > 0 ? NUM_DEVICES : 1));
    uint8_t hue = (uint8_t)(rainbow_hue + device_hue_offset);

    uint8_t red, green, blue;
    hue_to_rgb(hue, &red, &green, &blue);

    red = (uint8_t)((red * data->brightness) / 255U);
    green = (uint8_t)((green * data->brightness) / 255U);
    blue = (uint8_t)((blue * data->brightness) / 255U);

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

    rainbow_hue++;

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void center_glow_effect_step(const ThingSpeakData *data)
{
    float pulse = (sinf(center_glow_phase) + 1.0F) * 0.5F;
    float center = ((float)NEOPIXEL_NUM_LEDS - 1.0F) * 0.5F;
    float sigma = 5.0F + (pulse * 24.0F);

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        float distance = fabsf((float)i - center);
        float normalized_distance = distance / sigma;
        float intensity = expf(-(normalized_distance * normalized_distance) * 0.5F);
        intensity *= 0.25F + (pulse * 0.75F);
        set_scaled_color(i, data->color1, data->brightness, (uint8_t)(intensity * 255.0F));
    }

    center_glow_phase += 0.025F;
    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void ocean_swell_effect_step(const ThingSpeakData *data)
{
    uint8_t color1_red = (uint8_t)((data->color1 >> 16) & 0xFFU);
    uint8_t color1_green = (uint8_t)((data->color1 >> 8) & 0xFFU);
    uint8_t color1_blue = (uint8_t)(data->color1 & 0xFFU);
    uint8_t color2_red = (uint8_t)((data->color2 >> 16) & 0xFFU);
    uint8_t color2_green = (uint8_t)((data->color2 >> 8) & 0xFFU);
    uint8_t color2_blue = (uint8_t)(data->color2 & 0xFFU);

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        float wave = (sinf((float)i * 0.10F + ocean_swell_phase) + 1.0F) * 0.5F;
        wave *= 0.45F;
        uint8_t red = (uint8_t)(color1_red + ((color2_red - color1_red) * wave));
        uint8_t green = (uint8_t)(color1_green + ((color2_green - color1_green) * wave));
        uint8_t blue = (uint8_t)(color1_blue + ((color2_blue - color1_blue) * wave));

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

    ocean_swell_phase += 0.018F;
    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void color_drift_effect_step(const ThingSpeakData *data)
{
    uint8_t base_hue = rgb_to_hue(data->color1);

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        uint8_t hue = (uint8_t)(base_hue + color_drift_phase + ((float)i * 0.25F));
        uint8_t red, green, blue;
        hsv_to_rgb(hue, 175, 255, &red, &green, &blue);

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

    color_drift_phase += 0.8F;
    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void northern_lights_effect_step(const ThingSpeakData *data)
{
    uint8_t colors[3][3] = {
        {(uint8_t)((data->color1 >> 16) & 0xFFU), (uint8_t)((data->color1 >> 8) & 0xFFU), (uint8_t)(data->color1 & 0xFFU)},
        {(uint8_t)((data->color2 >> 16) & 0xFFU), (uint8_t)((data->color2 >> 8) & 0xFFU), (uint8_t)(data->color2 & 0xFFU)},
        {(uint8_t)((data->color3 >> 16) & 0xFFU), (uint8_t)((data->color3 >> 8) & 0xFFU), (uint8_t)(data->color3 & 0xFFU)}
    };

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        float band1 = (sinf((float)i * 0.075F + northern_lights_phase) + 1.0F) * 0.5F;
        float band2 = (sinf((float)i * 0.045F - northern_lights_phase * 0.7F + 2.0F) + 1.0F) * 0.5F;
        float mix = band1 * 0.55F + band2 * 0.45F;
        float level = 0.18F + (mix * 0.62F);
        uint8_t red = (uint8_t)((colors[0][0] * (1.0F - mix)) + (colors[1][0] * mix));
        uint8_t green = (uint8_t)((colors[0][1] * (1.0F - mix)) + (colors[1][1] * mix));
        uint8_t blue = (uint8_t)((colors[0][2] * (1.0F - mix)) + (colors[1][2] * mix));

        if (band2 > 0.65F) {
            red = (uint8_t)((red * 0.65F) + (colors[2][0] * 0.35F));
            green = (uint8_t)((green * 0.65F) + (colors[2][1] * 0.35F));
            blue = (uint8_t)((blue * 0.65F) + (colors[2][2] * 0.35F));
        }

        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                i,
                (uint8_t)((red * data->brightness * level) / 255.0F),
                (uint8_t)((green * data->brightness * level) / 255.0F),
                (uint8_t)((blue * data->brightness * level) / 255.0F)
            )
        );
    }

    northern_lights_phase += 0.012F;
    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void ember_glow_effect_step(const ThingSpeakData *data)
{
    uint32_t color = data->color1 != 0 ? data->color1 : 0xFF3A08U;
    uint8_t red = (uint8_t)((color >> 16) & 0xFFU);
    uint8_t green = (uint8_t)((color >> 8) & 0xFFU);
    uint8_t blue = (uint8_t)(color & 0xFFU);

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        ember_glow[i] = (uint8_t)(((uint16_t)ember_glow[i] * 247U) / 255U);
        if ((rand() & 0xFFU) < 18U) {
            ember_glow[i] = add_saturated(ember_glow[i], (uint8_t)(80U + (rand() % 176U)));
        }

        uint32_t scale = (uint32_t)data->brightness * ember_glow[i];
        ESP_ERROR_CHECK(
            led_strip_set_pixel(
                neopixel_strip,
                i,
                (uint8_t)((red * scale) / (255U * 255U)),
                (uint8_t)((green * scale) / (255U * 255U)),
                (uint8_t)((blue * scale) / (255U * 255U))
            )
        );
    }

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void clear_flare_pixels()
{
    if (neopixel_strip) {
        for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
            led_strip_set_pixel(neopixel_strip, i, 0, 0, 0);
        }
    }
}

static void start_flare()
{
    flare_position = 0.0F;
    flare_velocity = (float)(75 + (rand() % 26)) / 100.0F;
    flare_brightness = 1.0F;
    flare_exploding = false;

    for (int i = 0; i < FLARE_SPARK_COUNT; i++) {
        flare_spark_position[i] = 0.0F;
        flare_spark_velocity[i] = ((float)(rand() & 0xFFU) / 255.0F) * (flare_velocity / 8.0F);
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
            flare_spark_brightness[i] -= 2.0F;

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

        if (flare_velocity < 0.0F) {
            start_explosion();
        }
    } else {
        bool sparks_alive = false;

        for (int i = 0; i < explosion_spark_count; i++) {
            explosion_spark_position[i] += explosion_spark_velocity[i];
            explosion_spark_velocity[i] += FLARE_GRAVITY;
            explosion_spark_brightness[i] *= 0.95F;

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

static void candy_cane_effect_step(const ThingSpeakData *data, bool use_custom_colors)
{
    // Alternate stripes of width 4 (stripe cycle length = 8)
    const int stripe_width = 4;
    const int cycle_len = stripe_width * 2;

    uint8_t c1_red, c1_green, c1_blue;
    uint8_t c2_red, c2_green, c2_blue;

    if (use_custom_colors) {
        c1_red = (uint8_t)((data->color1 >> 16) & 0xFFU);
        c1_green = (uint8_t)((data->color1 >> 8) & 0xFFU);
        c1_blue = (uint8_t)(data->color1 & 0xFFU);

        c2_red = (uint8_t)((data->color2 >> 16) & 0xFFU);
        c2_green = (uint8_t)((data->color2 >> 8) & 0xFFU);
        c2_blue = (uint8_t)(data->color2 & 0xFFU);
    } else {
        // Red
        c1_red = 255;
        c1_green = 0;
        c1_blue = 0;

        // White
        c2_red = 255;
        c2_green = 255;
        c2_blue = 255;
    }

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
        int pos = (i + candy_cane_offset) % cycle_len;
        uint8_t red, green, blue;

        if (pos < stripe_width) {
            red = c1_red;
            green = c1_green;
            blue = c1_blue;
        } else {
            red = c2_red;
            green = c2_green;
            blue = c2_blue;
        }

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

    candy_cane_offset = (uint8_t)((candy_cane_offset + 1) % cycle_len);

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

static void rain_reset_dir() {
    memset(rain_drops, 0, sizeof(rain_drops));
    rain_drops_spawned = 0;
    rain_drops_landed = 0;
    rain_pile_height = 0;
    rain_spawn_timer = RAIN_SPAWN_INTERVAL; // spawn first drop immediately
    rain_frame_counter = 0;
}

void three_color_effect_step(const ThingSpeakData *data) {
    if (!fade_initialized) {
        fade_phase = FADE_TO_COLOR;
        fade_color_index = 0;
        fade_brightness = 0;
        fade_frame_counter = 0;
        fade_initialized = true;
    }

    // Throttle fade speed
    fade_frame_counter++;
    if (fade_frame_counter < FADE_FRAMES_PER_STEP) {
        return;
    }
    fade_frame_counter = 0;

    uint32_t color = data->color1;
    if (fade_color_index == 1) {
        color = data->color2;
    } else if (fade_color_index == 2) {
        color = data->color3;
    }

    uint32_t scale = (uint32_t)data->brightness * fade_brightness;
    uint8_t cur_r = (uint8_t)((((color >> 16) & 0xFFU) * scale) / (255U * 255U));
    uint8_t cur_g = (uint8_t)((((color >> 8) & 0xFFU) * scale) / (255U * 255U));
    uint8_t cur_b = (uint8_t)(((color & 0xFFU) * scale) / (255U * 255U));

    for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {

         ESP_ERROR_CHECK(led_strip_set_pixel(neopixel_strip, i, cur_r, cur_g, cur_b));

    }

    if (fade_phase == FADE_TO_COLOR) {
        if (fade_brightness >= 255 - FADE_STEP_SIZE) {
            fade_brightness = 255;
            fade_phase = FADE_TO_BLACK;
        } else {
            fade_brightness += FADE_STEP_SIZE;
        }
    } else { // FADE_TO_BLACK
        if (fade_brightness <= FADE_STEP_SIZE) {
            fade_brightness = 0;
            fade_color_index = (fade_color_index + 1) % 3;
            fade_phase = FADE_TO_COLOR;
        } else {
            fade_brightness -= FADE_STEP_SIZE;
        }
    }

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}

void rain_effect_step(const ThingSpeakData *data) {
    uint8_t c_red = (uint8_t)((data->color1 >> 16) & 0xFFU);
    uint8_t c_green = (uint8_t)((data->color1 >> 8) & 0xFFU);
    uint8_t c_blue = (uint8_t)(data->color1 & 0xFFU);

    if (c_red == 0 && c_green == 0 && c_blue == 0) {
        // Default rain color (blue/cyan) if color1 is not specified
        c_red = 0;
        c_green = 120;
        c_blue = 255;
    }

    if (!rain_initialized) {
        rain_direction = RAIN_DIR_DOWN;
        rain_reset_dir();
        rain_initialized = true;
    }

    // Throttle fall speed
    rain_frame_counter++;
    if (rain_frame_counter < RAIN_FRAMES_PER_STEP) {
        return;
    }
    rain_frame_counter = 0;

    // Clear strip
    ESP_ERROR_CHECK(led_strip_clear(neopixel_strip));

    // Determine pile bounds
    int pile_start, pile_end;
    if (rain_direction == RAIN_DIR_DOWN) {
        pile_start = NEOPIXEL_NUM_LEDS - rain_pile_height;
        pile_end = NEOPIXEL_NUM_LEDS - 1;
    } else {
        pile_start = 0;
        pile_end = rain_pile_height - 1;
    }

    uint8_t render_r = (uint8_t)((c_red * data->brightness) / 255U);
    uint8_t render_g = (uint8_t)((c_green * data->brightness) / 255U);
    uint8_t render_b = (uint8_t)((c_blue * data->brightness) / 255U);

    // Render pile at target end
    for (int i = pile_start; i <= pile_end; i++) {
        if (i >= 0 && i < NEOPIXEL_NUM_LEDS) {
            ESP_ERROR_CHECK(led_strip_set_pixel(neopixel_strip, i, render_r, render_g, render_b));
        }
    }

    // Spawn new drops up to TOTAL_RAIN_DROPS_PER_DIR (10)
    if (rain_drops_spawned < TOTAL_RAIN_DROPS_PER_DIR) {
        rain_spawn_timer++;
        if (rain_spawn_timer >= RAIN_SPAWN_INTERVAL) {
            for (int d = 0; d < MAX_RAIN_DROPS; d++) {
                if (!rain_drops[d].active) {
                    rain_drops[d].active = true;
                    rain_drops[d].size = 1 + (rand() % 2); // 1 or 2 pixels
                    rain_drops[d].pos = (rain_direction == RAIN_DIR_DOWN) ? 0 : (NEOPIXEL_NUM_LEDS - 1);
                    rain_drops_spawned++;
                    rain_spawn_timer = 0;
                    break;
                }
            }
        }
    }

    // Move and render active drops
    int step = (rain_direction == RAIN_DIR_DOWN) ? 1 : -1;
    int active_count = 0;

    for (int d = 0; d < MAX_RAIN_DROPS; d++) {
        if (!rain_drops[d].active) {
            continue;
        }

        active_count++;

        // Render drop pixels
        for (int j = 0; j < rain_drops[d].size; j++) {
            int idx = rain_drops[d].pos - (j * step);
            if (idx >= 0 && idx < NEOPIXEL_NUM_LEDS) {
                bool in_pile = (rain_direction == RAIN_DIR_DOWN) ? (idx >= pile_start) : (idx <= pile_end);
                if (!in_pile) {
                    ESP_ERROR_CHECK(led_strip_set_pixel(neopixel_strip, idx, render_r, render_g, render_b));
                }
            }
        }

        // Advance drop position
        rain_drops[d].pos += step;

        // Check if drop reached target pile/end
        bool hit_end = (rain_direction == RAIN_DIR_DOWN)
            ? (rain_drops[d].pos >= pile_start)
            : (rain_drops[d].pos <= pile_end);

        if (hit_end) {
            rain_drops[d].active = false;
            rain_pile_height += rain_drops[d].size;
            if (rain_pile_height > NEOPIXEL_NUM_LEDS) {
                rain_pile_height = NEOPIXEL_NUM_LEDS;
            }
            rain_drops_landed++;
        }
    }

    // After 10 drops are spawned and all active drops have landed, switch direction
    if (rain_drops_spawned >= TOTAL_RAIN_DROPS_PER_DIR && active_count == 0) {
        rain_direction = (rain_direction == RAIN_DIR_DOWN) ? RAIN_DIR_UP : RAIN_DIR_DOWN;
        rain_reset_dir();
    }

    ESP_ERROR_CHECK(led_strip_refresh(neopixel_strip));
}



void apply_effect(const ThingSpeakData *data)
{
    switch (data->pattern) {
        case 1:
            fire_effect_step(data);
            break;

        case 2:
            rainbow_effect_step(data, false);
            break;

        case 3:
            rainbow_effect_step(data, true);
            break;

        case 4:
            confetti_effect_step(data);
            break;

        case 5:
            sinelon_effect_step(data);
            break;

        case 6:
            bpm_effect_step(data);
            break;

        case 7:
            juggle_effect_step(data);
            break;

        case 8:
            fade_effect_step(data);
            break;

        case 9:
            flare_effect_step(data);
            break;

        case 10:
            candy_cane_effect_step(data, false);
            break;

        case 11:
            candy_cane_effect_step(data, true);
            break;

        case 12:
            mesh_fade_effect_step(data);
            break;

        case 13:
            mesh_strip_fade_effect_step(data);
            break;

        case 14:
            mesh_rainbow_solid_effect_step(data);
            break;

        case 15:
            three_color_effect_step(data);
            break;

        case 16:
            rain_effect_step(data);
            break;

        case 17:
            center_glow_effect_step(data);
            break;

        case 18:
            ocean_swell_effect_step(data);
            break;

        case 19:
            color_drift_effect_step(data);
            break;

        case 20:
            northern_lights_effect_step(data);
            break;

        case 21:
            ember_glow_effect_step(data);
            break;

        default:
            break;
    }
}
