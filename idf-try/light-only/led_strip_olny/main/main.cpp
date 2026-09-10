#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

namespace {

constexpr const char* kTag = "LED_STRIP";
constexpr gpio_num_t kLedDataPin = GPIO_NUM_4;
constexpr gpio_num_t kLedPowerPin = GPIO_NUM_27;
constexpr uint8_t kBrightness = 128;
constexpr TickType_t kColorTime = pdMS_TO_TICKS(5000);
constexpr TickType_t kOffTime = pdMS_TO_TICKS(5000);
constexpr int kLedCount = 60;

void set_color(led_strip_handle_t strip, uint8_t red, uint8_t green, uint8_t blue)
{
	const uint8_t scale = kBrightness;
	const uint8_t scaled_red = static_cast<uint8_t>((static_cast<uint16_t>(red) * scale) / 255);
	const uint8_t scaled_green = static_cast<uint8_t>((static_cast<uint16_t>(green) * scale) / 255);
	const uint8_t scaled_blue = static_cast<uint8_t>((static_cast<uint16_t>(blue) * scale) / 255);
	for (int pixel = 0; pixel < kLedCount; ++pixel) {
		ESP_ERROR_CHECK(led_strip_set_pixel(strip, pixel, scaled_red, scaled_green, scaled_blue));
	}
	ESP_ERROR_CHECK(led_strip_refresh(strip));
}

}  // namespace

extern "C" void app_main(void)
{
	ESP_LOGI(kTag, "Starting LED strip controller: %d LEDs, data GPIO %d, power GPIO %d",
		 kLedCount, kLedDataPin, kLedPowerPin);

	gpio_config_t power_config{};
	power_config.pin_bit_mask = 1ULL << kLedPowerPin;
	power_config.mode = GPIO_MODE_OUTPUT;
	power_config.pull_up_en = GPIO_PULLUP_DISABLE;
	power_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
	power_config.intr_type = GPIO_INTR_DISABLE;
	ESP_ERROR_CHECK(gpio_config(&power_config));

	// Enable strip power before sending any LED data.
	ESP_LOGI(kTag, "Enabling LED strip power on GPIO %d", kLedPowerPin);
	ESP_ERROR_CHECK(gpio_set_level(kLedPowerPin, 1));

	led_strip_config_t strip_config{};
	strip_config.strip_gpio_num = kLedDataPin;
	strip_config.max_leds = kLedCount;
	strip_config.led_model = LED_MODEL_WS2812;
	strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB;
	strip_config.flags.invert_out = false;

	led_strip_rmt_config_t rmt_config{};
	rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
	rmt_config.resolution_hz = 10 * 1000 * 1000;
	rmt_config.mem_block_symbols = 64;
	rmt_config.flags.with_dma = false;

	led_strip_handle_t strip = nullptr;
	ESP_LOGI(kTag, "Initializing WS2812 strip on GPIO %d at %d%% brightness",
		 kLedDataPin, (kBrightness * 100) / 255);
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
	ESP_ERROR_CHECK(led_strip_clear(strip));
	ESP_ERROR_CHECK(led_strip_refresh(strip));
	ESP_LOGI(kTag, "Strip initialized; starting color cycle");

	while (true) {
		ESP_LOGI(kTag, "Displaying RED for 5 seconds");
		set_color(strip, 255, 0, 0);
		vTaskDelay(kColorTime);

		ESP_LOGI(kTag, "Displaying GREEN for 5 seconds");
		set_color(strip, 0, 255, 0);
		vTaskDelay(kColorTime);

		ESP_LOGI(kTag, "Displaying BLUE for 5 seconds");
		set_color(strip, 0, 0, 255);
		vTaskDelay(kColorTime);

		ESP_LOGI(kTag, "Turning strip OFF for 5 seconds");
		set_color(strip, 0, 0, 0);
		vTaskDelay(kOffTime);
	}

}
