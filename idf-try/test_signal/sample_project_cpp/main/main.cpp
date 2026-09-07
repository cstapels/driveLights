#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#define LED_GPIO GPIO_NUM_2

// ============================================================
// CHANGE THIS FOR EACH BOARD
// ============================================================

// Board 1: Put Board 2's MAC address here
//uint8_t peer_mac[] = {0xE4,0x65,0xB8,0x78,0x9C,0x20};
uint8_t peer_mac[] = {0xE0,0x5A,0x1B,0x75,0x57,0xE4}; //num4

// Board 2: Put Board 1's MAC address here
// uint8_t peer_mac[] = {

//      0xC8,0xF0,0x9E,0x2E,0x08,0xBC
// };


// ============================================================
// MESSAGE STRUCTURE
// ============================================================

typedef struct {
    char message[16];
} espnow_message_t;


// ============================================================
// FLASH LED TWICE
// ============================================================

void flash_led_twice()
{
    for (int i = 0; i < 2; i++) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(150));

        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}


// ============================================================
// ESP-NOW RECEIVE CALLBACK
// ============================================================

void on_data_recv(const esp_now_recv_info_t *recv_info,
                  const uint8_t *data,
                  int len)
{
    if (len != sizeof(espnow_message_t)) {
        printf("Received unexpected message size: %d\n", len);
        return;
    }

    espnow_message_t msg;
    memcpy(&msg, data, sizeof(msg));

    printf("Received: %s\n", msg.message);

    // Flash LED twice when a message is received
    flash_led_twice();
}


// ============================================================
// SEND MESSAGE
// ============================================================

void send_ping()
{
    espnow_message_t msg;

    strcpy(msg.message, "PING");

    esp_err_t result = esp_now_send(
        peer_mac,
        (uint8_t *)&msg,
        sizeof(msg)
    );

    if (result == ESP_OK) {
        printf("PING sent\n");
    } else {
        printf("Error sending PING: %s\n",
               esp_err_to_name(result));
    }
}


// ============================================================
// MAIN
// ============================================================

extern "C" void app_main(void)
{
    // --------------------------------------------------------
    // Initialize NVS
    // --------------------------------------------------------

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    // --------------------------------------------------------
    // Initialize Wi-Fi
    // --------------------------------------------------------

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA)
    );

    ESP_ERROR_CHECK(
        esp_wifi_start()
    );


    // --------------------------------------------------------
    // Print our MAC address
    // --------------------------------------------------------

    uint8_t my_mac[6];

    ESP_ERROR_CHECK(
        esp_read_mac(my_mac, ESP_MAC_WIFI_STA)
    );

    printf("\n");
    printf("====================================\n");
    printf("ESP-NOW TEST STARTING\n");
    printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           my_mac[0], my_mac[1], my_mac[2],
           my_mac[3], my_mac[4], my_mac[5]);
    printf("====================================\n");


    // --------------------------------------------------------
    // Initialize LED
    // --------------------------------------------------------

    gpio_reset_pin(LED_GPIO);

    ESP_ERROR_CHECK(
        gpio_set_direction(
            LED_GPIO,
            GPIO_MODE_OUTPUT
        )
    );

    gpio_set_level(LED_GPIO, 0);


    // --------------------------------------------------------
    // Initialize ESP-NOW
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        esp_now_init()
    );


    // Register receive callback
    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(on_data_recv)
    );


    // --------------------------------------------------------
    // Add the other ESP32 as a peer
    // --------------------------------------------------------

    esp_now_peer_info_t peer_info = {};

    memcpy(
        peer_info.peer_addr,
        peer_mac,
        ESP_NOW_ETH_ALEN
    );

    peer_info.channel = 0;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(
        esp_now_add_peer(&peer_info)
    );


    printf("ESP-NOW initialized\n");
    printf("Sending PING every 2 seconds...\n\n");


    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    while (1) {

        send_ping();

        vTaskDelay(
            pdMS_TO_TICKS(2000)
        );
    }
}