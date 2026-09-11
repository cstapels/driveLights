#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "led_strip.h"

// ============================================================
// CONFIGURATION
// ============================================================

// Change this for each ESP32
//
// Head device = 0
// Other devices = 1, 2, 3, etc.
//
#define DEVICE_ID 0
//0 is com 13 now says 3 on it
//1 is com9 swirly
//4 device 2 on com 12
//6? is device 3 on com11

#define HEAD_ID 0

#define NUM_DEVICES 5
#define MAX_DEVICES 10

// LED
#define LED_GPIO GPIO_NUM_2
#define POWER_SWITCH_GPIO GPIO_NUM_26
#define NEOPIXEL_NUM_LEDS 150
#define NEOPIXEL_DATA_PIN GPIO_NUM_4
#define NEOPIXEL_POWER_PIN GPIO_NUM_27
#define STATUS_POWER_HOLD_MS 6000

// Mesh
#define DEFAULT_TTL 10
#define MAX_SEEN_MESSAGES 50
#define BROADCAST_ID 255

// Join beacon timing
#define JOIN_BEACON_INTERVAL_MS 500

// How long a remote scans each WiFi channel
#define CHANNEL_SCAN_TIME_MS 150

// Number of 2.4 GHz channels to scan
#define FIRST_WIFI_CHANNEL 1
#define LAST_WIFI_CHANNEL 11

// Discovery timing
#define DISCOVERY_INTERVAL_MS 1000
#define REMOTE_DISCOVERY_DURATION_MS 10000
#define UPDATE_RETRY_INTERVAL_MS 2000
#define MAX_UPDATE_RETRIES 3

// ThingSpeak
#define THINGSPEAK_CHANNEL_ID 2060365
#define STATUS_THINGSPEAK_CHANNEL_ID 364593
#define STATUS_THINGSPEAK_API_KEY "OV5KJROGLZED81ZH"
#define STATUS_THINGSPEAK_DEVICE_FIELD 3
#define STATUS_THINGSPEAK_BATTERY_FIELD 4

// Put credentials here on HEAD only
#define WIFI_SSID           "Still_waters"
#define WIFI_PASSWORD       "33turkeys511"
#define THINGSPEAK_API_KEY  "TS0D8HVAOZLRAJ1N"

// How often HEAD checks ThingSpeak
#define THINGSPEAK_CHECK_INTERVAL_MS 3500
#define HEAD_STATUS_DELAY_MS 15000
#define HEAD_STATUS_INTERVAL_MS 3600000
#define MESH_SIGNAL_MAX_AGE_MS 30000

// ============================================================
// MESSAGE TYPES
// ============================================================
enum MessageType : uint8_t {
    MSG_JOIN_BEACON = 1,
    MSG_DISCOVER,
    MSG_TEST,
    MSG_STATUS_REQUEST,
    MSG_STATUS_RESPONSE,
    MSG_THINGSPEAK_UPDATE,
    MSG_UPDATE_ACK
};

// ============================================================
// THINGSPEAK DATA
// ============================================================
typedef struct {
    uint16_t brightness;
    uint32_t color1;
    uint32_t color2;
    uint16_t color3;
    uint16_t pattern;
    uint16_t timeOn;
    uint16_t sleepTime;
    uint16_t fxSpeed;
} ThingSpeakData;

// ============================================================
// MESH MESSAGE
// ============================================================
typedef struct {
    uint32_t session_id;
    uint32_t message_id;
    uint8_t source_id;
    uint8_t target_id;
    uint8_t type;
    uint8_t ttl;
    uint8_t command;
    uint16_t battery_mv;
    int16_t mesh_rssi_dbm;
    uint32_t ack_message_id;
    ThingSpeakData thingspeak;
} MeshMessage;

// ============================================================
// JOIN BEACON
// ============================================================
typedef struct {
    uint32_t session_id;
    uint32_t beacon_id;
    uint8_t source_id;
    uint8_t type;
    uint8_t ttl;
    uint8_t channel;
} JoinBeacon;

// ============================================================
// JOIN BEACON DUPLICATE TRACKING
// ============================================================
#define MAX_SEEN_BEACONS 20

typedef struct {
    uint32_t session_id;
    uint32_t beacon_id;
    uint8_t source_id;
} SeenBeacon;

extern QueueHandle_t led_queue;
extern led_strip_handle_t neopixel_strip;
