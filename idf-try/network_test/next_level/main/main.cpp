#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "led_strip.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h"

#include "nvs_flash.h"


// ============================================================
// CONFIGURATION
// ============================================================

// Change this for each ESP32
//
// Head device = 0
// Other devices = 1, 2, 3, etc.
//
#define DEVICE_ID 3 
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

    uint16_t color1;

    uint16_t color2;

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

    // ThingSpeak payload
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

SeenBeacon seen_beacons[MAX_SEEN_BEACONS] = {};
int seen_beacon_index = 0;

// ============================================================
//my globals
// ============================================================

bool join_beacon_received = false;

// ============================================================
// CHECK IF JOIN BEACON WAS ALREADY SEEN
// ============================================================

bool beacon_already_seen(const JoinBeacon *beacon)
{
    for (int i = 0; i < MAX_SEEN_BEACONS; i++) {

        if (seen_beacons[i].session_id ==
                beacon->session_id &&
            seen_beacons[i].beacon_id ==
                beacon->beacon_id &&
            seen_beacons[i].source_id ==
                beacon->source_id) {

            return true;
        }
    }

    return false;
}


// ============================================================
// REMEMBER JOIN BEACON
// ============================================================

void remember_beacon(const JoinBeacon *beacon)
{
    seen_beacons[seen_beacon_index].session_id =
        beacon->session_id;

    seen_beacons[seen_beacon_index].beacon_id =
        beacon->beacon_id;

    seen_beacons[seen_beacon_index].source_id =
        beacon->source_id;

    seen_beacon_index++;

    if (seen_beacon_index >= MAX_SEEN_BEACONS) {
        seen_beacon_index = 0;
    }
}


// ============================================================
// NEIGHBORS
// ============================================================

bool neighbors[MAX_DEVICES] = {false};

typedef struct {
    bool known;
    uint8_t mac[6];
} KnownDevice;

KnownDevice known_devices[MAX_DEVICES] = {};
uint8_t local_mac[6] = {};


// ============================================================
// SEEN MESSAGES
// ============================================================

typedef struct {

    uint32_t session_id;

    uint32_t message_id;

    uint8_t source_id;

} SeenMessage;

SeenMessage seen_messages[MAX_SEEN_MESSAGES];

int seen_index = 0;


// ============================================================
// SEEN BEACONS
// ============================================================

uint32_t last_seen_beacon_id = 0;


// ============================================================
// MESSAGE COUNTER
// ============================================================

uint32_t next_message_id = 1;


// ============================================================
// NETWORK SESSION
// ============================================================

uint32_t session_id = 0;


// ============================================================
// CURRENT WIFI CHANNEL
// ============================================================

uint8_t network_channel = 0;


// ============================================================
// NETWORK STATE
// ============================================================

enum NetworkState {

    NETWORK_OFF,

    NETWORK_STARTING,

    NETWORK_CHANNEL_DISCOVERY,

    NETWORK_DISCOVERY,

    NETWORK_READY
};

NetworkState network_state = NETWORK_STARTING;


// ============================================================
// CURRENT THINGSPEAK DATA
// ============================================================

ThingSpeakData current_settings = {};


// ============================================================
// LAST THINGSPEAK ENTRY
// ============================================================

long last_thingspeak_entry = 0;
bool thingspeak_entry_initialized = false;
bool device_time_initialized = false;


// ============================================================
// RECEIVE QUEUE
// ============================================================

#define RX_QUEUE_SIZE 20

typedef struct {

    uint8_t mac[6];

    int len;

    uint8_t data[250];

    int8_t rssi_dbm;

} ReceivedPacket;

typedef struct {
    bool known;
    int8_t rssi_dbm;
    TickType_t last_seen;
} NeighborSignal;

QueueHandle_t rx_queue;
QueueHandle_t status_queue;
QueueHandle_t led_queue;
volatile TickType_t power_off_at = 0;
volatile TickType_t remote_power_off_at = 0;
led_strip_handle_t neopixel_strip = nullptr;
NeighborSignal neighbor_signals[MAX_DEVICES] = {};

MeshMessage pending_update = {};
volatile bool update_pending = false;
volatile uint8_t update_ack_mask = 0;
uint8_t update_expected_ack_mask = 0;
TickType_t update_retry_at = 0;
uint8_t update_retry_count = 0;

void write_status_to_thingspeak(
    const MeshMessage *msg
);

void write_head_status_to_status_channel();

esp_err_t status_http_event(
    esp_http_client_event_t *evt
)
{
    return ESP_OK;
}

void status_upload_task(
    void *parameter
)
{
    MeshMessage msg;

    while (1) {
        if (
            xQueueReceive(
                status_queue,
                &msg,
                portMAX_DELAY
            ) == pdTRUE
        ) {
            write_status_to_thingspeak(&msg);
        }
    }
}

void queue_status_upload(
    const MeshMessage *msg
)
{
    if (
        xQueueSend(
            status_queue,
            msg,
            0
        ) != pdTRUE
    ) {
        printf(
            "Status upload queue full\n"
        );
    }
}

void hold_power_for_ms(
    uint32_t duration_ms
)
{
    TickType_t requested_off_at =
        xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);

    if (requested_off_at > power_off_at) {
        power_off_at = requested_off_at;
    }

    gpio_set_level(POWER_SWITCH_GPIO, 1);
}

void power_switch_task(
    void *parameter
)
{
    while (1) {
        TickType_t now = xTaskGetTickCount();

        if (
            power_off_at != 0 &&
            (int32_t)(now - power_off_at) >= 0
        ) {
            gpio_set_level(POWER_SWITCH_GPIO, 0);
            power_off_at = 0;
        }

        if (
            remote_power_off_at != 0 &&
            (int32_t)(now - remote_power_off_at) >= 0
        ) {
            gpio_set_level(NEOPIXEL_POWER_PIN, 0);
            remote_power_off_at = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void hold_remote_power_for_ms(
    uint32_t duration_ms
)
{
    remote_power_off_at =
        xTaskGetTickCount() + pdMS_TO_TICKS(duration_ms);

    gpio_set_level(NEOPIXEL_POWER_PIN, 1);
}

void record_received_signal(
    const uint8_t *mac,
    const uint8_t *data,
    int len,
    int8_t rssi_dbm
)
{
    int device_id = -1;

    if (
        len == sizeof(MeshMessage)
    ) {
        MeshMessage message = {};
        memcpy(&message, data, sizeof(message));

        if (
            message.type == MSG_DISCOVER &&
            message.source_id < NUM_DEVICES
        ) {
            device_id = message.source_id;
        }
    }

    if (device_id < 0) {
        for (int i = 0; i < NUM_DEVICES; i++) {
            if (
                known_devices[i].known &&
                memcmp(known_devices[i].mac, mac, 6) == 0
            ) {
                device_id = i;
                break;
            }
        }
    }

    if (
        device_id >= 0 &&
        device_id < NUM_DEVICES &&
        device_id != DEVICE_ID
    ) {
        neighbor_signals[device_id].known = true;
        neighbor_signals[device_id].rssi_dbm = rssi_dbm;
        neighbor_signals[device_id].last_seen = xTaskGetTickCount();
    }
}

int16_t strongest_recent_mesh_rssi()
{
    int16_t strongest = -127;
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < NUM_DEVICES; i++) {
        if (
            neighbor_signals[i].known &&
            (now - neighbor_signals[i].last_seen) <=
                pdMS_TO_TICKS(MESH_SIGNAL_MAX_AGE_MS) &&
            neighbor_signals[i].rssi_dbm > strongest
        ) {
            strongest = neighbor_signals[i].rssi_dbm;
        }
    }

    return strongest;
}

void build_mesh_signal_summary(
    char *buffer,
    size_t buffer_size
)
{
    size_t used = 0;
    TickType_t now = xTaskGetTickCount();

    used += snprintf(
        buffer + used,
        buffer_size - used,
        "mesh"
    );

    for (int i = 0; i < NUM_DEVICES; i++) {
        if (
            neighbor_signals[i].known &&
            (now - neighbor_signals[i].last_seen) <=
                pdMS_TO_TICKS(MESH_SIGNAL_MAX_AGE_MS) &&
            used < buffer_size
        ) {
            used += snprintf(
                buffer + used,
                buffer_size - used,
                "_%d_%d",
                i,
                neighbor_signals[i].rssi_dbm
            );
        }
    }
}

uint16_t read_battery_level()
{
    static adc_oneshot_unit_handle_t adc_handle = nullptr;

    if (adc_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = ADC_UNIT_1;

        ESP_ERROR_CHECK(
            adc_oneshot_new_unit(
                &init_config,
                &adc_handle
            )
        );

        adc_oneshot_chan_cfg_t channel_config = {};
        channel_config.bitwidth = ADC_BITWIDTH_DEFAULT;
        channel_config.atten = ADC_ATTEN_DB_12;

        ESP_ERROR_CHECK(
            adc_oneshot_config_channel(
                adc_handle,
                ADC_CHANNEL_7,
                &channel_config
            )
        );
    }

    int raw_level = 0;

    ESP_ERROR_CHECK(
        adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL_7,
            &raw_level
        )
    );

    if (raw_level < 0) {
        return 0;
    }

    if (raw_level > UINT16_MAX) {
        return UINT16_MAX;
    }

    return (uint16_t)raw_level;
}

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

void update_neopixels(
    const ThingSpeakData *data
)
{
    if (data->brightness <= 1) {
        ESP_ERROR_CHECK(
            led_strip_clear(neopixel_strip)
        );
        gpio_set_level(NEOPIXEL_POWER_PIN, 0);
        return;
    }

    gpio_set_level(NEOPIXEL_POWER_PIN, 1);

    if (data->pattern != 0) {
        printf(
            "NeoPixel pattern %u is not implemented yet\n",
            data->pattern
        );
        return;
    }

    uint8_t red = (data->color1 >> 11) & 0x1F;
    uint8_t green = (data->color1 >> 5) & 0x3F;
    uint8_t blue = data->color1 & 0x1F;

    red = (uint8_t)((red * 255U / 31U) * data->brightness / 255U);
    green = (uint8_t)((green * 255U / 63U) * data->brightness / 255U);
    blue = (uint8_t)((blue * 255U / 31U) * data->brightness / 255U);

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

void send_update_ack(
    uint32_t message_id
);

void led_update_task(
    void *parameter
)
{
    MeshMessage update;

    while (1) {
        if (
            xQueueReceive(
                led_queue,
                &update,
                portMAX_DELAY
            ) == pdTRUE
        ) {
            update_neopixels(&update.thingspeak);
            send_update_ack(update.message_id);
        }
    }
}


// ============================================================
// LED
// ============================================================

void flash_led_twice()
{
    for (int i = 0; i < 2; i++) {

        gpio_set_level(LED_GPIO, 1);

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );

        gpio_set_level(LED_GPIO, 0);

        vTaskDelay(
            pdMS_TO_TICKS(100)
        );
    }
}


// ============================================================
// PRINT MAC
// ============================================================

void print_mac(const uint8_t *mac)
{
    printf(
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
}


// ============================================================
// ADD PEER
// ============================================================

void add_peer(const uint8_t *mac)
{
    if (memcmp(
            mac,
            local_mac,
            6
        ) == 0) {

        return;
    }

    if (esp_now_is_peer_exist(mac)) {
        return;
    }

    esp_now_peer_info_t peer = {};

    memcpy(
        peer.peer_addr,
        mac,
        ESP_NOW_ETH_ALEN
    );

    peer.channel = 0;

    peer.ifidx = WIFI_IF_STA;

    peer.encrypt = false;

    esp_err_t result =
        esp_now_add_peer(&peer);

    if (result != ESP_OK &&
        result != ESP_ERR_ESPNOW_EXIST) {

        printf(
            "Could not add peer "
        );

        print_mac(mac);

        printf(
            ": %s\n",
            esp_err_to_name(result)
        );
    }
}


// ============================================================
// REMEMBER MESSAGE
// ============================================================

void remember_message(
    const MeshMessage *msg
)
{
    seen_messages[seen_index] =
    {
        msg->session_id,
        msg->message_id,
        msg->source_id
    };

    seen_index++;

    if (seen_index >= MAX_SEEN_MESSAGES) {
        seen_index = 0;
    }
}


// ============================================================
// CHECK MESSAGE ALREADY SEEN
// ============================================================

bool message_already_seen(
    const MeshMessage *msg
)
{
    for (int i = 0;
         i < MAX_SEEN_MESSAGES;
         i++) {

        if (
            seen_messages[i].session_id ==
                msg->session_id &&

            seen_messages[i].message_id ==
                msg->message_id &&

            seen_messages[i].source_id ==
                msg->source_id
        ) {

            return true;
        }
    }

    return false;
}


// ============================================================
// SEND TO DEVICE
// ============================================================

void send_to_device(
    int device_id,
    const MeshMessage *msg
)
{
    if (
        device_id < 0 ||
        device_id >= NUM_DEVICES ||
        device_id == DEVICE_ID ||
        !known_devices[device_id].known
    ) {
        if (
            device_id >= 0 &&
            device_id < NUM_DEVICES &&
            !known_devices[device_id].known
        ) {
            printf(
                "Device %d MAC has not been learned yet\n",
                device_id
            );
        }

        return;
    }

    add_peer(known_devices[device_id].mac);

    esp_err_t result =
        esp_now_send(
            known_devices[device_id].mac,
            (const uint8_t *)msg,
            sizeof(MeshMessage)
        );

    if (result != ESP_OK) {

        printf(
            "Send to Device %d failed: %s\n",
            device_id,
            esp_err_to_name(result)
        );
    }
}


// ============================================================
// FORWARD MESSAGE TO NEIGHBORS
// ============================================================

void forward_to_neighbors(
    const MeshMessage *msg
)
{
    if (msg->ttl == 0) {

        printf(
            "TTL expired - not forwarding\n"
        );

        return;
    }

    MeshMessage forwarded = *msg;

    forwarded.ttl--;

    for (int i = 0;
         i < NUM_DEVICES;
         i++) {

        if (!neighbors[i]) {
            continue;
        }

        if (i == msg->source_id) {
            continue;
        }

        if (i == DEVICE_ID) {
            continue;
        }

        printf(
            "Forwarding message %lu "
            "to Device %d "
            "(TTL=%d)\n",

            (unsigned long)
                forwarded.message_id,

            i,

            forwarded.ttl
        );

        send_to_device(
            i,
            &forwarded
        );
    }
}


// ============================================================
// SEND BROADCAST MESSAGE
// ============================================================
//
// Used for discovery and initial flooding.
// ============================================================

void send_broadcast(
    const MeshMessage *msg
)
{
    uint8_t broadcast_mac[6] =
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

    if (!esp_now_is_peer_exist(
            broadcast_mac)) {

        esp_now_peer_info_t peer = {};

        memcpy(
            peer.peer_addr,
            broadcast_mac,
            6
        );

        peer.channel = 0;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;

        esp_now_add_peer(&peer);
    }

    esp_err_t result =
        esp_now_send(
            broadcast_mac,
            (const uint8_t *)msg,
            sizeof(MeshMessage)
        );

    if (result != ESP_OK) {

        printf(
            "Broadcast failed: %s\n",
            esp_err_to_name(result)
        );
    }
}

void send_update_ack(
    uint32_t message_id
)
{
    MeshMessage ack = {};

    ack.session_id = session_id;
    ack.message_id = next_message_id++;
    ack.source_id = DEVICE_ID;
    ack.target_id = HEAD_ID;
    ack.type = MSG_UPDATE_ACK;
    ack.ttl = DEFAULT_TTL;
    ack.ack_message_id = message_id;

    send_broadcast(&ack);
}


// ============================================================
// SEND JOIN BEACON
// ============================================================

void send_join_beacon()
{
     printf(
        "*** SEND_JOIN_BEACON called, state=%d ***\n",
        network_state
    );

    JoinBeacon beacon = {};

    beacon.session_id = session_id;

    beacon.beacon_id =
        next_message_id++;

    beacon.source_id = DEVICE_ID;

    beacon.type =
        MSG_JOIN_BEACON;

    beacon.ttl = DEFAULT_TTL;

    beacon.channel =
        network_channel;


    uint8_t broadcast_mac[6] =
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


    if (!esp_now_is_peer_exist(
            broadcast_mac)) {

        esp_now_peer_info_t peer = {};

        memcpy(
            peer.peer_addr,
            broadcast_mac,
            6
        );

        peer.channel = 0;

        peer.ifidx = WIFI_IF_STA;

        peer.encrypt = false;

        esp_now_add_peer(&peer);
    }


    esp_err_t result =
        esp_now_send(
            broadcast_mac,
            (uint8_t *)&beacon,
            sizeof(beacon)
        );


    if (result != ESP_OK) {

        printf(
            "JOIN beacon send failed: %s\n",
            esp_err_to_name(result)
        );
    }
}


// ============================================================
// SEND DISCOVER
// ============================================================

void send_discover()
{
    MeshMessage msg = {};

    msg.session_id = session_id;

    msg.message_id =
        next_message_id++;

    msg.source_id =
        DEVICE_ID;

    msg.target_id =
        BROADCAST_ID;

    msg.type =
        MSG_DISCOVER;

    msg.ttl = 0;


    printf(
        "Device %d sending DISCOVER\n",
        DEVICE_ID
    );

    printf(
        "DISCOVER payload source_id=%u, size=%d\n",
        msg.source_id,
        sizeof(msg)
    );


    send_broadcast(&msg);
}


// ============================================================
// SEND TEST MESSAGE
// ============================================================

void send_test_message()
{
    MeshMessage msg = {};

    msg.session_id =
        session_id;

    msg.message_id =
        next_message_id++;

    msg.source_id =
        DEVICE_ID;

    msg.target_id =
        BROADCAST_ID;

    msg.type =
        MSG_TEST;

    msg.ttl =
        DEFAULT_TTL;


    remember_message(&msg);


    printf("\n");
    printf(
        "================================\n"
    );

    printf(
        "HEAD SENDING TEST MESSAGE\n"
    );

    printf(
        "Session: %lu\n",
        (unsigned long)
            msg.session_id
    );

    printf(
        "Message ID: %lu\n",
        (unsigned long)
            msg.message_id
    );

    printf(
        "================================\n"
    );


    // Initial broadcast
    send_broadcast(&msg);
}


// ============================================================
// SEND THINGSPEAK UPDATE
// ============================================================

void send_thingspeak_update(
    const ThingSpeakData *data
)
{
    MeshMessage msg = {};

    msg.session_id =
        session_id;

    msg.message_id =
        next_message_id++;

    msg.source_id =
        HEAD_ID;

    msg.target_id =
        BROADCAST_ID;

    msg.type =
        MSG_THINGSPEAK_UPDATE;

    msg.ttl =
        DEFAULT_TTL;

    msg.thingspeak =
        *data;


    remember_message(&msg);


    printf("\n");
    printf(
        "========================================\n"
    );

    printf(
        "HEAD SENDING THINGSPEAK UPDATE\n"
    );

    printf(
        "Message ID: %lu\n",
        (unsigned long)
            msg.message_id
    );

    printf(
        "Brightness: %u\n",
        data->brightness
    );

    printf(
        "Color1: %u\n",
        data->color1
    );

    printf(
        "Color2: %u\n",
        data->color2
    );

    printf(
        "Color3: %u\n",
        data->color3
    );

    printf(
        "Pattern: %u\n",
        data->pattern
    );

    printf(
        "Time On: %u\n",
        data->timeOn
    );

    printf(
        "Sleep Time: %u\n",
        data->sleepTime
    );

    printf(
        "FX Speed: %u\n",
        data->fxSpeed
    );

    printf(
        "========================================\n"
    );


    pending_update = msg;
    update_pending = true;
    update_ack_mask = 0;
    update_expected_ack_mask = 0;

    for (int i = 0; i < NUM_DEVICES; i++) {
        if (i != HEAD_ID && known_devices[i].known) {
            update_expected_ack_mask |= (uint8_t)(1U << i);
        }
    }

    update_retry_count = 1;
    update_retry_at =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(UPDATE_RETRY_INTERVAL_MS);

    send_broadcast(&pending_update);
}

void update_retry_task(
    void *parameter
)
{
    while (1) {
        if (
            update_pending &&
            update_expected_ack_mask != 0 &&
            (update_ack_mask & update_expected_ack_mask) ==
                update_expected_ack_mask
        ) {
            update_pending = false;
            printf(
                "All known devices acknowledged update %lu\n",
                (unsigned long)pending_update.message_id
            );
        }

        if (
            update_pending &&
            (int32_t)(xTaskGetTickCount() - update_retry_at) >= 0
        ) {
            if (update_retry_count >= MAX_UPDATE_RETRIES) {
                printf(
                    "Update %lu timed out after %d attempts\n",
                    (unsigned long)pending_update.message_id,
                    update_retry_count
                );
                update_pending = false;
            } else {
                printf(
                    "Retrying update %lu (attempt %d)\n",
                    (unsigned long)pending_update.message_id,
                    update_retry_count + 1
                );

                send_broadcast(&pending_update);
                update_retry_count++;
                update_retry_at =
                    xTaskGetTickCount() +
                    pdMS_TO_TICKS(UPDATE_RETRY_INTERVAL_MS);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ============================================================
// SEND STATUS REQUEST
// ============================================================

void send_status_request(
    int target_device_id
)
{
    MeshMessage msg = {};

    msg.session_id = session_id;
    msg.message_id = next_message_id++;
    msg.source_id = HEAD_ID;
    msg.target_id = target_device_id;
    msg.type = MSG_STATUS_REQUEST;
    msg.ttl = DEFAULT_TTL;

    remember_message(&msg);

    printf(
        "HEAD requesting status from Device %d\n",
        target_device_id
    );

    send_broadcast(&msg);
}


// ============================================================
// SEND STATUS RESPONSE
// ============================================================

void send_status_response()
{
    MeshMessage msg = {};

    msg.session_id = session_id;
    msg.message_id = next_message_id++;
    msg.source_id = DEVICE_ID;
    msg.target_id = HEAD_ID;
    msg.type = MSG_STATUS_RESPONSE;
    msg.ttl = DEFAULT_TTL;
    msg.command = network_state;
    msg.battery_mv = read_battery_level();
    msg.mesh_rssi_dbm = strongest_recent_mesh_rssi();

    remember_message(&msg);

    send_broadcast(&msg);
}


// ============================================================
// PROCESS THINGSPEAK UPDATE
// ============================================================

void process_thingspeak_update(
    const MeshMessage *msg
)
{
    current_settings =
        msg->thingspeak;


    printf("\n");
    printf(
        "******** THINGSPEAK UPDATE ********\n"
    );

    printf(
        "Device %d applying new settings\n",
        DEVICE_ID
    );

    printf(
        "Brightness = %u\n",
        current_settings.brightness
    );

    printf(
        "Color1     = %u\n",
        current_settings.color1
    );

    printf(
        "Color2     = %u\n",
        current_settings.color2
    );

    printf(
        "Color3     = %u\n",
        current_settings.color3
    );

    printf(
        "Pattern    = %u\n",
        current_settings.pattern
    );

    printf(
        "Time On    = %u\n",
        current_settings.timeOn
    );

    printf(
        "Sleep Time = %u\n",
        current_settings.sleepTime
    );

    printf(
        "FX Speed   = %u\n",
        current_settings.fxSpeed
    );

    printf(
        "***********************************\n\n"
    );


    if (DEVICE_ID != HEAD_ID) {
        if (current_settings.timeOn > 0) {
            hold_remote_power_for_ms(
                (uint32_t)current_settings.timeOn * 1000U
            );
        } else {
            remote_power_off_at = 0;
            gpio_set_level(NEOPIXEL_POWER_PIN, 0);
        }

        if (
            xQueueSend(
                led_queue,
                msg,
                0
            ) != pdTRUE
        ) {
            printf(
                "LED update queue full; update not acknowledged\n"
            );
        }
    }

    // This is where the actual LED/device
    // behavior will eventually be updated.
}


// ============================================================
// PRINT NEIGHBORS
// ============================================================

void print_neighbors()
{
    printf(
        "\nDevice %d neighbors:\n",
        DEVICE_ID
    );

    bool found = false;

    for (int i = 0;
         i < NUM_DEVICES;
         i++) {

        if (neighbors[i]) {

            printf(
                "  Device %d\n",
                i
            );

            found = true;
        }
    }

    if (!found) {
        printf("  NONE\n");
    }

    printf("\n");
}


// ============================================================
// PRINT MESH MEMBERS
// ============================================================

void print_mesh_members()
{
    printf(
        "\nMesh members known by HEAD:\n"
    );

    printf(
        "  Device %d (HEAD) MAC: ",
        HEAD_ID
    );

    print_mac(local_mac);
    printf("\n");

    for (int i = 0; i < NUM_DEVICES; i++) {

        if (i == HEAD_ID) {
            continue;
        }

        if (known_devices[i].known) {
            printf(
                "  Device %d MAC: ",
                i
            );

            print_mac(known_devices[i].mac);
            printf("\n");
        } else {
            printf(
                "  Device %d: NOT SEEN\n",
                i
            );
        }
    }

    printf("\n");
}


// ============================================================
// PRINT MESH RSSI MAP
// ============================================================

void print_mesh_rssi_map()
{
    TickType_t now = xTaskGetTickCount();

    printf(
        "\nMesh RSSI map from Device %d:\n",
        HEAD_ID
    );

    printf(
        "  Device %d (HEAD)\n",
        HEAD_ID
    );

    for (int i = 0; i < NUM_DEVICES; i++) {

        if (i == HEAD_ID) {
            continue;
        }

        if (!known_devices[i].known) {
            printf(
                "  Device %d: UNKNOWN\n",
                i
            );
            continue;
        }

        if (!neighbor_signals[i].known) {
            printf(
                "  Device %d: NO RSSI DATA\n",
                i
            );
            continue;
        }

        TickType_t age =
            now - neighbor_signals[i].last_seen;

        if (age > pdMS_TO_TICKS(MESH_SIGNAL_MAX_AGE_MS)) {
            printf(
                "  Device %d: STALE (age=%lu ms)\n",
                i,
                (unsigned long)pdTICKS_TO_MS(age)
            );
            continue;
        }

        printf(
            "  Device %d: RSSI=%d dBm, age=%lu ms\n",
            i,
            neighbor_signals[i].rssi_dbm,
            (unsigned long)pdTICKS_TO_MS(age)
        );
    }

    printf("\n");
}


// ============================================================
// RECEIVE CALLBACK
// ============================================================
//
// IMPORTANT:
// Do not do significant processing here.
// Put packet into FreeRTOS queue.
// ============================================================

void on_data_recv(
    const esp_now_recv_info_t *recv_info,
    const uint8_t *data,
    int len
)
{
    printf(
        "ESP-NOW RX CALLBACK FIRED, len=%d\n",
        len
    );

    if (len <= 0 ||
        len > 250) {
        return;
    }

    ReceivedPacket packet = {};

    int8_t rssi_dbm = -127;

    if (recv_info->rx_ctrl != NULL) {
        rssi_dbm = recv_info->rx_ctrl->rssi;
    }

    record_received_signal(
        recv_info->src_addr,
        data,
        len,
        rssi_dbm
    );

    memcpy(
        packet.mac,
        recv_info->src_addr,
        6
    );

    packet.len = len;
    packet.rssi_dbm = rssi_dbm;

    memcpy(
        packet.data,
        data,
        len
    );

    if (xQueueSend(
            rx_queue,
            &packet,
            0
        ) != pdTRUE) {

        printf("RX queue full\n");
    }
}


// ============================================================
// PROCESS JOIN BEACON
// ============================================================

void process_join_beacon(
    const uint8_t *data,
    int len
)
{
    if (len != sizeof(JoinBeacon)) {
        return;
    }


    JoinBeacon beacon;

    memcpy(
        &beacon,
        data,
        sizeof(beacon)
    );


    int source =
        beacon.source_id;


    if (source < 0 ||
        source >= NUM_DEVICES) {

        return;
    }


    if (source == DEVICE_ID) {
        return;
    }


    // --------------------------------------------------------
    // Ignore duplicate JOIN beacon
    // --------------------------------------------------------

    if (beacon.beacon_id == last_seen_beacon_id) {

        printf(
            "Duplicate JOIN beacon ignored "
            "(ID=%lu)\n",
            (unsigned long)beacon.beacon_id
        );

        return;
    }

    last_seen_beacon_id =
        beacon.beacon_id;


    // --------------------------------------------------------
    // We received a valid MyFriend beacon.
    // The radio is already on the channel on which
    // we received it.
    // --------------------------------------------------------

    printf("\n");

    printf(
        "JOIN BEACON received from "
        "Device %d\n",
        source
    );

    printf(
        "Session: %lu\n",
        (unsigned long)beacon.session_id
    );

    printf(
        "Channel: %d\n",
        beacon.channel
    );


    // --------------------------------------------------------
    // Tell channel scanner that we found the network
    // --------------------------------------------------------

    network_channel =
        beacon.channel;

    join_beacon_received =
        true;


    // --------------------------------------------------------
    // If we are a remote, adopt the Head's session
    // --------------------------------------------------------

    if (DEVICE_ID != HEAD_ID) {

        if (session_id !=
            beacon.session_id) {

            session_id =
                beacon.session_id;

            printf(
                "Joined MyFriend session "
                "%lu\n",
                (unsigned long)session_id
            );
        }
    }


    // --------------------------------------------------------
    // Add direct neighbor
    // --------------------------------------------------------

    neighbors[source] =
        true;

    // --------------------------------------------------------
    // Propagate beacon
    // --------------------------------------------------------

    if (beacon.ttl > 0) {

        JoinBeacon forwarded =
            beacon;

        forwarded.ttl--;


        uint8_t broadcast_mac[6] =
            {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


        esp_err_t result =
            esp_now_send(
                broadcast_mac,
                (uint8_t *)&forwarded,
                sizeof(forwarded)
            );


        if (result == ESP_OK) {

            printf(
                "Propagated JOIN beacon "
                "(ID=%lu, TTL=%d)\n",
                (unsigned long)
                    forwarded.beacon_id,
                forwarded.ttl
            );

        } else {

            printf(
                "JOIN beacon propagation failed: %s\n",
                esp_err_to_name(result)
            );
        }
    }


    network_state =
        NETWORK_DISCOVERY;
}




// ============================================================
// PROCESS DISCOVER
// ============================================================

void process_discover(
    const uint8_t *data,
    int len,
    const uint8_t *source_mac
)
{
    printf("Processing DISCOVER\n");

    if (len != sizeof(MeshMessage)) {
        printf(
            "DISCOVER wrong length: %d, expected %d\n",
            len,
            sizeof(MeshMessage)
        );
        return;
    }

    MeshMessage msg;

    memcpy(
        &msg,
        data,
        sizeof(msg)
    );


    int source = msg.source_id;

    printf(
        "DISCOVER payload source_id=%u, packet byte[8]=%u, "
        "compiled DEVICE_ID=%d\n",
        msg.source_id,
        data[8],
        DEVICE_ID
    );


    if (
        source < HEAD_ID ||
        source >= NUM_DEVICES ||
        source == DEVICE_ID
    ) {

        printf(
            "DISCOVER has invalid source ID: %d\n",
            source
        );

        printf(
            "DISCOVER sender MAC: "
        );

        print_mac(source_mac);

        printf("\n");

        return;
    }


    known_devices[source].known = true;
    memcpy(
        known_devices[source].mac,
        source_mac,
        6
    );


    neighbors[source] = true;

    add_peer(source_mac);


    printf(
        "DISCOVER received directly "
        "from Device %d\n",
        source
    );
}


// ============================================================
// PROCESS NORMAL MESSAGE
// ============================================================

void process_mesh_message(
    const MeshMessage *msg
)
{
    // --------------------------------------------------------
    // Session validation
    // --------------------------------------------------------

    if (msg->session_id !=
        session_id) {

        printf(
            "Ignoring wrong session "
            "%lu\n",

            (unsigned long)
                msg->session_id
        );

        return;
    }


    // --------------------------------------------------------
    // Duplicate detection
    // --------------------------------------------------------

    if (message_already_seen(msg)) {

        if (
            msg->type == MSG_THINGSPEAK_UPDATE &&
            DEVICE_ID != HEAD_ID
        ) {
            send_update_ack(msg->message_id);
        }

        printf(
            "Duplicate ignored: "
            "Source %d Message %lu\n",

            msg->source_id,

            (unsigned long)
                msg->message_id
        );

        return;
    }


    remember_message(msg);


    printf("\n");
    printf(
        "--------------------------------\n"
    );

    printf(
        "NEW MESSAGE RECEIVED\n"
    );

    printf(
        "Device: %d\n",
        DEVICE_ID
    );

    printf(
        "From source: %d\n",
        msg->source_id
    );

    printf(
        "Message ID: %lu\n",
        (unsigned long)
            msg->message_id
    );

    printf(
        "Type: %d\n",
        msg->type
    );

    printf(
        "Target: %d\n",
        msg->target_id
    );

    printf(
        "TTL: %d\n",
        msg->ttl
    );

    printf(
        "--------------------------------\n"
    );


    // --------------------------------------------------------
    // Process if this device is target
    // --------------------------------------------------------

    if (
        msg->target_id == DEVICE_ID ||
        msg->target_id == BROADCAST_ID
    ) {

        switch (msg->type) {

            case MSG_TEST:

                printf(
                    "Device %d processed "
                    "TEST message!\n",
                    DEVICE_ID
                );

                break;


            case MSG_THINGSPEAK_UPDATE:

                process_thingspeak_update(
                    msg
                );

                break;


            case MSG_STATUS_REQUEST:

                printf(
                    "Status request "
                    "for Device %d\n",
                    DEVICE_ID
                );

                if (DEVICE_ID != HEAD_ID) {
                    send_status_response();
                }

                break;


            case MSG_STATUS_RESPONSE:

                printf(
                    "STATUS RESPONSE from Device %d: state=%d\n",
                    msg->source_id,
                    msg->command
                );

                if (DEVICE_ID == HEAD_ID) {
                    queue_status_upload(msg);
                    print_mesh_rssi_map();
                }

                break;


            case MSG_UPDATE_ACK:

                if (
                    DEVICE_ID == HEAD_ID &&
                    update_pending &&
                    msg->ack_message_id == pending_update.message_id &&
                    msg->source_id < NUM_DEVICES
                ) {
                    update_ack_mask |=
                        (uint8_t)(1U << msg->source_id);

                    printf(
                        "UPDATE ACK from Device %d for message %lu\n",
                        msg->source_id,
                        (unsigned long)msg->ack_message_id
                    );
                }

                break;


            default:

                printf(
                    "Unknown message type\n"
                );

                break;
        }
    }


    // --------------------------------------------------------
    // Forward
    // --------------------------------------------------------

    if (
        msg->ttl > 0 &&
        msg->target_id != DEVICE_ID
    ) {

        forward_to_neighbors(msg);
    }
}


// ============================================================
// RX PROCESSING TASK
// ============================================================

void rx_processing_task(
    void *parameter
)
{
    ReceivedPacket packet;


    while (1) {

        if (xQueueReceive(
                rx_queue,
                &packet,
                portMAX_DELAY
            ) != pdTRUE) {

            continue;
        }
printf(
    "RX packet received, len=%d\n",
    packet.len
);

        // ----------------------------------------------------
        // Determine packet type
        // ----------------------------------------------------

        if (packet.len ==
            sizeof(JoinBeacon)) {

            JoinBeacon beacon;

            memcpy(
                &beacon,
                packet.data,
                sizeof(beacon)
            );


            if (beacon.type ==
                MSG_JOIN_BEACON) {

                process_join_beacon(
                    packet.data,
                    packet.len
                );

                continue;
            }
        }


        // ----------------------------------------------------
        // Mesh packet
        // ----------------------------------------------------

        if (packet.len ==
            sizeof(MeshMessage)) {

            MeshMessage msg;

            memcpy(
                &msg,
                packet.data,
                sizeof(msg)
            );


            if (msg.type ==
                MSG_DISCOVER) {

                process_discover(
                    packet.data,
                    packet.len,
                    packet.mac
                );

                continue;
            }


            process_mesh_message(
                &msg
            );
        }
    }
}


// ============================================================
// WIFI EVENT HANDLER
// ============================================================

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data
)
{
    if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START
    ) {

        esp_wifi_connect();

    } else if (
        event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_DISCONNECTED
    ) {

        printf(
            "WiFi disconnected - reconnecting\n"
        );

        esp_wifi_connect();

    } else if (
        event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP
    ) {

        ip_event_got_ip_t *event =
            (ip_event_got_ip_t *)event_data;

        printf(
            "WiFi connected. IP: "
        );

        printf(
            IPSTR "\n",
            IP2STR(
                &event->ip_info.ip
            )
        );
    }
}


// ============================================================
// INITIALIZE WIFI - HEAD ONLY
// ============================================================

void initialize_head_wifi()
{
    printf(
        "Initializing HEAD WiFi...\n"
    );


    ESP_ERROR_CHECK(
        esp_netif_init()
    );


    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    esp_netif_create_default_wifi_sta();


    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL
        )
    );


    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );


    wifi_config_t wifi_config = {};

    strncpy(
        (char *)wifi_config.sta.ssid,
        WIFI_SSID,
        sizeof(wifi_config.sta.ssid)
    );

    strncpy(
        (char *)wifi_config.sta.password,
        WIFI_PASSWORD,
        sizeof(wifi_config.sta.password)
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_start()
    );


    printf(
        "Connecting to WiFi: %s\n",
        WIFI_SSID
    );


    esp_wifi_connect();


    // Give WiFi time to connect
    for (int i = 0; i < 30; i++) {

        wifi_ap_record_t ap_info;

        if (
            esp_wifi_sta_get_ap_info(
                &ap_info
            ) == ESP_OK
        ) {

            printf(
                "WiFi connected\n"
            );

            printf(
                "SSID: %s\n",
                ap_info.ssid
            );

            printf(
                "WiFi channel: %d\n",
                ap_info.primary
            );

            network_channel =
                ap_info.primary;

            return;
        }

        vTaskDelay(
            pdMS_TO_TICKS(500)
        );
    }


    printf(
        "ERROR: Could not determine "
        "WiFi channel\n"
    );
}


// ============================================================
// INITIALIZE WIFI - REMOTE ONLY
// ============================================================
//
// Remotes use the WiFi radio for ESP-NOW,
// but NEVER associate with an AP.
// ============================================================

void initialize_remote_radio()
{
    printf(
        "Initializing Remote ESP-NOW radio...\n"
    );


    ESP_ERROR_CHECK(
        esp_netif_init()
    );


    ESP_ERROR_CHECK(
        esp_event_loop_create_default()
    );


    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();


    ESP_ERROR_CHECK(
        esp_wifi_init(&cfg)
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_mode(
            WIFI_MODE_STA
        )
    );


    ESP_ERROR_CHECK(
        esp_wifi_start()
    );


    printf(
        "Remote radio started "
        "(NO WiFi connection)\n"
    );
}


// ============================================================
// INITIALIZE ESP-NOW
// ============================================================

void initialize_esp_now()
{
    printf(
        "Initializing ESP-NOW...\n"
    );


    ESP_ERROR_CHECK(
        esp_now_init()
    );


    ESP_ERROR_CHECK(
        esp_now_register_recv_cb(
            on_data_recv
        )
    );


    // Broadcast peer
    uint8_t broadcast_mac[6] =
        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};


    esp_now_peer_info_t peer = {};

    memcpy(
        peer.peer_addr,
        broadcast_mac,
        6
    );

    peer.channel = 0;

    peer.ifidx =
        WIFI_IF_STA;

    peer.encrypt = false;


    esp_err_t result =
        esp_now_add_peer(&peer);


    if (
        result != ESP_OK &&
        result != ESP_ERR_ESPNOW_EXIST
    ) {

        printf(
            "Broadcast peer error: %s\n",
            esp_err_to_name(result)
        );
    }
}


// ============================================================
// SET RADIO CHANNEL
// ============================================================

void set_radio_channel(
    uint8_t channel
)
{
    printf(
        "Setting radio channel to %d\n",
        channel
    );


    ESP_ERROR_CHECK(
        esp_wifi_set_channel(
            channel,
            WIFI_SECOND_CHAN_NONE
        )
    );


    network_channel =
        channel;
}


// ============================================================
// SCAN FOR MYFRIEND CHANNEL
// ============================================================

bool scan_for_myfriend_channel()
{
    join_beacon_received = false;
    printf("\n");
    printf(
        "========================================\n"
    );

    printf(
        "Scanning for MyFriend JOIN beacon...\n"
    );

    printf(
        "========================================\n"
    );


    for (
        int channel =
            FIRST_WIFI_CHANNEL;

        channel <=
            LAST_WIFI_CHANNEL;

        channel++
    ) {

        printf(
            "Scanning channel %d...\n",
            channel
        );


        set_radio_channel(
            channel
        );


        // Listen on this channel.
        //
        // The RX callback will receive JOIN
        // beacons if one is present.
        //
        // We check the session afterward.

        uint32_t start =
            xTaskGetTickCount();


        while (
            (
                xTaskGetTickCount()
                - start
            )
            <
            pdMS_TO_TICKS(
                CHANNEL_SCAN_TIME_MS
            )
        ) {

            // Check if a beacon caused us
            // to enter discovery.
            //
            // session_id != 0 means we joined.

            if (join_beacon_received) {

                printf(
                    "MyFriend network found!\n"
                );

                printf(
                    "Channel = %d\n",
                    network_channel
                );

                return true;
            }


            vTaskDelay(
                pdMS_TO_TICKS(20)
            );
        }
    }


    return false;
}


// ============================================================
// HEAD JOIN BEACON TASK
// ============================================================
void head_beacon_task(
    void *parameter
)
{
    while (1) {

        if (
            DEVICE_ID == HEAD_ID
        ) {

            if (
                network_state ==
                NETWORK_DISCOVERY
            ) {
                send_join_beacon();
            }
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                JOIN_BEACON_INTERVAL_MS
            )
        );
    }
}

// ============================================================
// DISCOVERY TASK
// ============================================================

void discovery_task(
    void *parameter
)
{
    TickType_t discovery_started_at = 0;

    vTaskDelay(
        pdMS_TO_TICKS(1000)
    );


    while (1) {

        if (
            network_state ==
                NETWORK_DISCOVERY
        ) {

            if (discovery_started_at == 0) {
                discovery_started_at = xTaskGetTickCount();
            }

            if (
                DEVICE_ID != HEAD_ID &&
                xTaskGetTickCount() - discovery_started_at >=
                    pdMS_TO_TICKS(REMOTE_DISCOVERY_DURATION_MS)
            ) {
                network_state = NETWORK_READY;

                printf(
                    "Remote discovery complete; stopping DISCOVER broadcasts\n"
                );

                discovery_started_at = 0;
                continue;
            }

            send_discover();

            vTaskDelay(
                pdMS_TO_TICKS(
                    DISCOVERY_INTERVAL_MS
                )
            );

        } else {

            discovery_started_at = 0;

            vTaskDelay(
                pdMS_TO_TICKS(500)
            );
        }
    }
}


// ============================================================
// THINGSPEAK HTTP RESPONSE BUFFER
// ============================================================

#define HTTP_BUFFER_SIZE 4096

char http_buffer[
    HTTP_BUFFER_SIZE
];

int http_buffer_len = 0;


// ============================================================
// HTTP EVENT HANDLER
// ============================================================

esp_err_t thingspeak_http_event(
    esp_http_client_event_t *evt
)
{
    switch (evt->event_id) {

        case HTTP_EVENT_ON_DATA:

            if (
                http_buffer_len +
                evt->data_len
                <
                HTTP_BUFFER_SIZE - 1
            ) {

                memcpy(
                    &http_buffer[
                        http_buffer_len
                    ],

                    evt->data,

                    evt->data_len
                );

                http_buffer_len +=
                    evt->data_len;

                http_buffer[
                    http_buffer_len
                ] = '\0';
            }

            break;


        default:
            break;
    }


    return ESP_OK;
}

bool set_time_from_created_at(
    const char *json
)
{
    const char *key = strstr(json, "created_at");

    if (!key) {
        return false;
    }

    const char *value = strchr(key, ':');

    if (!value) {
        return false;
    }

    value++;

    while (*value == ' ' || *value == '\t') {
        value++;
    }

    if (*value != '\"' && *value != '\'') {
        return false;
    }

    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    char sign;
    int offset_hour;
    int offset_minute;

    if (
        sscanf(
            value + 1,
            "%d-%d-%dT%d:%d:%d%c%d:%d",
            &year,
            &month,
            &day,
            &hour,
            &minute,
            &second,
            &sign,
            &offset_hour,
            &offset_minute
        ) != 9
    ) {
        return false;
    }

    struct tm utc_tm = {};
    utc_tm.tm_year = year - 1900;
    utc_tm.tm_mon = month - 1;
    utc_tm.tm_mday = day;
    utc_tm.tm_hour = hour;
    utc_tm.tm_min = minute;
    utc_tm.tm_sec = second;

    time_t timestamp = timegm(&utc_tm);
    int offset_seconds =
        (offset_hour * 60 + offset_minute) * 60;

    if (sign == '-') {
        timestamp += offset_seconds;
    } else if (sign == '+') {
        timestamp -= offset_seconds;
    } else {
        return false;
    }

    struct timeval time_value = {};
    time_value.tv_sec = timestamp;

    if (settimeofday(&time_value, NULL) != 0) {
        return false;
    }

    device_time_initialized = true;

    printf(
        "Device time synchronized from ThingSpeak created_at\n"
    );

    return true;
}


// ============================================================
// WRITE STATUS TO THINGSPEAK
// ============================================================

void write_status_to_thingspeak(
    const MeshMessage *msg
)
{
    char url[256];
    char mesh_summary[96];

    build_mesh_signal_summary(
        mesh_summary,
        sizeof(mesh_summary)
    );

    snprintf(
        url,
        sizeof(url),
        "https://api.thingspeak.com/update?"
        "api_key=%s&field%d=%d&field%d=%u&field5=%d&status="
        "device_%u_state_%u_battery_raw_%u_%s",
        STATUS_THINGSPEAK_API_KEY,
        STATUS_THINGSPEAK_DEVICE_FIELD,
        msg->source_id,
        STATUS_THINGSPEAK_BATTERY_FIELD,
        msg->battery_mv,
        msg->mesh_rssi_dbm,
        msg->source_id,
        msg->command,
        msg->battery_mv,
        mesh_summary
    );

    esp_http_client_config_t config = {};

    config.url = url;
    config.method = HTTP_METHOD_GET;
    config.event_handler = status_http_event;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client =
        esp_http_client_init(&config);

    if (!client) {
        printf(
            "Could not initialize ThingSpeak status client\n"
        );
        return;
    }

    esp_err_t err =
        esp_http_client_perform(client);

    if (err != ESP_OK) {
        printf(
            "ThingSpeak status HTTP error: %s\n",
            esp_err_to_name(err)
        );
        esp_http_client_cleanup(client);
        return;
    }

    int status =
        esp_http_client_get_status_code(client);

    printf(
        "ThingSpeak status write: channel=%d device=%d "
        "state=%d HTTP=%d\n",
        STATUS_THINGSPEAK_CHANNEL_ID,
        msg->source_id,
        msg->command,
        status
    );

    esp_http_client_cleanup(client);
}

void write_head_status_to_status_channel()
{
    time_t now = time(NULL);
    uint16_t battery = read_battery_level();
    int wifi_rssi = -127;
    wifi_ap_record_t ap_info = {};

    if (
        esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK
    ) {
        wifi_rssi = ap_info.rssi;
    }

    char mesh_summary[96];
    build_mesh_signal_summary(
        mesh_summary,
        sizeof(mesh_summary)
    );

    char status_text[128];

    snprintf(
        status_text,
        sizeof(status_text),
        "head_state_%d_battery_raw_%u_epoch_%lld",
        network_state,
        battery,
        (long long)now
    );

    char url[256];

    snprintf(
        url,
        sizeof(url),
        "https://api.thingspeak.com/update?"
        "api_key=%s&field2=%d&field3=%d&field4=%u&field5=%d&status=%s",
        STATUS_THINGSPEAK_API_KEY,
        wifi_rssi,
        HEAD_ID,
        battery,
        strongest_recent_mesh_rssi(),
        status_text
    );

    esp_http_client_config_t config = {};
    config.url = url;
    config.method = HTTP_METHOD_GET;
    config.event_handler = status_http_event;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.timeout_ms = 10000;

    esp_http_client_handle_t client =
        esp_http_client_init(&config);

    if (!client) {
        printf(
            "Could not initialize main channel status client\n"
        );
        return;
    }

    esp_err_t err =
        esp_http_client_perform(client);

    if (err != ESP_OK) {
        printf(
            "Main channel status HTTP error: %s\n",
            esp_err_to_name(err)
        );
        esp_http_client_cleanup(client);
        return;
    }

    printf(
        "Head status written to status channel %d: %s\n",
        STATUS_THINGSPEAK_CHANNEL_ID,
        status_text
    );

    esp_http_client_cleanup(client);
}

void head_status_task(
    void *parameter
)
{
    vTaskDelay(
        pdMS_TO_TICKS(HEAD_STATUS_DELAY_MS)
    );

    while (1) {
        write_head_status_to_status_channel();

        vTaskDelay(
            pdMS_TO_TICKS(HEAD_STATUS_INTERVAL_MS)
        );
    }
}


// ============================================================
// SIMPLE JSON VALUE EXTRACTOR
// ============================================================
//
// Looks for:
//
// "field1":"123"
//
// and returns 123.
//
// This intentionally avoids bringing in a
// full JSON library for this first version.
// ============================================================

int extract_json_int(
    const char *json,
    const char *key,
    int default_value
)
{
    // Start searching in the feeds section.
    // The channel section also contains field1, field2, etc.
    const char *feeds = strstr(json, "\"feeds\"");

    if (!feeds) {
        return default_value;
    }


    char search[64];

    snprintf(
        search,
        sizeof(search),
        "\"%s\"",
        key
    );


    const char *p =
        strstr(feeds, search);

    if (!p) {
        return default_value;
    }


    // Find the colon after the key
    p = strchr(p, ':');

    if (!p) {
        return default_value;
    }

    p++;


    // Skip whitespace
    while (
        *p == ' ' ||
        *p == '\t' ||
        *p == '\r' ||
        *p == '\n'
    ) {
        p++;
    }


    // Handle null
    if (strncmp(p, "null", 4) == 0) {
        return default_value;
    }


    // Handle quoted or unquoted numbers
    if (*p == '"') {
        p++;
    }


    char *endptr;

    long value =
        strtol(p, &endptr, 10);


    if (endptr == p) {
        return default_value;
    }


    return (int)value;
}

// ============================================================
// GET LATEST THINGSPEAK DATA
// ============================================================

bool read_thingspeak()
{
    char url[256];


    snprintf(
        url,
        sizeof(url),

        "https://api.thingspeak.com/"
        "channels/%d/feeds.json?"
        "api_key=%s&results=1",

        THINGSPEAK_CHANNEL_ID,

        THINGSPEAK_API_KEY
    );


    printf("\n");
    printf(
        "Checking ThingSpeak...\n"
    );


    http_buffer_len = 0;

    memset(
        http_buffer,
        0,
        sizeof(http_buffer)
    );


esp_http_client_config_t config = {};

config.url = url;
config.method = HTTP_METHOD_GET;
config.event_handler = thingspeak_http_event;
config.crt_bundle_attach = esp_crt_bundle_attach;
config.timeout_ms = 10000;

    config.url = url;

    config.method =
        HTTP_METHOD_GET;

    config.event_handler =
        thingspeak_http_event;

    config.timeout_ms =
        10000;


    esp_http_client_handle_t client =
        esp_http_client_init(
            &config
        );


    if (!client) {

        printf(
            "Could not initialize "
            "HTTP client\n"
        );

        return false;
    }


    esp_err_t err =
        esp_http_client_perform(
            client
        );


    if (err != ESP_OK) {

        printf(
            "ThingSpeak HTTP error: %s\n",
            esp_err_to_name(err)
        );

        esp_http_client_cleanup(
            client
        );

        return false;
    }


    int status =
        esp_http_client_get_status_code(
            client
        );


    esp_http_client_cleanup(
        client
    );


    if (status != 200) {

        printf(
            "ThingSpeak request failed\n"
        );

        return false;
    }


    set_time_from_created_at(http_buffer);


    // --------------------------------------------------------
    // Get entry ID
    // --------------------------------------------------------

    int entry_id =
        extract_json_int(
            http_buffer,
            "entry_id",
            -1
        );


    if (entry_id < 0) {

        printf(
            "Could not find entry_id\n"
        );

        return false;
    }


    printf(
        "ThingSpeak entry ID = %d\n",
        entry_id
    );


    // Establish the initial baseline without broadcasting it.
    if (!thingspeak_entry_initialized) {

        last_thingspeak_entry = entry_id;
        thingspeak_entry_initialized = true;

        printf(
            "Initial ThingSpeak entry recorded; no update sent\n"
        );

        return false;
    }


    // --------------------------------------------------------
    // Check whether this is new
    // --------------------------------------------------------

    if (
        entry_id ==
        last_thingspeak_entry
    ) {

        printf(
            "No new ThingSpeak data\n"
        );

        return false;
    }


    printf(
        "URL: %s\n",
        url
    );

    printf(
        "ThingSpeak HTTP status: %d\n",
        status
    );

    printf(
        "ThingSpeak response:\n%s\n",
        http_buffer
    );


    // --------------------------------------------------------
    // Read fields 1-8
    // --------------------------------------------------------

    ThingSpeakData data = {};


    data.brightness =
        extract_json_int(
            http_buffer,
            "field1",
            0
        );


    data.color1 =
        extract_json_int(
            http_buffer,
            "field2",
            0
        );


    data.color2 =
        extract_json_int(
            http_buffer,
            "field3",
            0
        );


    data.color3 =
        extract_json_int(
            http_buffer,
            "field4",
            0
        );


    data.pattern =
        extract_json_int(
            http_buffer,
            "field5",
            0
        );


    data.timeOn =
        extract_json_int(
            http_buffer,
            "field6",
            0
        );


    data.sleepTime =
        extract_json_int(
            http_buffer,
            "field7",
            0
        );


    data.fxSpeed =
        extract_json_int(
            http_buffer,
            "field8",
            0
        );


    // Remember entry only after successful
    // extraction.

    last_thingspeak_entry =
        entry_id;


    current_settings =
        data;


    if (
        DEVICE_ID == HEAD_ID &&
        data.brightness > 1
    ) {
        hold_power_for_ms(
            (uint32_t)data.timeOn * 1000U
        );
    }


    send_thingspeak_update(
        &data
    );


    if (
        DEVICE_ID == HEAD_ID &&
        data.pattern >= 600 &&
        data.pattern <= 699
    ) {
        int target_device_id =
            data.pattern - 600;

        if (data.pattern == 650) {
            MeshMessage head_status = {};
            head_status.source_id = HEAD_ID;
            head_status.target_id = HEAD_ID;
            head_status.type = MSG_STATUS_RESPONSE;
            head_status.command = network_state;
            head_status.battery_mv = read_battery_level();
            head_status.mesh_rssi_dbm =
                strongest_recent_mesh_rssi();

            printf(
                "Pattern 650: collecting mesh RSSI status\n"
            );

            queue_status_upload(&head_status);
            send_status_request(BROADCAST_ID);
            hold_power_for_ms(STATUS_POWER_HOLD_MS);
        } else if (target_device_id == HEAD_ID) {
            MeshMessage head_status = {};
            head_status.source_id = HEAD_ID;
            head_status.target_id = HEAD_ID;
            head_status.type = MSG_STATUS_RESPONSE;
            head_status.command = network_state;
            head_status.battery_mv = read_battery_level();
            head_status.mesh_rssi_dbm =
                strongest_recent_mesh_rssi();

            printf(
                "Pattern 600: queueing HEAD status and battery\n"
            );

            queue_status_upload(&head_status);
        } else if (target_device_id < NUM_DEVICES) {
            send_status_request(target_device_id);
            hold_power_for_ms(STATUS_POWER_HOLD_MS);
        } else {
            printf(
                "Pattern %u does not select a remote device\n",
                data.pattern
            );
        }
    }


    return true;
}


// ============================================================
// THINGSPEAK TASK
// ============================================================

void thingspeak_task(
    void *parameter
)
{
    // Give network time to stabilize.

    vTaskDelay(
        pdMS_TO_TICKS(5000)
    );


    while (1) {

        if (
            network_state ==
                NETWORK_READY
        ) {

            read_thingspeak();
        }


        vTaskDelay(
            pdMS_TO_TICKS(
                THINGSPEAK_CHECK_INTERVAL_MS
            )
        );
    }
}


// ============================================================
// APP MAIN
// ============================================================

extern "C" void app_main(void)
{
    esp_log_level_set(
        "esp-x509-crt-bundle",
        ESP_LOG_WARN
    );

    printf("\n\n");

    printf(
        "========================================\n"
    );

    printf(
        "MYFRIEND ESP-NOW MESH\n"
    );

    printf(
        "DEVICE ID = %d\n",
        DEVICE_ID
    );

    printf(
        "========================================\n"
    );


    // --------------------------------------------------------
    // NVS
    // --------------------------------------------------------

    esp_err_t ret =
        nvs_flash_init();


    if (
        ret ==
            ESP_ERR_NVS_NO_FREE_PAGES ||

        ret ==
            ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {

        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );

        ret =
            nvs_flash_init();
    }


    ESP_ERROR_CHECK(ret);


    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    gpio_reset_pin(
        LED_GPIO
    );


    ESP_ERROR_CHECK(
        gpio_set_direction(
            LED_GPIO,
            GPIO_MODE_OUTPUT
        )
    );


    gpio_set_level(
        LED_GPIO,
        0
    );

    gpio_reset_pin(
        POWER_SWITCH_GPIO
    );

    ESP_ERROR_CHECK(
        gpio_set_direction(
            POWER_SWITCH_GPIO,
            GPIO_MODE_OUTPUT
        )
    );

    gpio_set_level(
        POWER_SWITCH_GPIO,
        0
    );

    gpio_reset_pin(
        NEOPIXEL_POWER_PIN
    );

    ESP_ERROR_CHECK(
        gpio_set_direction(
            NEOPIXEL_POWER_PIN,
            GPIO_MODE_OUTPUT
        )
    );

    gpio_set_level(
        NEOPIXEL_POWER_PIN,
        0
    );


    // --------------------------------------------------------
    // RX QUEUE
    // --------------------------------------------------------

    rx_queue =
        xQueueCreate(
            RX_QUEUE_SIZE,
            sizeof(ReceivedPacket)
        );


    if (!rx_queue) {

        printf(
            "ERROR creating RX queue\n"
        );

        return;
    }


    // --------------------------------------------------------
    // NETWORK INITIALIZATION
    // --------------------------------------------------------

    if (
        DEVICE_ID == HEAD_ID
    ) {

        // ----------------------------------------------------
        // HEAD
        // ----------------------------------------------------

        printf(
            "Starting as HEAD\n"
        );

        status_queue =
            xQueueCreate(
                4,
                sizeof(MeshMessage)
            );

        if (!status_queue) {
            printf(
                "ERROR creating status upload queue\n"
            );
            return;
        }


        network_state =
            NETWORK_STARTING;


        initialize_head_wifi();


        if (
            network_channel == 0
        ) {

            printf(
                "ERROR: Head has no "
                "WiFi channel\n"
            );

            return;
        }


        // Create new session.

        session_id =
            esp_random();


        if (session_id == 0) {
            session_id = 1;
        }


        printf(
            "HEAD Session ID = %lu\n",
            (unsigned long)
                session_id
        );


        initialize_esp_now();

        xTaskCreate(
            status_upload_task,
            "status_upload",
            8192,
            NULL,
            3,
            NULL
        );

        xTaskCreate(
            update_retry_task,
            "update_retry",
            4096,
            NULL,
            3,
            NULL
        );

        xTaskCreate(
            power_switch_task,
            "power_switch",
            2048,
            NULL,
            3,
            NULL
        );


        xTaskCreate(
            rx_processing_task,
            "rx_processing",
            4096,
            NULL,
            5,
            NULL
        );


        network_state =
            NETWORK_DISCOVERY;


    } else {

        // ----------------------------------------------------
        // REMOTE
        // ----------------------------------------------------

        printf(
            "Starting as REMOTE\n"
        );


        network_state =
            NETWORK_CHANNEL_DISCOVERY;


        initialize_remote_radio();

        initialize_neopixels();

        led_queue =
            xQueueCreate(
                4,
                sizeof(MeshMessage)
            );

        if (!led_queue) {
            printf(
                "ERROR creating LED update queue\n"
            );
            return;
        }

        xTaskCreate(
            led_update_task,
            "led_update",
            4096,
            NULL,
            4,
            NULL
        );

        xTaskCreate(
            power_switch_task,
            "power_switch",
            2048,
            NULL,
            3,
            NULL
        );


        initialize_esp_now();





// --------------------------------------------------------
// Start RX task BEFORE channel scanning
// --------------------------------------------------------

xTaskCreate(
    rx_processing_task,
    "rx_processing",
    4096,
    NULL,
    5,
    NULL
);


// --------------------------------------------------------
// Scan channels
// --------------------------------------------------------

bool found =
    scan_for_myfriend_channel();


        if (!found) {

            printf("\n");

            printf(
                "========================================\n"
            );

            printf(
                "MYFRIEND NETWORK NOT FOUND\n"
            );

            printf(
                "========================================\n"
            );

            // Keep scanning forever.

            while (1) {

                found =
                    scan_for_myfriend_channel();


                if (found) {
                    break;
                }
            }
        }
    }


    // --------------------------------------------------------
    // Print MAC
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        esp_read_mac(
            local_mac,
            ESP_MAC_WIFI_STA
        )
    );


    printf(
        "My MAC: "
    );

    print_mac(local_mac);

    printf("\n");





    // --------------------------------------------------------
    // Start discovery task
    // --------------------------------------------------------

    xTaskCreate(
        discovery_task,
        "discovery",
        4096,
        NULL,
        4,
        NULL
    );


   // --------------------------------------------------------
// HEAD tasks
// --------------------------------------------------------

if (
    DEVICE_ID == HEAD_ID
) {

    xTaskCreate(
        head_beacon_task,
        "head_beacon",
        4096,
        NULL,
        4,
        NULL
    );


    xTaskCreate(
        thingspeak_task,
        "thingspeak",
        8192,
        NULL,
        3,
        NULL
    );


    xTaskCreate(
        head_status_task,
        "head_status",
        8192,
        NULL,
        3,
        NULL
    );


    // Give the mesh time to form
    vTaskDelay(
        pdMS_TO_TICKS(5000)
    );


    print_neighbors();
    print_mesh_members();


    send_test_message();


    // Discovery is complete
    network_state =
        NETWORK_READY;


    printf("\n");

    printf(
        "========================================\n"
    );

    printf(
        "HEAD NETWORK READY\n"
    );

    printf(
        "WiFi channel: %d\n",
        network_channel
    );

    printf(
        "Session: %lu\n",
        (unsigned long)
            session_id
    );

    printf(
        "========================================\n"
    );


} else {

    // ----------------------------------------------------
    // REMOTE
    // ----------------------------------------------------

    network_state =
        NETWORK_DISCOVERY;


    printf("\n");

    printf(
        "========================================\n"
    );

    printf(
        "REMOTE NETWORK READY\n"
    );

    printf(
        "Device: %d\n",
        DEVICE_ID
    );

    printf(
        "Channel: %d\n",
        network_channel
    );

    printf(
        "Session: %lu\n",
        (unsigned long)
            session_id
    );

    printf(
        "========================================\n"
    );
}

    // --------------------------------------------------------
    // Main loop
    // --------------------------------------------------------

    while (1) {

        vTaskDelay(
            pdMS_TO_TICKS(1000)
        );
    }
}