/*
 * hal_mqtt.cpp — MQTT bridge to Home Assistant
 *
 * Design overview:
 *   Runs as a dedicated FreeRTOS task on core 1 alongside _stackchan_update_task.
 *   - Reads HA broker config from NVS via Settings("mqtt", ...)
 *   - Connects to the broker using ESP-IDF's esp_mqtt_client
 *   - Publishes HA Discovery config topics (retained) so HA auto-discovers sensors
 *   - Publishes battery level and charging state every 30 s
 *   - Publishes "online"/"offline" via LWT for availability tracking
 *
 * Inter-task safety:
 *   - GetHAL().getBatteryLevel() / isBatteryCharging() only read cached values:
 *     they are safe to call from any task.
 *   - No direct I2C/SPI access from this task.
 */

#include "hal.h"
#include "hal_mqtt.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <string>

#include <esp_log.h>
#include <esp_netif.h>
#include <mqtt_client.h>
#include <cJSON.h>
#include <settings.h>
#include "board/hal_bridge.h"
#include "drivers/LTR553/LTR553.h"
#include "drivers/Si12T/Si12T.h"

static const char *TAG = "HalMqtt";

/* ---------------------------------------------------------------------------
 * Internal state (file-scope)
 * ------------------------------------------------------------------------- */
static TaskHandle_t            s_mqtt_task      = nullptr;
static esp_mqtt_client_handle_t s_mqtt_client   = nullptr;
static volatile bool           s_task_running   = false;
static volatile bool           s_mqtt_connected = false;

static char s_device_id[32] = {0};     /* e.g. "stackchan_aabbccddeeff" */
static char s_state_topic[64] = {0};   /* e.g. "stackchan_aabbccddeeff/state" */
static char s_avail_topic[64] = {0};   /* e.g. "stackchan_aabbccddeeff/availability" */

/* Per-sensor state topics for trigger-based sensors */
static char s_topic_touch[3][64]  = {};
static char s_topic_proximity[64] = {};

/* Backlight control topics */
static char s_topic_bl_state[64];         /* stackchan_xxx/backlight/state         */
static char s_topic_bl_set[64];           /* stackchan_xxx/backlight/set           */
static char s_topic_bl_brightness_state[64]; /* stackchan_xxx/backlight/brightness/state */
static char s_topic_bl_brightness_set[64];   /* stackchan_xxx/backlight/brightness/set   */

/* Backlight state cache */
static uint8_t s_cache_backlight = 255;        /* cached brightness (0-255) for cache/ON restore */
static uint8_t s_last_backlight_brightness = 255; /* last non-zero brightness (0-255) — persists across ON/OFF */

/* Trigger sensor state cache + timing */
static uint8_t  s_cache_touch[3]      = {0xFF, 0xFF, 0xFF};
static uint16_t s_cache_proximity     = 0xFFFF;
static uint32_t s_last_trigger_publish = 0;
static const uint32_t TRIGGER_MIN_MS  = pdMS_TO_TICKS(500);

/* ---------------------------------------------------------------------------
 * Sensor state (shared across modules)
 * ------------------------------------------------------------------------- */
static LTR553 *s_ltr553        = nullptr;
static bool    s_ltr553_ok     = false;

static si12t_handle_t s_si12t    = nullptr;
static bool           s_si12t_ok = false;

/* ---------------------------------------------------------------------------
 * Config helpers (NVS via Settings)
 * ------------------------------------------------------------------------- */

struct HalMqttConfig {
    std::string broker;
    uint16_t    port;
    std::string username;
    std::string password;
    bool        enabled;
};

static bool load_config(HalMqttConfig &cfg)
{
    Settings nvs("mqtt", false);

    cfg.broker  = nvs.GetString("broker");
    cfg.port    = static_cast<uint16_t>(nvs.GetInt("port", 1883));
    cfg.username = nvs.GetString("username");
    cfg.password = nvs.GetString("password");
    cfg.enabled = nvs.GetBool("enabled", false);

    if (cfg.broker.empty()) {
        ESP_LOGW(TAG, "MQTT broker not configured (set in NVS namespace 'mqtt', key 'broker')");
        return false;
    }

    if (!cfg.enabled) {
        ESP_LOGW(TAG, "MQTT is disabled (set NVS 'mqtt'/'enabled' = true)");
        return false;
    }

    ESP_LOGI(TAG, "Config loaded: %s:%d", cfg.broker.c_str(), cfg.port);
    return true;
}

/* ---------------------------------------------------------------------------
 * HA Discovery  (Home Assistant MQTT Discovery v1.0)
 * ------------------------------------------------------------------------- */

/* forward declarations for sensor discovery functions defined below */
static void publish_light_discovery(esp_mqtt_client_handle_t client);
static void publish_proximity_discovery(esp_mqtt_client_handle_t client);
static void publish_touch_discovery(esp_mqtt_client_handle_t client, int ch);

static void publish_discovery(esp_mqtt_client_handle_t client)
{
    char topic[128];
    char unique_id[64];

    /* ---- Battery (sensor) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/%s/battery/config", s_device_id);
    snprintf(unique_id, sizeof(unique_id), "%s_battery", s_device_id);

    cJSON *batt = cJSON_CreateObject();
    cJSON_AddStringToObject(batt, "name",             "StackChan Battery");
    cJSON_AddStringToObject(batt, "state_topic",      s_state_topic);
    cJSON_AddStringToObject(batt, "value_template",   "{{ value_json.battery }}");
    cJSON_AddStringToObject(batt, "unit_of_measurement", "%");
    cJSON_AddStringToObject(batt, "device_class",     "battery");
    cJSON_AddStringToObject(batt, "unique_id",        unique_id);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddStringToObject(dev, "sw_version",  "1.4.1");
    cJSON_AddItemToObject(batt, "device", dev);

    char *payload = cJSON_PrintUnformatted(batt);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(batt);

    /* ---- Charging (binary_sensor) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/binary_sensor/%s/charging/config", s_device_id);
    snprintf(unique_id, sizeof(unique_id), "%s_charging", s_device_id);

    cJSON *chg = cJSON_CreateObject();
    cJSON_AddStringToObject(chg, "name",           "StackChan Charging");
    cJSON_AddStringToObject(chg, "state_topic",    s_state_topic);
    cJSON_AddStringToObject(chg, "value_template", "{{ value_json.charging }}");
    cJSON_AddStringToObject(chg, "device_class",   "battery_charging");
    cJSON_AddStringToObject(chg, "payload_on",     "true");
    cJSON_AddStringToObject(chg, "payload_off",    "false");
    cJSON_AddStringToObject(chg, "unique_id",      unique_id);

    dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddItemToObject(chg, "device", dev);

    payload = cJSON_PrintUnformatted(chg);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(chg);

    ESP_LOGI(TAG, "HA Discovery published for %s", s_device_id);

    /* ---- Sensor discoveries ---- */
    publish_light_discovery(client);
    publish_proximity_discovery(client);
    for (int i = 0; i < 3; i++) {
        publish_touch_discovery(client, i + 1);
    }
}

/* ---------------------------------------------------------------------------
 * Periodic stable-sensor state  (combined JSON, 30 s interval)
 * ------------------------------------------------------------------------- */

static void publish_stable_state(esp_mqtt_client_handle_t client)
{
    /* ---- Battery ---- */
    int  level    = GetHAL().getBatteryLevel();
    bool charging = GetHAL().isBatteryCharging();

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddNumberToObject(root, "battery",  level);
    cJSON_AddBoolToObject(root,   "charging", charging);

    /* ---- LTR-553 ALS (ambient light) — stable, included in periodic ---- */
    uint16_t ch0 = 0, ch1 = 0;
    if (s_ltr553_ok && s_ltr553->readALS(ch0, ch1)) {
        float lux = s_ltr553->calcLux(ch0, ch1);
        cJSON_AddNumberToObject(root, "ambient_light", (double)std::round(lux));
    }

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, s_state_topic, payload, 0, 0, 0);
        free(payload);
    }
    cJSON_Delete(root);

    ESP_LOGD(TAG, "Stable state published: battery=%d%%, charging=%d",
             level, charging);
}

/* ---------------------------------------------------------------------------
 * Trigger-based sensor polling  (per-sensor topics, publish on change)
 * ------------------------------------------------------------------------- */

static void publish_initial_trigger_state(esp_mqtt_client_handle_t client)
{
    /* Publish current cached values so HA has an initial state immediately */
    for (int i = 0; i < 3; i++) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%u", s_cache_touch[i]);
        esp_mqtt_client_publish(client, s_topic_touch[i], buf, 0, 0, 0);
    }
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%u", s_cache_proximity);
        esp_mqtt_client_publish(client, s_topic_proximity, buf, 0, 0, 0);
    }
    ESP_LOGD(TAG, "Initial trigger state published");
}

static void poll_triggers(esp_mqtt_client_handle_t client)
{
    uint32_t now = xTaskGetTickCount();
    if (now - s_last_trigger_publish < TRIGGER_MIN_MS) return;

    bool published = false;

    /* ---- Si12T touch (all 3 channels in one I2C read) ---- */
    if (s_si12t_ok) {
        uint8_t touch_result = 0;
        if (si12t_read_touch_result(s_si12t, &touch_result) == ESP_OK) {
            uint8_t parsed[3] = {0};
            si12t_parse_touch_result_to(touch_result, parsed);
            for (int i = 0; i < 3; i++) {
                if (parsed[i] != s_cache_touch[i]) {
                    s_cache_touch[i] = parsed[i];
                    char buf[4];
                    snprintf(buf, sizeof(buf), "%u", parsed[i]);
                    esp_mqtt_client_publish(client, s_topic_touch[i], buf, 0, 0, 0);
                    published = true;
                }
            }
        }
    }

    /* ---- LTR-553 PS (proximity) ---- */
    if (s_ltr553_ok) {
        uint16_t ps_data = 0;
        if (s_ltr553->readPS(ps_data) && ps_data != s_cache_proximity) {
            s_cache_proximity = ps_data;
            char buf[16];
            snprintf(buf, sizeof(buf), "%u", ps_data);
            esp_mqtt_client_publish(client, s_topic_proximity, buf, 0, 0, 0);
            published = true;
        }
    }

    if (published) {
        s_last_trigger_publish = now;
    }
}

/* ---------------------------------------------------------------------------
 * HA sensor discovery — LTR-553 and Si12T
 * ------------------------------------------------------------------------- */

static void publish_light_discovery(esp_mqtt_client_handle_t client)
{
    char topic[128], unique_id[64];
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/%s_ambient_light/config", s_device_id);
    snprintf(unique_id, sizeof(unique_id), "%s_ambient_light", s_device_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",                "StackChan Ambient Light");
    cJSON_AddStringToObject(root, "state_topic",         s_state_topic);
    cJSON_AddStringToObject(root, "value_template",      "{{ value_json.ambient_light }}");
    cJSON_AddStringToObject(root, "unit_of_measurement", "lx");
    cJSON_AddStringToObject(root, "device_class",        "illuminance");
    cJSON_AddStringToObject(root, "unique_id",           unique_id);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);
}

static void publish_proximity_discovery(esp_mqtt_client_handle_t client)
{
    char topic[128], unique_id[64];
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/%s_proximity/config", s_device_id);
    snprintf(unique_id, sizeof(unique_id), "%s_proximity", s_device_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",                 "StackChan Proximity");
    cJSON_AddStringToObject(root, "state_topic",          s_topic_proximity);
    cJSON_AddStringToObject(root, "unit_of_measurement", "counts");
    cJSON_AddStringToObject(root, "unique_id",            unique_id);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);
}

static void publish_touch_discovery(esp_mqtt_client_handle_t client, int ch)
{
    char topic[128], unique_id[64];
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/%s_touch_%d/config", s_device_id, ch);
    snprintf(unique_id, sizeof(unique_id), "%s_touch_%d", s_device_id, ch);

    char name_buf[48];
    snprintf(name_buf, sizeof(name_buf), "Touch Ch. %d", ch);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",       name_buf);
    cJSON_AddStringToObject(root, "state_topic", s_topic_touch[ch - 1]);
    cJSON_AddStringToObject(root, "unique_id",  unique_id);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);
}

/* ===========================================================================
 * Backlight (Light entity — brightness + power)
 * =========================================================================== */

static void init_backlight_topics()
{
    snprintf(s_topic_bl_state,          sizeof(s_topic_bl_state),          "%s/backlight/state",            s_device_id);
    snprintf(s_topic_bl_set,            sizeof(s_topic_bl_set),            "%s/backlight/set",              s_device_id);
    snprintf(s_topic_bl_brightness_state, sizeof(s_topic_bl_brightness_state), "%s/backlight/brightness/state", s_device_id);
    snprintf(s_topic_bl_brightness_set,   sizeof(s_topic_bl_brightness_set),   "%s/backlight/brightness/set",   s_device_id);
}

/**
 * HA MQTT Light discovery — gives a brightness slider + on/off toggle in HA.
 * Because the HAL brightness is already 0-255 we use it 1:1.
 */
static void publish_backlight_discovery(esp_mqtt_client_handle_t client)
{
    char topic[128], unique_id[64];
    snprintf(topic, sizeof(topic), "homeassistant/light/%s_backlight/config", s_device_id);
    snprintf(unique_id, sizeof(unique_id), "%s_backlight", s_device_id);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",                    "StackChan Backlight");
    cJSON_AddStringToObject(root, "unique_id",               unique_id);
    cJSON_AddStringToObject(root, "state_topic",             s_topic_bl_state);
    cJSON_AddStringToObject(root, "command_topic",           s_topic_bl_set);
    cJSON_AddStringToObject(root, "brightness_state_topic",  s_topic_bl_brightness_state);
    cJSON_AddStringToObject(root, "brightness_command_topic", s_topic_bl_brightness_set);
    cJSON_AddStringToObject(root, "payload_on",              "ON");
    cJSON_AddStringToObject(root, "payload_off",             "OFF");
    cJSON_AddNumberToObject(root, "brightness_scale",        255);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", s_device_id);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);
}

/** Publish both state (ON/OFF) + brightness to HA (retained for reconnection).
 *  When b_known is supplied, use it directly instead of reading back from HAL
 *  (avoids picking up stale brightness_ during the backlight's gradual transition).
 *
 *  ⚠ HAL reads back brightness in 0-100 range.  The MQTT layer operates in
 *     0-255 (HA standard).  Reverse scale when falling back to the getter. */
static void publish_backlight_state(esp_mqtt_client_handle_t client, uint8_t b_known = UINT8_MAX)
{
    uint8_t b;
    if (b_known != UINT8_MAX) {
        b = b_known;  /* already in the 0-255 HA range */
    } else {
        /* Read back from HAL (0-100) and scale up to 0-255 for HA */
        b = (uint8_t)(((uint16_t)GetHAL().getBackLightBrightness() * 255 + 50) / 100);
    }
    s_cache_backlight = b;

    /* Retain both so HA re-reads the latest state on reconnect/refresh */
    esp_mqtt_client_publish(client, s_topic_bl_state, (b > 0) ? "ON" : "OFF", 0, 1, 1);

    char buf[8];
    snprintf(buf, sizeof(buf), "%u", b);
    esp_mqtt_client_publish(client, s_topic_bl_brightness_state, buf, 0, 1, 1);
}

/**
 * Handle incoming commands for backlight.
 *   /backlight/set           →  "ON" / "OFF"  (or JSON {"state":…, "brightness":…})
 *   /backlight/brightness/set →  "128"  (raw number string)
 *
 * ⚠ HA range: 0-255   HAL/hardware range: 0-100
 *    Scaling is done here before passing to the HAL.
 */
static void handle_backlight_command(esp_mqtt_client_handle_t client,
                                     const char *topic, const char *data)
{
    bool on_off_topic = (strcmp(topic, s_topic_bl_set) == 0);
    bool bri_topic    = (strcmp(topic, s_topic_bl_brightness_set) == 0);

    if (!on_off_topic && !bri_topic) return;

    uint8_t bri = s_cache_backlight;  /* start from current */

    if (bri_topic) {
        /* Brightness-only command: "128" */
        int v = atoi(data);
        if (v >= 0 && v <= 255) {
            bri = (uint8_t)v;
            if (bri > 0) s_last_backlight_brightness = bri;
        }
    } else {
        /* ON/OFF / JSON command */
        if (data[0] == '{') {
            /* JSON payload from HA — extract "state" and optionally "brightness" */
            cJSON *json = cJSON_Parse(data);
            if (json) {
                cJSON *s = cJSON_GetObjectItem(json, "state");
                if (cJSON_IsString(s)) {
                    if (strcmp(s->valuestring, "OFF") == 0) {
                        bri = 0;
                    } else {
                        /* ON — restore to last non-zero brightness */
                        bri = (s_cache_backlight > 0)
                                  ? s_cache_backlight
                                  : s_last_backlight_brightness;
                    }
                }
                cJSON *b = cJSON_GetObjectItem(json, "brightness");
                if (cJSON_IsNumber(b)) {
                    int v = b->valueint;
                    if (v >= 0 && v <= 255) {
                        bri = (uint8_t)v;
                        if (bri > 0) s_last_backlight_brightness = bri;
                    }
                }
                cJSON_Delete(json);
            }
        } else {
            /* Plain "ON" / "OFF" */
            if (strcmp(data, "OFF") == 0) {
                bri = 0;
            } else if (strcmp(data, "ON") == 0) {
                /* ON — restore to last non-zero brightness */
                bri = (s_cache_backlight > 0)
                          ? s_cache_backlight
                          : s_last_backlight_brightness;
            }
        }
    }

    /* Scale from HA's 0-255 to the HAL/Backlight 0-100 range */
    uint8_t bri_scaled = (uint8_t)(((uint16_t)bri * 100 + 127) / 255);

    /* Apply to hardware */
    GetHAL().setBackLightBrightness(bri_scaled);

    /* Publish state — pass the HA-range bri directly so we don't
       read back a stale brightness_ during the gradual PWM transition */
    publish_backlight_state(client, bri);
}

/* ===========================================================================
 * Command dispatcher — routes by exact topic match
 * =========================================================================== */

static void handle_command(esp_mqtt_client_handle_t client,
                           const char *topic, int topic_len,
                           const char *data, int data_len)
{
    if (topic == nullptr || data == nullptr || topic_len <= 0) {
        return;
    }

    /* Build null-terminated copies (topics & data are well under 128 bytes) */
    char topic_nt[128];
    char data_nt[128];
    size_t tlen = (size_t)topic_len < sizeof(topic_nt) - 1 ? (size_t)topic_len : sizeof(topic_nt) - 1;
    size_t dlen = (size_t)data_len < sizeof(data_nt) - 1 ? (size_t)data_len : sizeof(data_nt) - 1;
    memcpy(topic_nt, topic, tlen);
    topic_nt[tlen] = '\0';
    memcpy(data_nt, data, dlen);
    data_nt[dlen] = '\0';

    /* Backlight */
    if (strcmp(topic_nt, s_topic_bl_set) == 0
     || strcmp(topic_nt, s_topic_bl_brightness_set) == 0) {
        handle_backlight_command(client, topic_nt, data_nt);
        return;
    }

    /* Future components will add their branches here:
     * if (strcmp(topic_nt, s_topic_servo_pitch_set) == 0) ...
     * if (strcmp(topic_nt, s_topic_expression_set) == 0) ...
     * if (strcmp(topic_nt, s_topic_xiaozhi_set) == 0) ...
     */

    ESP_LOGD(TAG, "Unhandled command topic: %.*s -> %.*s",
             topic_len, topic, data_len, data);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (event == nullptr) return;
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id) {

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to MQTT broker");
        s_mqtt_connected = true;

        /* Publish online availability (retained) */
        esp_mqtt_client_publish(client, s_avail_topic, "online", 0, 1, 1);

        /* Publish HA discovery (retained) so HA auto-discovers our entities */
        publish_discovery(client);

        /* Send initial stable state (battery, charging, ambient light) */
        publish_stable_state(client);

        /* Send initial trigger states (touch channels, proximity) */
        publish_initial_trigger_state(client);

        /* ---- Controllable components ---- */
        publish_backlight_discovery(client);
        publish_backlight_state(client);
        esp_mqtt_client_subscribe(client, s_topic_bl_set, 1);
        esp_mqtt_client_subscribe(client, s_topic_bl_brightness_set, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        s_mqtt_connected = false;
        break;

    case MQTT_EVENT_DATA:
        handle_command(client, event->topic, event->topic_len,
                       event->data, event->data_len);
        /* TODO: handle cover/brightness/volume commands in a later iteration */
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error event");
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * MQTT background task  (core 1, priority 3)
 * ------------------------------------------------------------------------- */

static void mqtt_task(void *arg)
{
    ESP_LOGI(TAG, "Task started on core %d", xPortGetCoreID());

    /* ---- Load config from NVS ---- */
    HalMqttConfig cfg;
    if (!load_config(cfg)) {
        ESP_LOGE(TAG, "MQTT not configured — task will exit. "
                      "Set NVS values via Settings(\"mqtt\", ...) or serial command.");
        s_task_running = false;
        vTaskDelete(nullptr);
        return;
    }

    /* ---- Build MQTT URI ---- */
    char uri[256];
    snprintf(uri, sizeof(uri), "mqtt://%s:%d", cfg.broker.c_str(), cfg.port);
    ESP_LOGI(TAG, "Connecting to %s", uri);

    /* ---- Wait for network (WiFi) to be up with an IP ---- */
    esp_netif_t *netif = nullptr;
    for (int i = 0; i < 120; i++) {
        netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != nullptr) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK
                && ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "Network ready (" IPSTR ")", IP2STR(&ip_info.ip));
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if (netif == nullptr) {
        ESP_LOGE(TAG, "Network not available after 60 s, aborting");
        s_task_running = false;
        vTaskDelete(nullptr);
        return;
    }

    /* ---- Initialize onboard sensors ---- */

    // LTR-553ALS-WA ambient-light & proximity sensor (I2C addr 0x23)
    auto i2c_bus = hal_bridge::board_get_i2c_bus();
    if (i2c_bus) {
        s_ltr553 = new LTR553(i2c_bus, 0x23);
        if (s_ltr553) {
            s_ltr553_ok = s_ltr553->begin();
            if (!s_ltr553_ok) {
                ESP_LOGW(TAG, "LTR-553 init failed — sensor unavailable");
            }
        }
    } else {
        ESP_LOGW(TAG, "No I2C bus — LTR-553 + Si12T unavailable");
    }

    // Si12T capacitive touch (I2C addr 0x68)
    // NOTE: The Si12T is *also* initialised and configured by hal_head_touch.cpp
    // (head_touch_init → si12t_setup with TYPE_LOW / LEVEL_3).  That call
    // owns the sensor configuration.  We obtain a *separate handle* here so
    // poll_triggers() can read the OUTPUT register without interfering with
    // the head-touch sensitivity / channel settings.  Do NOT call si12t_setup()
    // in this task — that would overwrite the head_touch config and may cause
    // spurious touch triggers.
    if (i2c_bus) {
        si12t_config_t si12t_cfg = {};
        si12t_cfg.i2c_bus  = i2c_bus;
        si12t_cfg.dev_addr = SI12T_GND_ADDRESS;
        if (si12t_init(&si12t_cfg, &s_si12t) == ESP_OK) {
            s_si12t_ok = true;
            ESP_LOGI(TAG, "Si12T touch sensor initialised (read-only handle)");
        } else {
            ESP_LOGW(TAG, "Si12T init failed — touch sensor unavailable");
        }
    }

    /* ---- Configure MQTT client (ESP-IDF v5.x style) ---- */
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = uri;
    mqtt_cfg.credentials.client_id = s_device_id;

    if (!cfg.username.empty()) {
        mqtt_cfg.credentials.username = cfg.username.c_str();
        if (!cfg.password.empty()) {
            mqtt_cfg.credentials.authentication.password = cfg.password.c_str();
        }
    }

    mqtt_cfg.session.keepalive           = 60;
    mqtt_cfg.session.disable_clean_session = true;

    /* Last Will: publishes "offline" if the device disconnects unexpectedly */
    mqtt_cfg.session.last_will.topic   = s_avail_topic;
    mqtt_cfg.session.last_will.msg     = "offline";
    mqtt_cfg.session.last_will.msg_len = 7;
    mqtt_cfg.session.last_will.qos     = 1;
    mqtt_cfg.session.last_will.retain  = true;

    mqtt_cfg.buffer.size = 2048;

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == nullptr) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        s_task_running = false;
        vTaskDelete(nullptr);
        return;
    }

    esp_mqtt_client_register_event(s_mqtt_client, MQTT_EVENT_ANY,
                                   mqtt_event_handler, nullptr);

    /* ---- Initialise per-sensor state topics (must be before client start!) ---- */
    for (int i = 0; i < 3; i++) {
        snprintf(s_topic_touch[i], sizeof(s_topic_touch[i]),
                 "%s/touch_%d/state", s_device_id, i + 1);
    }
    snprintf(s_topic_proximity, sizeof(s_topic_proximity),
             "%s/proximity/state", s_device_id);

    init_backlight_topics();

    esp_mqtt_client_start(s_mqtt_client);

    /* ---- Main loop: periodic + trigger-based sensor reporting ---- */
    uint32_t     last_publish       = 0;
    const uint32_t publish_interval = pdMS_TO_TICKS(2 * 1000);  /* 30 s */

    /* Seed trigger caches so first poll doesn't false-trigger */
    if (s_si12t_ok) {
        uint8_t touch_result = 0;
        if (si12t_read_touch_result(s_si12t, &touch_result) == ESP_OK) {
            si12t_parse_touch_result_to(touch_result, s_cache_touch);
        }
    }
    if (s_ltr553_ok) {
        s_ltr553->readPS(s_cache_proximity);
    }

    while (s_task_running) {
        uint32_t now = xTaskGetTickCount();

        /* ---- Periodic stable-sensor publish (every 30 s) ---- */
        if (s_mqtt_connected && (now - last_publish >= publish_interval)) {
            publish_stable_state(s_mqtt_client);
            publish_backlight_state(s_mqtt_client);  /* sync backlight retained state */
            last_publish = now;
        }

        /* ---- Trigger-based: poll touch + proximity, publish on change ---- */
        if (s_mqtt_connected) {
            poll_triggers(s_mqtt_client);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    /* ---- Cleanup ---- */
    ESP_LOGI(TAG, "Stopping MQTT client");
    esp_mqtt_client_stop(s_mqtt_client);
    esp_mqtt_client_destroy(s_mqtt_client);
    s_mqtt_client    = nullptr;
    s_mqtt_connected = false;
    vTaskDelete(nullptr);
}

/* ===========================================================================
 * Public API
 * =========================================================================== */

void Hal::startMqtt()
{
    if (s_mqtt_task != nullptr) {
        ESP_LOGW(TAG, "MQTT task is already running");
        return;
    }

    /* Build stable device ID from the factory MAC address */
    std::string mac = getFactoryMacString("");
    for (auto &c : mac) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

    snprintf(s_device_id,  sizeof(s_device_id),  "stackchan_%s", mac.c_str());
    snprintf(s_state_topic, sizeof(s_state_topic), "%s/state",       s_device_id);
    snprintf(s_avail_topic, sizeof(s_avail_topic), "%s/availability", s_device_id);

    ESP_LOGI(TAG, "Device ID: %s",         s_device_id);
    ESP_LOGI(TAG, "State topic: %s",       s_state_topic);
    ESP_LOGI(TAG, "Availability topic: %s", s_avail_topic);

    s_task_running = true;
    BaseType_t ret = xTaskCreatePinnedToCore(
        mqtt_task,
        "mqtt",
        4096,          /* stack size */
        nullptr,       /* arg */
        3,             /* priority — same as _stackchan_update_task */
        &s_mqtt_task,
        1              /* core 1 — away from WiFi/LwIP on core 0 */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MQTT task (err=%d)", ret);
        s_task_running = false;
        s_mqtt_task    = nullptr;
    }
}

void hal_mqtt_stop()
{
    /* Release sensor resources */
    if (s_si12t) {
        si12t_delete(s_si12t);
        s_si12t    = nullptr;
        s_si12t_ok = false;
    }
    delete s_ltr553;
    s_ltr553    = nullptr;
    s_ltr553_ok = false;

    if (s_mqtt_task == nullptr) return;
    s_task_running = false;
    /* Wait briefly for the task to self-delete */
    vTaskDelay(pdMS_TO_TICKS(50));
    s_mqtt_task = nullptr;
}
