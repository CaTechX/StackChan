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
#include <cctype>
#include <string>

#include <esp_log.h>
#include <esp_netif.h>
#include <mqtt_client.h>
#include <cJSON.h>
#include <settings.h>

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
    cJSON_AddStringToObject(dev, "sw_version",  "1.4.0");
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
}

/* ---------------------------------------------------------------------------
 * State reporting
 * ------------------------------------------------------------------------- */

static void publish_battery_state(esp_mqtt_client_handle_t client)
{
    int  level    = GetHAL().getBatteryLevel();
    bool charging = GetHAL().isBatteryCharging();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "battery",  level);
    cJSON_AddBoolToObject(root,   "charging", charging);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(client, s_state_topic, payload, 0, 0, 0);
        free(payload);
    }
    cJSON_Delete(root);

    ESP_LOGD(TAG, "Battery state: %d%%, charging=%d", level, charging);
}

/* ---------------------------------------------------------------------------
 * MQTT event callback
 * ------------------------------------------------------------------------- */

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

        /* Send initial battery state */
        publish_battery_state(client);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "Disconnected from MQTT broker");
        s_mqtt_connected = false;
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Message received: topic=%.*s, data=%.*s",
                 event->topic_len, event->topic,
                 event->data_len, event->data);
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
    esp_mqtt_client_start(s_mqtt_client);

    /* ---- Main loop: periodic battery reporting ---- */
    uint32_t     last_publish    = 0;
    const uint32_t publish_interval = pdMS_TO_TICKS(30 * 1000);  /* 30 s */

    while (s_task_running) {
        uint32_t now = xTaskGetTickCount();
        if (s_mqtt_connected && (now - last_publish >= publish_interval)) {
            publish_battery_state(s_mqtt_client);
            last_publish = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    if (s_mqtt_task == nullptr) return;
    s_task_running = false;
    /* Wait briefly for the task to self-delete */
    vTaskDelay(pdMS_TO_TICKS(50));
    s_mqtt_task = nullptr;
}
