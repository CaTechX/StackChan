/*
 * hal_auto_start.cpp — HA MQTT Switch for auto-starting xiaozhi on boot
 *
 * Design:
 *   Persistent NVS flag (namespace "xiaozhi", key "auto_start") controls
 *   whether the firmware automatically enters xiaozhi mode after power-on.
 *
 *   The flag is writable from Home Assistant via an MQTT switch entity and
 *   is checked in app_main() after all hardware initialisation.
 *
 *   When the switch is turned ON the change takes effect on the *next* boot
 *   (it does not start xiaozhi immediately at runtime).
 *
 * NVS namespace:
 *   "xiaozhi" — same namespace used by hal_bridge for idle-shutdown config,
 *   so all xiaozhi-related settings live in one place.
 */

#include "hal_auto_start.h"
#include <settings.h>
#include <cJSON.h>
#include <esp_log.h>
#include <cstring>
#include <cstdio>

static const char *TAG = "HalAutoStart";

/* ---------------------------------------------------------------------------
 *  Static state
 * --------------------------------------------------------------------------- */
static esp_mqtt_client_handle_t s_client      = nullptr;
static constexpr const char*    DEVICE_ID     = "stackchan";
static char s_topic_set[48];
static char s_topic_status[48];

static constexpr const char *NVS_NS   = "xiaozhi";
static constexpr const char *NVS_KEY  = "auto_start";

/* ---------------------------------------------------------------------------
 *  NVS helpers
 * --------------------------------------------------------------------------- */

bool hal_auto_start_is_enabled()
{
    Settings settings(NVS_NS, false);
    return settings.GetBool(NVS_KEY, false);
}

static void set_enabled(bool enabled)
{
    Settings settings(NVS_NS, true);
    settings.SetBool(NVS_KEY, enabled);
    ESP_LOGI(TAG, "NVS: auto_start → %s", enabled ? "ON" : "OFF");
}

/* ---------------------------------------------------------------------------
 *  MQTT helpers
 * --------------------------------------------------------------------------- */

static void publish_state()
{
    bool enabled = hal_auto_start_is_enabled();
    esp_mqtt_client_publish(s_client, s_topic_status,
                            enabled ? "ON" : "OFF", 0, 1, 1);
}

static void publish_discovery()
{
    char unique_id[64], topic[128];
    snprintf(unique_id, sizeof(unique_id), "%s_auto_xiaozhi", DEVICE_ID);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",         "Auto-start xiaozhi");
    cJSON_AddStringToObject(root, "unique_id",    unique_id);
    cJSON_AddStringToObject(root, "state_topic",  s_topic_status);
    cJSON_AddStringToObject(root, "command_topic", s_topic_set);
    cJSON_AddStringToObject(root, "payload_on",   "ON");
    cJSON_AddStringToObject(root, "payload_off",  "OFF");

    /* Group under the same StackChan device */
    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers",  DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",         "StackChan");
    cJSON_AddStringToObject(dev, "model",        "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    snprintf(topic, sizeof(topic),
             "homeassistant/switch/%s/config", unique_id);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Discovery published: %s", topic);
}

/* ---------------------------------------------------------------------------
 *  Command handler
 * --------------------------------------------------------------------------- */
static void handle_command(const char *data)
{
    if (!data || !data[0]) return;

    bool new_val = (strcmp(data, "ON") == 0);
    bool current = hal_auto_start_is_enabled();

    if (new_val != current)
        set_enabled(new_val);

    publish_state();
}

/* ---------------------------------------------------------------------------
 *  Public API
 * --------------------------------------------------------------------------- */

void hal_auto_start_init(esp_mqtt_client_handle_t client)
{
    s_client = client;

    snprintf(s_topic_set,    sizeof(s_topic_set),
             "%s/auto_xiaozhi/set",    DEVICE_ID);
    snprintf(s_topic_status, sizeof(s_topic_status),
             "%s/auto_xiaozhi/status", DEVICE_ID);

    ESP_LOGI(TAG, "Init — set=%s  status=%s", s_topic_set, s_topic_status);
}

void hal_auto_start_on_connected(esp_mqtt_client_handle_t client)
{
    s_client = client;

    publish_discovery();
    esp_mqtt_client_subscribe(client, s_topic_set, 1);
    publish_state();

    ESP_LOGI(TAG, "Connected — subscribed to %s", s_topic_set);
}

bool hal_auto_start_handle_command(esp_mqtt_client_handle_t client,
                                   const char *topic, const char *data)
{
    if (!topic || !data) return false;
    if (strcmp(topic, s_topic_set) == 0) {
        s_client = client;
        handle_command(data);
        return true;
    }
    return false;
}

void hal_auto_start_stop()
{
    s_client = nullptr;
}
