/*
 * hal_led.cpp — HA MQTT Light entity for the 12-LED NeoPixel bar
 *
 * Design intent:
 *   Expose the NeoPixel strip as a HA light entity (RGB + brightness + effects)
 *   while cooperating with xiaozhi's own LED 0 status indicator.
 *
 * LED arbitration:
 *   - LED 0  ← xiaozhi status (green when listening, blue when speaking,
 *                               off when standby)
 *   - LEDs 1-11 ← HA light (all 12 when xiaozhi is standby)
 *   - hal_led_set_xiaozhi_active() toggles a flag; when true, apply/skip
 *     logic leaves LED 0 untouched so xiaozhi's direct HAL calls show through.
 *
 * Safety notes (multi-task I2C):
 *   setRgbColor() is a buffer write (safe from any task).  refreshRgb() drives
 *   I2C — the underlying i2c_master_transmit has transport-level serialisation.
 *   A brief race between hal_led_tick() and the StackChan update loop may
 *   cause a single-LED glitch for one frame; acceptable for V1.
 */

#include "hal_led.h"
#include "hal.h"                  // for GetHAL()

#include <cstring>
#include <cstdio>
#include <cstdint>

#include <esp_log.h>
#include <cJSON.h>

static const char *TAG = "HalLed";

/* ===========================================================================
 *  Static state
 * =========================================================================== */
static esp_mqtt_client_handle_t s_client    = nullptr;

/* MQTT topic names (built from device_id at init) */
static constexpr const char* DEVICE_ID  = "stackchan";
static char s_topic_set[48];
static char s_topic_status[48];

/* HA light state */
static bool     s_ha_active   = false;      // light ON/OFF
static bool     s_xiao_active = false;      // xiaozhi owns LED 0
static uint8_t  s_r           = 200;        // default warmish white
static uint8_t  s_g           = 200;
static uint8_t  s_b           = 200;
static uint8_t  s_brightness  = 240;        // 0-255
static std::string s_effect   = "None";
static int      s_effect_step = 0;          // for rainbow hue rotation

/* ===========================================================================
 *  HSV → RGB  (h: 0-359, s/v: 0-255)
 * =========================================================================== */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0) {
        *r = v; *g = v; *b = v;
        return;
    }
    uint8_t region = (uint8_t)(h / 60);
    uint8_t rem    = (uint8_t)((h % 60) * 255 / 60);
    uint16_t v16   = v;
    uint8_t p = (uint8_t)((v16 * (255 - s)) / 255);
    uint8_t q = (uint8_t)((v16 * (255 - ((uint16_t)s * rem / 255))) / 255);
    uint8_t t = (uint8_t)((v16 * (255 - ((uint16_t)s * (255 - rem) / 255))) / 255);

    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default:*r = v; *g = p; *b = q; break;
    }
}

/* ===========================================================================
 *  Apply current HA state to hardware LEDs
 * =========================================================================== */
static void hal_led_apply()
{
    if (!s_ha_active) {
        /* OFF — turn off all LEDs; skip LED 0 only if xiaozhi owns it */
        for (int i = 0; i < 12; i++) {
            if (i == 0 && s_xiao_active) continue;
            GetHAL().setRgbColor(i, 0, 0, 0);
        }
        GetHAL().refreshRgb();
        return;
    }

    /* Scale by brightness */
    uint8_t r = (uint8_t)(((uint16_t)s_r * s_brightness) / 255);
    uint8_t g = (uint8_t)(((uint16_t)s_g * s_brightness) / 255);
    uint8_t b = (uint8_t)(((uint16_t)s_b * s_brightness) / 255);

    for (int i = 0; i < 12; i++) {
        if (i == 0 && s_xiao_active)
            continue;               /* xiaozhi owns LED 0 right now */
        GetHAL().setRgbColor(i, r, g, b);
    }
    GetHAL().refreshRgb();
}

/* ===========================================================================
 *  Publish retained state to MQTT status topic
 * =========================================================================== */
static void hal_led_publish_state()
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "state", s_ha_active ? "ON" : "OFF");
    cJSON_AddNumberToObject(root, "brightness", s_brightness);

    cJSON *color = cJSON_CreateObject();
    cJSON_AddNumberToObject(color, "r", s_r);
    cJSON_AddNumberToObject(color, "g", s_g);
    cJSON_AddNumberToObject(color, "b", s_b);
    cJSON_AddItemToObject(root, "color", color);

    cJSON_AddStringToObject(root, "effect", s_effect.c_str());

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(s_client, s_topic_status, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);
}

/* ===========================================================================
 *  HA MQTT Discovery (retained)
 * =========================================================================== */
static void hal_led_publish_discovery()
{
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "%s_led", DEVICE_ID);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "name",        "Light Bar");
    cJSON_AddStringToObject(root, "unique_id",   unique_id);
    cJSON_AddStringToObject(root, "state_topic", s_topic_status);
    cJSON_AddStringToObject(root, "command_topic", s_topic_set);
    cJSON_AddStringToObject(root, "schema",      "json");
    cJSON_AddBoolToObject(root,   "brightness",  true);
    cJSON_AddBoolToObject(root,   "effect",      true);

    /* effect_list — cJSON doesn't have CreateStringArray, build manually */
    {
        cJSON *el = cJSON_CreateArray();
        cJSON_AddItemToArray(el, cJSON_CreateString("None"));
        cJSON_AddItemToArray(el, cJSON_CreateString("Rainbow"));
        cJSON_AddItemToArray(el, cJSON_CreateString("Random"));
        cJSON_AddItemToArray(el, cJSON_CreateString("Twinkle"));
        cJSON_AddItemToObject(root, "effect_list", el);
    }

    /* supported_color_modes */
    {
        cJSON *cm = cJSON_CreateArray();
        cJSON_AddItemToArray(cm, cJSON_CreateString("rgb"));
        cJSON_AddItemToObject(root, "supported_color_modes", cm);
    }

    /* Device (groups under the same StackChan device in HA) */
    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers",  DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",         "StackChan");
    cJSON_AddStringToObject(dev, "model",        "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char topic[128];
    snprintf(topic, sizeof(topic), "homeassistant/light/%s/config", unique_id);

    char *payload = cJSON_PrintUnformatted(root);
    if (payload) {
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG, "Discovery published: %s", topic);
}

/* ===========================================================================
 *  Command handler
 * =========================================================================== */
static void handle_led_command(const char *data)
{
    if (!data || !data[0]) return;

    /* ---- Plain ON/OFF (non-JSON) ---- */
    if (data[0] != '{') {
        bool was = s_ha_active;
        s_ha_active = (strcmp(data, "ON") == 0);
        if (s_ha_active != was) {
            if (s_ha_active) {
                /* ON with default color if first time */
                s_effect = "None";
            }
            hal_led_apply();
            hal_led_publish_state();
        }
        return;
    }

    /* ---- JSON payload ---- */
    cJSON *json = cJSON_Parse(data);
    if (!json) {
        ESP_LOGW(TAG, "JSON parse error: %s", data);
        return;
    }

    /* state */
    cJSON *state = cJSON_GetObjectItem(json, "state");
    if (cJSON_IsString(state)) {
        s_ha_active = (strcmp(state->valuestring, "ON") == 0);
    }

    /* Early exit for OFF */
    if (!s_ha_active) {
        cJSON_Delete(json);
        hal_led_apply();
        hal_led_publish_state();
        return;
    }

    /* brightness */
    cJSON *bri = cJSON_GetObjectItem(json, "brightness");
    if (cJSON_IsNumber(bri)) {
        int v = bri->valueint;
        if (v < 0)   v = 0;
        if (v > 255) v = 255;
        s_brightness = (uint8_t)v;
    }

    /* color (RGB) */
    cJSON *color = cJSON_GetObjectItem(json, "color");
    if (cJSON_IsObject(color)) {
        cJSON *r = cJSON_GetObjectItem(color, "r");
        cJSON *g = cJSON_GetObjectItem(color, "g");
        cJSON *b = cJSON_GetObjectItem(color, "b");
        if (cJSON_IsNumber(r)) s_r = (uint8_t)r->valueint;
        if (cJSON_IsNumber(g)) s_g = (uint8_t)g->valueint;
        if (cJSON_IsNumber(b)) s_b = (uint8_t)b->valueint;
    }

    /* effect */
    cJSON *eff = cJSON_GetObjectItem(json, "effect");
    if (cJSON_IsString(eff)) {
        s_effect = eff->valuestring;
        s_effect_step = 0;
    }

    cJSON_Delete(json);

    hal_led_apply();
    hal_led_publish_state();
}

/* ===========================================================================
 *  Effects
 * =========================================================================== */

/** Rainbow — rotate hues across all 12 LEDs, 5°/tick, full cycle ~14 s */
static void rainbow_tick()
{
    if (!s_ha_active || s_effect != "Rainbow") return;

    for (int i = 0; i < 12; i++) {
        if (i == 0 && s_xiao_active) continue;

        int hue = (s_effect_step + i * 30) % 360;
        uint8_t r, g, b;
        hsv_to_rgb((uint16_t)hue, 255, s_brightness, &r, &g, &b);
        GetHAL().setRgbColor(i, r, g, b);
    }
    GetHAL().refreshRgb();
    s_effect_step = (s_effect_step + 5) % 360;
}

/* Random — placeholders for future effects */
static void random_tick()   { /* TODO */ }
static void twinkle_tick()  { /* TODO */ }

/* ===========================================================================
 *  Public API
 * =========================================================================== */

void hal_led_init(esp_mqtt_client_handle_t client)
{
    s_client = client;

    snprintf(s_topic_set,    sizeof(s_topic_set),    "%s/led/set",    DEVICE_ID);
    snprintf(s_topic_status, sizeof(s_topic_status), "%s/led/status", DEVICE_ID);

    ESP_LOGI(TAG, "Init — set=%s  status=%s", s_topic_set, s_topic_status);
}

void hal_led_on_connected(esp_mqtt_client_handle_t client)
{
    s_client = client;

    /* Retained discovery so HA auto-discovers on reconnect */
    hal_led_publish_discovery();

    /* Subscribe to command topic (QoS 1) */
    esp_mqtt_client_subscribe(client, s_topic_set, 1);

    /* Publish current state (retained) and sync hardware */
    hal_led_publish_state();
    hal_led_apply();

    ESP_LOGI(TAG, "Connected — subscribed to %s", s_topic_set);
}

void hal_led_tick(esp_mqtt_client_handle_t client)
{
    s_client = client;

    if (s_effect == "Rainbow") {
        rainbow_tick();
    } else if (s_effect == "Random") {
        random_tick();
    } else if (s_effect == "Twinkle") {
        twinkle_tick();
    }
}

bool hal_led_handle_command(esp_mqtt_client_handle_t client,
                            const char *topic, const char *data)
{
    if (!topic || !data) return false;

    s_client = client;

    if (strcmp(topic, s_topic_set) == 0) {
        handle_led_command(data);
        return true;
    }
    return false;
}

void hal_led_set_xiaozhi_active(bool active)
{
    if (s_xiao_active == active) return;

    s_xiao_active = active;

    /* Re-apply HA colours — when active=false, LED 0 is included;
     * when active=true, LED 0 is skipped. */
    hal_led_apply();

    ESP_LOGD(TAG, "Xiaozhi LED-0 ownership: %s", active ? "xiaozhi" : "HA");
}

void hal_led_stop()
{
    /* No dynamically allocated resources in V1 */
    s_client = nullptr;
}
