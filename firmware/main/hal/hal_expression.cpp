#include "hal_expression.h"
#include "board/stackchan_display.h"  // StackChanAvatarDisplay + DisplayLockGuard

#include "board.h"   // Board::GetInstance()

#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

static const char* TAG = "HalExpression";

// ── File-scope state ───────────────────────────────────────────────────────

static esp_mqtt_client_handle_t s_client      = nullptr;
static constexpr const char*    DEVICE_ID     = "stackchan";

// Topic buffers
static char s_topic_select_set[64]    = {0};
static char s_topic_select_state[64]  = {0};
static char s_topic_number_set[64]    = {0};
static char s_topic_number_state[64]  = {0};

// Cached duration (seconds), default 8 s
static uint32_t       s_duration_sec     = 8;
static esp_timer_handle_t s_expire_timer = nullptr;

// Cached last-option published so we can publish "idle" on expiry
static char s_last_option[16] = {0};

// Periodic timer for dizzy mouth animation (600ms toggle)
static esp_timer_handle_t s_mouth_toggle_timer = nullptr;

// ── Helpers ────────────────────────────────────────────────────────────────

static void publish_state(const char* value)
{
    if (!s_client) return;
    esp_mqtt_client_publish(s_client, s_topic_select_state, value, 0, 1, 1);
}

static void publish_duration_state()
{
    if (!s_client) return;
    char buf[8];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s_duration_sec);
    esp_mqtt_client_publish(s_client, s_topic_number_state, buf, 0, 1, 1);
}

/**
 * @brief Parse an expression option  "happy+heart" → emotion="happy", decorator="heart"
 */
static void parse_option(const char* option, char* emotion, size_t emotion_sz,
                                                   char* decorator, size_t decorator_sz)
{
    const char* plus = strchr(option, '+');
    if (plus) {
        size_t len = (size_t)(plus - option);
        if (len >= emotion_sz) len = emotion_sz - 1;
        memcpy(emotion, option, len);
        emotion[len] = '\0';
        strncpy(decorator, plus + 1, decorator_sz - 1);
        decorator[decorator_sz - 1] = '\0';
    } else {
        strncpy(emotion, option, emotion_sz - 1);
        emotion[emotion_sz - 1] = '\0';
        decorator[0] = '\0';
    }
}

// ── Timer callback (expiry of HA emotion) ──────────────────────────────────

static void on_emotion_expiry(void* /*arg*/)
{
    // Stop mouth toggle timer first
    if (s_mouth_toggle_timer) {
        esp_timer_stop(s_mouth_toggle_timer);
    }

    auto display = Board::GetInstance().GetDisplay();
    if (!display) return;

    auto* sc = dynamic_cast<StackChanAvatarDisplay*>(display);
    if (!sc) {
        ESP_LOGW(TAG, "Display not available for expiry");
        return;
    }
    sc->clearEmotionExpression();

    publish_state("idle");
    s_last_option[0] = '\0';

    ESP_LOGI(TAG, "HA emotion expired, restored to idle");
}

// ── Dizzy mouth toggle callback ────────────────────────────────────────────

static void on_mouth_toggle(void* /*arg*/)
{
    auto display = Board::GetInstance().GetDisplay();
    if (!display) return;

    auto* sc = dynamic_cast<StackChanAvatarDisplay*>(display);
    if (!sc) return;

    sc->animateDizzyMouth();
}

static void start_expire_timer(uint32_t duration_ms)
{
    if (s_expire_timer) {
        esp_timer_stop(s_expire_timer);
        if (duration_ms > 0) {
            esp_timer_start_once(s_expire_timer, duration_ms * 1000ULL);
        }
    }
}

// ── Apply an emotion option to the display ────────────────────────────────

static bool apply_emotion_option(const char* option)
{
    auto display = Board::GetInstance().GetDisplay();
    if (!display) return false;

    auto* sc = dynamic_cast<StackChanAvatarDisplay*>(display);
    if (!sc) {
        ESP_LOGE(TAG, "Display is not StackChanAvatarDisplay");
        return false;
    }

    // Stop any ongoing mouth toggle animation (dizzy)
    if (s_mouth_toggle_timer) {
        esp_timer_stop(s_mouth_toggle_timer);
    }

    // ── Special preset: dizzy ──────────────────────────────────────────
    if (strcmp(option, "dizzy") == 0) {
        // "dizzy" has a fixed 4-second duration (matching ImuEventModifier)
        sc->applyDizzyPreset();
        strncpy(s_last_option, option, sizeof(s_last_option) - 1);
        s_last_option[sizeof(s_last_option) - 1] = '\0';
        publish_state(option);
        start_expire_timer(4000);

        // Start mouth rotation animation (600ms toggle, matching ImuEventModifier)
        if (s_mouth_toggle_timer) {
            esp_timer_start_periodic(s_mouth_toggle_timer, 600 * 1000ULL);
        }

        ESP_LOGI(TAG, "Applied dizzy preset (4s)");
        return true;
    }

    // ── Parse emotion [+ decorator] ───────────────────────────────────
    char emotion[32], decorator[32];
    parse_option(option, emotion, sizeof(emotion), decorator, sizeof(decorator));

    bool ok = sc->applyEmotionExpression(emotion, decorator);
    if (!ok) {
        ESP_LOGW(TAG, "Failed to apply emotion '%s' (may be blocked by high-priority expression)", option);
        return false;
    }

    strncpy(s_last_option, option, sizeof(s_last_option) - 1);
    s_last_option[sizeof(s_last_option) - 1] = '\0';
    publish_state(option);

    // Start expiry timer (if duration > 0)
    if (s_duration_sec > 0) {
        start_expire_timer(s_duration_sec * 1000);
    } else {
        // Duration = 0 means "keep until changed" — no timer
        // (handled by explicit neutral command later)
    }

    ESP_LOGI(TAG, "Applied emotion '%s' (duration=%lus)", option, (unsigned long)s_duration_sec);
    return true;
}

// ── Discovery JSON ─────────────────────────────────────────────────────────

static void publish_discovery_select()
{
    cJSON* root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "name",         "Expression");
    char unique_id[128];
    snprintf(unique_id, sizeof(unique_id), "%s_expression", DEVICE_ID);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    cJSON_AddStringToObject(root, "command_topic", s_topic_select_set);
    cJSON_AddStringToObject(root, "state_topic",  s_topic_select_state);

    // Options list
    cJSON* options = cJSON_AddArrayToObject(root, "options");
    const char* opts[] = {
        "idle", "happy",
        "sad", "angry", "doubtful", "sleepy", "dizzy"
    };
    for (auto opt : opts) {
        cJSON_AddItemToArray(options, cJSON_CreateString(opt));
    }

    // Device info (same as other HA entities)
    cJSON* dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers",  DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",         "StackChan");
    cJSON_AddStringToObject(dev, "model",        "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char* payload = cJSON_PrintUnformatted(root);
    if (payload) {
        char topic[256];
        snprintf(topic, sizeof(topic),
            "homeassistant/select/%s_expression/config", DEVICE_ID);
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
        ESP_LOGI(TAG, "Discovery published: %s", topic);
        free(payload);
    }
    cJSON_Delete(root);
}

static void publish_discovery_number()
{
    cJSON* root = cJSON_CreateObject();

    char unique_id[128];
    snprintf(unique_id, sizeof(unique_id), "%s_expression_duration", DEVICE_ID);
    cJSON_AddStringToObject(root, "name",         "Expression Duration");
    cJSON_AddStringToObject(root, "unique_id",    unique_id);
    cJSON_AddStringToObject(root, "command_topic", s_topic_number_set);
    cJSON_AddStringToObject(root, "state_topic",  s_topic_number_state);
    cJSON_AddStringToObject(root, "mode",         "slider");
    cJSON_AddNumberToObject(root, "min",          2);
    cJSON_AddNumberToObject(root, "max",          10);
    cJSON_AddNumberToObject(root, "step",         1);
    cJSON_AddStringToObject(root, "unit_of_measurement", "s");

    // Device info (same as select entity)
    cJSON* dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers",  DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",         "StackChan");
    cJSON_AddStringToObject(dev, "model",        "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(root, "device", dev);

    char* payload = cJSON_PrintUnformatted(root);
    if (payload) {
        char topic[256];
        snprintf(topic, sizeof(topic),
            "homeassistant/number/%s_expression_duration/config", DEVICE_ID);
        esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
        ESP_LOGI(TAG, "Discovery published: %s", topic);
        free(payload);
    }
    cJSON_Delete(root);
}

// ── Command handlers ───────────────────────────────────────────────────────

static void handle_expression_set(esp_mqtt_client_handle_t client, const char* data)
{
    if (!data) return;

    // Strip quotes if HA sends JSON-string payload
    const char* val = data;
    char buf[64];
    if (data[0] == '"') {
        size_t len = strlen(data);
        if (len > 2 && data[len - 1] == '"') {
            len = len - 2;
            if (len >= sizeof(buf)) len = sizeof(buf) - 1;
            memcpy(buf, data + 1, len);
            buf[len] = '\0';
            val = buf;
        }
    }

    // Silently reject "idle" command (read-only marker)
    if (strcmp(val, "idle") == 0) {
        ESP_LOGD(TAG, "Ignoring idle command (read-only)");
        return;
    }

    apply_emotion_option(val);
}

static void handle_duration_set(esp_mqtt_client_handle_t client, const char* data)
{
    if (!data) return;

    // Parse numeric duration
    const char* val = data;
    char buf[32];
    if (data[0] == '"') {
        size_t len = strlen(data);
        if (len > 2 && data[len - 1] == '"') {
            len = len - 2;
            if (len >= sizeof(buf)) len = sizeof(buf) - 1;
            memcpy(buf, data + 1, len);
            buf[len] = '\0';
            val = buf;
        }
    }

    unsigned long sec = strtoul(val, nullptr, 10);
    if (sec < 2)  sec = 2;
    if (sec > 10) sec = 10;

    s_duration_sec = (uint32_t)sec;
    publish_duration_state();

    ESP_LOGI(TAG, "Duration set to %lu s", sec);
}

// ── Public API ─────────────────────────────────────────────────────────────

void hal_expression_init(esp_mqtt_client_handle_t client)
{
    s_client = client;

    // Build topic strings
    snprintf(s_topic_select_set,   sizeof(s_topic_select_set),
             "%s/expression/set",       DEVICE_ID);
    snprintf(s_topic_select_state, sizeof(s_topic_select_state),
             "%s/expression/state",     DEVICE_ID);
    snprintf(s_topic_number_set,   sizeof(s_topic_number_set),
             "%s/expression/duration/set",  DEVICE_ID);
    snprintf(s_topic_number_state, sizeof(s_topic_number_state),
             "%s/expression/duration/state", DEVICE_ID);

    // Create one-shot timer for emotion expiry
    esp_timer_create_args_t timer_args = {};
    timer_args.callback       = on_emotion_expiry;
    timer_args.arg            = nullptr;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name           = "ha_emotion_expire";
    timer_args.skip_unhandled_events = false;

    esp_err_t err = esp_timer_create(&timer_args, &s_expire_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create expire timer: %s", esp_err_to_name(err));
    }

    // Create periodic timer for dizzy mouth rotation animation (600ms)
    esp_timer_create_args_t mouth_timer_args = {};
    mouth_timer_args.callback       = on_mouth_toggle;
    mouth_timer_args.arg            = nullptr;
    mouth_timer_args.dispatch_method = ESP_TIMER_TASK;
    mouth_timer_args.name           = "ha_mouth_toggle";
    mouth_timer_args.skip_unhandled_events = false;

    err = esp_timer_create(&mouth_timer_args, &s_mouth_toggle_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create mouth toggle timer: %s", esp_err_to_name(err));
    }
}

void hal_expression_on_connected(esp_mqtt_client_handle_t client)
{
    s_client = client;

    publish_discovery_select();
    publish_discovery_number();

    // Subscribe to command topics (QoS 1)
    esp_mqtt_client_subscribe(client, s_topic_select_set, 1);
    esp_mqtt_client_subscribe(client, s_topic_number_set, 1);

    // Publish initial states
    publish_state("idle");
    publish_duration_state();

    ESP_LOGI(TAG, "Connected — subscribed to %s, %s",
             s_topic_select_set, s_topic_number_set);
}

bool hal_expression_handle_command(esp_mqtt_client_handle_t client,
                                   const char* topic, const char* data)
{
    if (strcmp(topic, s_topic_select_set) == 0) {
        handle_expression_set(client, data);
        return true;
    }
    if (strcmp(topic, s_topic_number_set) == 0) {
        handle_duration_set(client, data);
        return true;
    }
    return false;
}
