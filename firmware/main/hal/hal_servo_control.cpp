/*
 * hal_servo_control.cpp — HA MQTT control for StackChan yaw/pitch servos
 *
 * Protocol (MQTT):
 *   {device_id}/servo/yaw/set        — Set yaw angle (internal units)
 *   {device_id}/servo/yaw/status     — Current yaw angle
 *   {device_id}/servo/pitch/set      — Set pitch angle (internal units)
 *   {device_id}/servo/pitch/status   — Current pitch angle
 *   {device_id}/servo/position/set   — Set both axes at once (JSON: {"yaw":N,"pitch":N})
 *   {device_id}/servo/speed/set      — Set movement speed (0-1000)
 *   {device_id}/servo/speed/status   — Current speed
 *   {device_id}/servo/home/set       — Trigger go-home (button)
 *
 * HA Discovery: 2x number (angle) + 1x number (speed) + 1x button (home).
 *
 * Conflict with xiaozhi motion:
 *   Commands go through GetStackChan().motion().moveWithSpeed() which sets
 *   the servo animation target. Xiaozhi's avatar calls the same API via
 *   motion modifiers. "Last writer wins" — if xiaozhi is actively animating
 *   (e.g. talking) it will override the MQTT command; when idle, the MQTT
 *   position holds.
 *
 * Protection:
 *   Servo angle limits (yaw: -1280..1280, pitch: 30..870) are enforced by
 *   Servo::update_angle_anim_target() so no range checking needed here.
 *   Raw-position clamping is in ScsServo::set_angle_impl().
 *
 * Speed:
 *   0..1000, mapped internally to spring stiffness/damping.
 *   Default 300 = moderate speed.
 *
 * Sync logic (2026-05-17, v2):
 *   PROBLEM: On command, publish_yaw() read getCurrentYawAngle() immediately
 *   — the servo hadn't started moving yet, so HA stored the OLD position
 *   (or a random intermediate value during spring animation). Later, a
 *   progressive-update tick() caused oscillation between intermediate values
 *   and extreme limits (-1280), creating an infinite publish loop.
 *
 *   FIX:
 *     1. On command → publish the **commanded target** value immediately,
 *        so HA shows what the user requested right away.
 *     2. Use stored last-target values for the opposite axis (s_last_yaw_target /
 *        s_last_pitch_target) instead of reading hardware via getCurrentAngle() —
 *        this avoids SCS bus contention and preserves the other axis's target
 *        when commands arrive in rapid succession.
 *     3. Set a "settle pending" flag.
 *     4. In tick() → do NOT publish intermediate values. Instead, check if
 *        the servo has _stopped_ moving (isMoving() == false). Once settled,
 *        publish the stored target value once, then clear the flag.
 *     5. This eliminates the oscillation entirely: only two publishes per
 *        command (target immediately + confirm on settle).
 */

#include "hal_servo_control.h"
#include <cJSON.h>
#include <esp_log.h>
#include <cstring>
#include <cstdio>
#include <stackchan/stackchan.h>

using namespace stackchan::motion;

static const char *TAG = "HalServoCtrl";

/* -------------------------------------------------------------------------
 * Internal state (file-scope)
 * ------------------------------------------------------------------------- */

static esp_mqtt_client_handle_t s_client    = nullptr;
static constexpr const char*    DEVICE_ID   = "stackchan";
static char s_topic_yaw_status[64]     = {0};
static char s_topic_yaw_set[64]        = {0};
static char s_topic_pitch_status[64]   = {0};
static char s_topic_pitch_set[64]      = {0};
static char s_topic_speed_status[64]   = {0};
static char s_topic_speed_set[64]      = {0};
static char s_topic_home_set[64]       = {0};
static char s_topic_position_set[64]   = {0};

/* Movement tracking — set true when a movement command is issued, cleared
 * by tick() once the spring animation has settled. */
static bool s_movement_pending = false;
static int  s_last_yaw_target   = 0;
static int  s_last_pitch_target = 0;

static int s_speed = 300;  // default moderate speed

/* -------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

static Motion *get_motion()
{
    auto &sc = GetStackChan();
    return &sc.motion();
}

static void publish_yaw_value(int value)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    esp_mqtt_client_publish(s_client, s_topic_yaw_status, buf, 0, 1, 1);
}

static void publish_pitch_value(int value)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", value);
    esp_mqtt_client_publish(s_client, s_topic_pitch_status, buf, 0, 1, 1);
}

/* Read actual current position from hardware animation state and publish it.
 * Used for initial state report (on_connected) and settle-finalize. */
static void publish_yaw()
{
    Motion *m = get_motion();
    if (!m) return;
    publish_yaw_value(m->getCurrentYawAngle());
}

static void publish_pitch()
{
    Motion *m = get_motion();
    if (!m) return;
    publish_pitch_value(m->getCurrentPitchAngle());
}

static void publish_speed()
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", s_speed);
    esp_mqtt_client_publish(s_client, s_topic_speed_status, buf, 0, 1, 1);
}

static void publish_all_state()
{
    publish_yaw();
    publish_pitch();
    publish_speed();
}

/* -------------------------------------------------------------------------
 * Command handlers
 * ------------------------------------------------------------------------- */

static void handle_yaw_command(const char *data)
{
    if (!data || !data[0]) return;
    int angle = atoi(data);
    Motion *m = get_motion();
    if (!m) return;
    /* Use stored last pitch target instead of reading hardware —
     * this preserves the pitch value set by a previous command. */
    m->moveWithSpeed(angle, s_last_pitch_target, s_speed);
    ESP_LOGI(TAG, "Yaw → %d (speed %d)", angle, s_speed);
    s_last_yaw_target = angle;
    publish_yaw_value(angle);
    s_movement_pending = true;
}

static void handle_pitch_command(const char *data)
{
    if (!data || !data[0]) return;
    int angle = atoi(data);
    Motion *m = get_motion();
    if (!m) return;
    /* Use stored last yaw target instead of reading hardware —
     * this preserves the yaw value set by a previous command. */
    m->moveWithSpeed(s_last_yaw_target, angle, s_speed);
    ESP_LOGI(TAG, "Pitch → %d (speed %d)", angle, s_speed);
    s_last_pitch_target = angle;
    publish_pitch_value(angle);
    s_movement_pending = true;
}

/* Combined position command — accepts JSON {"yaw": N, "pitch": N}
 * so both axes move simultaneously in a single call. */
static void handle_position_command(const char *data)
{
    if (!data || !data[0]) return;
    cJSON *json = cJSON_Parse(data);
    if (!json) {
        ESP_LOGW(TAG, "Invalid position JSON: %s", data);
        return;
    }
    int yaw   = s_last_yaw_target;
    int pitch = s_last_pitch_target;
    cJSON *yaw_item   = cJSON_GetObjectItem(json, "yaw");
    cJSON *pitch_item = cJSON_GetObjectItem(json, "pitch");
    if (cJSON_IsNumber(yaw_item))   yaw   = yaw_item->valueint;
    if (cJSON_IsNumber(pitch_item)) pitch = pitch_item->valueint;
    cJSON_Delete(json);

    Motion *m = get_motion();
    if (!m) return;
    m->moveWithSpeed(yaw, pitch, s_speed);
    ESP_LOGI(TAG, "Position → yaw=%d pitch=%d (speed %d)", yaw, pitch, s_speed);
    s_last_yaw_target   = yaw;
    s_last_pitch_target = pitch;
    publish_yaw_value(yaw);
    publish_pitch_value(pitch);
    s_movement_pending = true;
}

static void handle_speed_command(const char *data)
{
    if (!data || !data[0]) return;
    int val = atoi(data);
    val = (val < 0) ? 0 : (val > 1000) ? 1000 : val;
    s_speed = val;
    ESP_LOGI(TAG, "Speed → %d", s_speed);
    publish_speed();
}

static void go_home()
{
    static constexpr int PITCH_ANGLE_MIN = 30;  // mirrors pitch_servo_config.angleLimit min in hal_servo.cpp
    Motion *m = get_motion();
    if (!m) return;
    m->goHome(s_speed);
    ESP_LOGI(TAG, "Home: yaw=0 pitch=0 speed=%d", s_speed);
    /* goHome sets both axes to 0, but pitch limit [30,870] clamps 0→30.
     * Publish the clamped value instead of 0, because HA discovery
     * configures pitch min=30 and would silently drop a 0. */
    s_last_yaw_target   = 0;
    s_last_pitch_target = PITCH_ANGLE_MIN;
    publish_yaw_value(s_last_yaw_target);
    publish_pitch_value(s_last_pitch_target);
    publish_speed();
    s_movement_pending = true;
}

static void handle_home_command(const char *data)
{
    if (!data) return;
    go_home();
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void hal_servo_mqtt_init(esp_mqtt_client_handle_t client)
{
    s_client = client;

    /* Build topic strings */
    snprintf(s_topic_yaw_status,   sizeof(s_topic_yaw_status),   "%s/servo/yaw/status",   DEVICE_ID);
    snprintf(s_topic_yaw_set,     sizeof(s_topic_yaw_set),       "%s/servo/yaw/set",      DEVICE_ID);
    snprintf(s_topic_pitch_status, sizeof(s_topic_pitch_status), "%s/servo/pitch/status", DEVICE_ID);
    snprintf(s_topic_pitch_set,   sizeof(s_topic_pitch_set),     "%s/servo/pitch/set",    DEVICE_ID);
    snprintf(s_topic_speed_status, sizeof(s_topic_speed_status), "%s/servo/speed/status", DEVICE_ID);
    snprintf(s_topic_speed_set,   sizeof(s_topic_speed_set),     "%s/servo/speed/set",    DEVICE_ID);
    snprintf(s_topic_home_set,    sizeof(s_topic_home_set),      "%s/servo/home/set",     DEVICE_ID);
    snprintf(s_topic_position_set, sizeof(s_topic_position_set), "%s/servo/position/set", DEVICE_ID);
}

/* -------------------------------------------------------------------------
 * HA Discovery (published on MQTT connect with retain)
 * ------------------------------------------------------------------------- */

static void publish_servo_discovery(esp_mqtt_client_handle_t client)
{
    char topic[128], unique_id[128];

    /* ---- Yaw angle (number) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/number/%s_servo_yaw/config", DEVICE_ID);
    snprintf(unique_id, sizeof(unique_id), "%s_servo_yaw", DEVICE_ID);

    cJSON *yaw = cJSON_CreateObject();
    cJSON_AddStringToObject(yaw, "name",              "Servo Yaw Angle");
    cJSON_AddStringToObject(yaw, "unique_id",         unique_id);
    cJSON_AddStringToObject(yaw, "command_topic",     s_topic_yaw_set);
    cJSON_AddStringToObject(yaw, "state_topic",       s_topic_yaw_status);
    cJSON_AddStringToObject(yaw, "mode",              "box");
    cJSON_AddNumberToObject(yaw, "min",               -1280);
    cJSON_AddNumberToObject(yaw, "max",               1280);
    cJSON_AddNumberToObject(yaw, "step",              1);

    cJSON *dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddStringToObject(dev, "model",       "StackChan");
    cJSON_AddStringToObject(dev, "manufacturer", "M5Stack");
    cJSON_AddItemToObject(yaw, "device", dev);

    char *payload = cJSON_PrintUnformatted(yaw);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(yaw);

    /* ---- Pitch angle (number) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/number/%s_servo_pitch/config", DEVICE_ID);
    snprintf(unique_id, sizeof(unique_id), "%s_servo_pitch", DEVICE_ID);

    cJSON *pitch = cJSON_CreateObject();
    cJSON_AddStringToObject(pitch, "name",              "Servo Pitch Angle");
    cJSON_AddStringToObject(pitch, "unique_id",         unique_id);
    cJSON_AddStringToObject(pitch, "command_topic",     s_topic_pitch_set);
    cJSON_AddStringToObject(pitch, "state_topic",       s_topic_pitch_status);
    cJSON_AddStringToObject(pitch, "mode",              "box");
    cJSON_AddNumberToObject(pitch, "min",               30);
    cJSON_AddNumberToObject(pitch, "max",               870);
    cJSON_AddNumberToObject(pitch, "step",              1);

    dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddItemToObject(pitch, "device", dev);

    payload = cJSON_PrintUnformatted(pitch);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(pitch);

    /* ---- Speed (number) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/number/%s_servo_speed/config", DEVICE_ID);
    snprintf(unique_id, sizeof(unique_id), "%s_servo_speed", DEVICE_ID);

    cJSON *spd = cJSON_CreateObject();
    cJSON_AddStringToObject(spd, "name",              "Servo Speed");
    cJSON_AddStringToObject(spd, "unique_id",         unique_id);
    cJSON_AddStringToObject(spd, "command_topic",     s_topic_speed_set);
    cJSON_AddStringToObject(spd, "state_topic",       s_topic_speed_status);
    cJSON_AddStringToObject(spd, "mode",              "box");
    cJSON_AddNumberToObject(spd, "min",               0);
    cJSON_AddNumberToObject(spd, "max",               1000);
    cJSON_AddNumberToObject(spd, "step",              1);
    cJSON_AddNumberToObject(spd, "initial",           300);

    dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddItemToObject(spd, "device", dev);

    payload = cJSON_PrintUnformatted(spd);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(spd);

    /* ---- Go Home (button) ---- */
    snprintf(topic, sizeof(topic),
             "homeassistant/button/%s_servo_home/config", DEVICE_ID);
    snprintf(unique_id, sizeof(unique_id), "%s_servo_home", DEVICE_ID);

    cJSON *home = cJSON_CreateObject();
    cJSON_AddStringToObject(home, "name",              "Go Home");
    cJSON_AddStringToObject(home, "unique_id",         unique_id);
    cJSON_AddStringToObject(home, "command_topic",     s_topic_home_set);
    cJSON_AddStringToObject(home, "payload_press",     "PRESS");
    cJSON_AddStringToObject(home, "mode",              "restart");

    dev = cJSON_CreateObject();
    cJSON_AddStringToObject(dev, "identifiers", DEVICE_ID);
    cJSON_AddStringToObject(dev, "name",        "StackChan");
    cJSON_AddItemToObject(home, "device", dev);

    payload = cJSON_PrintUnformatted(home);
    if (payload) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
        free(payload);
    }
    cJSON_Delete(home);

    ESP_LOGI(TAG, "Servo HA Discovery published for %s", DEVICE_ID);
}

void hal_servo_mqtt_on_connected(esp_mqtt_client_handle_t client)
{
    publish_servo_discovery(client);
    publish_all_state();

    /* Subscribe to command topics */
    esp_mqtt_client_subscribe(client, s_topic_yaw_set,   1);
    esp_mqtt_client_subscribe(client, s_topic_pitch_set, 1);
    esp_mqtt_client_subscribe(client, s_topic_speed_set, 1);
    esp_mqtt_client_subscribe(client, s_topic_home_set,      1);
    esp_mqtt_client_subscribe(client, s_topic_position_set,  1);
    ESP_LOGI(TAG, "Connected — subscribed to servo command topics");
}

bool hal_servo_mqtt_handle_command(esp_mqtt_client_handle_t client, const char *topic, const char *data)
{
    (void)client;
    if (!topic || !data || !data[0]) return false;

    if (strcmp(topic, s_topic_yaw_set) == 0) {
        handle_yaw_command(data);
        return true;
    }
    if (strcmp(topic, s_topic_pitch_set) == 0) {
        handle_pitch_command(data);
        return true;
    }
    if (strcmp(topic, s_topic_speed_set) == 0) {
        handle_speed_command(data);
        return true;
    }
    if (strcmp(topic, s_topic_home_set) == 0) {
        handle_home_command(data);
        return true;
    }
    if (strcmp(topic, s_topic_position_set) == 0) {
        handle_position_command(data);
        return true;
    }
    return false;
}

void hal_servo_mqtt_tick(void)
{
    /* No pending movement → nothing to do.
     * This is the common case: most ticks there's no command. */
    if (!s_movement_pending)
        return;

    Motion *m = get_motion();
    if (!m) return;

    /* Still moving — wait. Do NOT publish intermediate values.
     * (The target was already published by the command handler.) */
    if (m->isMoving())
        return;

    /* Movement complete (animation settled).
     * Publish final target values (not hardware-read, which may drift
     * due to servo mechanical tolerances). */
    publish_yaw_value(s_last_yaw_target);
    publish_pitch_value(s_last_pitch_target);
    s_movement_pending = false;
    ESP_LOGD(TAG, "Settled: yaw=%d pitch=%d",
             m->getCurrentYawAngle(), m->getCurrentPitchAngle());
}
