#pragma once

#include <mqtt_client.h>

/**
 * @brief Initialise LED component — set up topic names.
 * Call from mqtt_task() BEFORE esp_mqtt_client_start().
 */
void hal_led_init(esp_mqtt_client_handle_t client);

/**
 * @brief Called on MQTT_EVENT_CONNECTED — publish HA discovery, subscribe,
 *        and sync initial LED state to hardware.
 */
void hal_led_on_connected(esp_mqtt_client_handle_t client);

/**
 * @brief Periodic tick for effect animations (rainbow, etc.).
 * Call from the MQTT task main loop once per cycle.
 */
void hal_led_tick(esp_mqtt_client_handle_t client);

/**
 * @brief Route incoming MQTT command to the LED component.
 * Returns true if the topic was consumed (no further routing needed).
 */
bool hal_led_handle_command(esp_mqtt_client_handle_t client,
                            const char *topic, const char *data);

/**
 * @brief Inform the LED component whether xiaozhi is actively using LED 0.
 * Called from StackChanAvatarDisplay::SetStatus().
 * When active=true, LED 0 is reserved for xiaozhi's status indicator;
 * when active=false, HA regains control of LED 0.
 */
void hal_led_set_xiaozhi_active(bool active);

/**
 * @brief Shutdown — currently a no-op, provided for future resource cleanup.
 */
void hal_led_stop();
