#pragma once

#include <mqtt_client.h>

/**
 * @brief Init — set up MQTT topic names.  Call from mqtt_task()
 *        BEFORE esp_mqtt_client_start().
 */
void hal_auto_start_init(esp_mqtt_client_handle_t client);

/**
 * @brief MQTT_EVENT_CONNECTED — publish discovery, subscribe, sync state.
 */
void hal_auto_start_on_connected(esp_mqtt_client_handle_t client);

/**
 * @brief Route incoming MQTT commands.  Returns true if consumed.
 */
bool hal_auto_start_handle_command(esp_mqtt_client_handle_t client,
                                   const char *topic, const char *data);

/**
 * @brief Check NVS for the auto-start flag.
 *        Called from app_main() after HAL init.
 */
bool hal_auto_start_is_enabled();

/**
 * @brief Shutdown placeholder.
 */
void hal_auto_start_stop();
