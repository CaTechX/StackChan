#pragma once

#include <mqtt_client.h>

/**
 * @brief HA-side emotion expression control module.
 *
 * Provides a MQTT Select entity (emotion) and Number entity (duration)
 * for Home Assistant.  The select sends an emotion option like "happy+heart"
 * or "dizzy"; the number sets a cached duration in seconds.
 *
 * Design rules (see docs/mqtt-component-checklist.md):
 *   - init  →  on_connected  →  handle_command
 *   - No retain on command topics.
 *   - State topics use QoS 1 + retain.
 */

void hal_expression_init(esp_mqtt_client_handle_t client, const char* device_id);
void hal_expression_on_connected(esp_mqtt_client_handle_t client);
bool hal_expression_handle_command(esp_mqtt_client_handle_t client, const char* topic, const char* data);
