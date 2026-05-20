#pragma once

#include <mqtt_client.h>

void hal_servo_mqtt_init(esp_mqtt_client_handle_t client);
void hal_servo_mqtt_on_connected(esp_mqtt_client_handle_t client);
bool hal_servo_mqtt_handle_command(esp_mqtt_client_handle_t client, const char *topic, const char *data);
void hal_servo_mqtt_tick(void);
