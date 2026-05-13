#pragma once

// Public API for MQTT bridge module.
// Start/stop is managed by Hal::startMqtt() — call hal_mqtt_stop() only on shutdown.

void hal_mqtt_stop();
