/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <smooth_ui_toolkit.hpp>
#include <uitk/short_namespace.hpp>
#include <mooncake_log.h>
#include <mooncake.h>
#include <apps/apps.h>
#include <hal/hal.h>
#include <settings.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>

using namespace mooncake;
using namespace smooth_ui_toolkit;

extern "C" void app_main(void)
{
    // Setup logger
    mclog::set_level(mclog::level_info);
    mclog::set_time_format(mclog::time_format_unix_milliseconds);

    // HAL init
    GetHAL().init();

    // Setup ui hal
    ui_hal::on_delay([](uint32_t ms) { GetHAL().delay(ms); });
    ui_hal::on_get_tick([]() { return GetHAL().millis(); });

    // Skip the Mooncake launcher when AI Agent auto-start is enabled
    // (official v1.5.1 feature via XiaozhiConfig_t.startAiAgentOnBoot).
    const bool skip_mooncake =
        GetHAL().getXiaozhiConfig().startAiAgentOnBoot && GetHAL().getWarmRebootTarget() < 0;

    if (!skip_mooncake) {
        // Install apps
        GetMooncake().installApp(std::make_unique<AppLauncher>());
        GetMooncake().installApp(std::make_unique<AppAiAgent>());
        GetMooncake().installApp(std::make_unique<AppAvatar>());
        GetMooncake().installApp(std::make_unique<AppEspnowControl>());
        GetMooncake().installApp(std::make_unique<AppAppCenter>());
        GetMooncake().installApp(std::make_unique<AppEzdata>());
        GetMooncake().installApp(std::make_unique<AppDance>());
        GetMooncake().installApp(std::make_unique<AppSetup>());

        // Main loop
        while (1) {
            GetHAL().feedTheDog();
            GetHAL().updateHeapStatusLog();

            GetMooncake().update();

            if (GetHAL().isXiaozhiStartRequested()) {
                break;
            }
        }

        // Acquire the LVGL mutex before destroying LVGL objects. This blocks
        // the LVGL Ticker task (which runs lv_timer_handler()) from accessing
        // the LVGL state concurrently. We will release it after cleanup.
        // Note: lvgl_port_lock(0) means wait forever (0→portMAX_DELAY internally).
        if (lvgl_port_lock(0)) {
            GetMooncake().uninstallAllApps();
            lvgl_port_unlock();
        }
        DestroyMooncake();
    }

    // Persist HA MQTT broker configuration (read by hal_mqtt on startup)
    Settings mqtt_nvs("mqtt", true);
    mqtt_nvs.SetString("broker",   "192.168.1.254");   // 你的 HA 服务器 IP
    mqtt_nvs.SetInt("port",        1883);
    mqtt_nvs.SetBool("enabled",    true);

    // Start xiaozhi, never returns
    GetHAL().startXiaozhi();
}
