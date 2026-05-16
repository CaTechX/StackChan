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
#include <hal/hal_auto_start.h>
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

    // Install apps
    GetMooncake().installApp(std::make_unique<AppLauncher>());
    GetMooncake().installApp(std::make_unique<AppAiAgent>());
    GetMooncake().installApp(std::make_unique<AppAvatar>());
    GetMooncake().installApp(std::make_unique<AppEspnowControl>());
    GetMooncake().installApp(std::make_unique<AppAppCenter>());
    GetMooncake().installApp(std::make_unique<AppEzdata>());
    GetMooncake().installApp(std::make_unique<AppDance>());
    GetMooncake().installApp(std::make_unique<AppSetup>());

    // Check auto-start — the NVS flag may be set for next boot
    bool auto_start = hal_auto_start_is_enabled();
    int settle_frames = auto_start ? 5 : 0;

    // Main loop
    while (1) {
        GetHAL().feedTheDog();
        GetHAL().updateHeapStatusLog();

        GetMooncake().update();

        // In auto-start mode: let Mooncake / LVGL run a few frames to settle
        // before triggering xiaozhi.  This mimics the manual flow (user taps →
        // requestXiaozhiStart → loop breaks → cleanup → startXiaozhi) and
        // avoids a crash in lv_event_mark_deleted that occurs when destroying
        // freshly-created LVGL objects after just one frame.
        if (settle_frames > 0) {
            settle_frames--;
            if (settle_frames == 0) {
                GetHAL().requestXiaozhiStart();
                ESP_LOGI("main", "Auto-start xiaozhi triggered after settling");
            }
        }

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

    Settings mqtt_nvs("mqtt", true);
    mqtt_nvs.SetString("broker",   "192.168.1.254");   // 你的 HA 服务器 IP
    mqtt_nvs.SetInt("port",        1883);
    mqtt_nvs.SetBool("enabled",    true);

    // Start xiaozhi, never returns
    GetHAL().startXiaozhi();
}
