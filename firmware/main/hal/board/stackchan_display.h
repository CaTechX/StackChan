/*
 * SPDX-FileCopyrightText: 2026 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include <display/lvgl_display/lvgl_display.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_timer.h>
#include <memory>

class StackChanAvatarDisplay : public LvglDisplay {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_       = nullptr;
    int speaking_modifier_id_           = -1;
    int idle_motion_modifier_id_        = -1;
    int idle_expression_modifier_id_    = -1;
    int blink_modifier_id_              = -1;
    bool is_sleeping_                   = false;
    uint8_t idle_motion_level_          = 2;

    lv_obj_t* preview_image_                         = nullptr;
    esp_timer_handle_t preview_timer_                = nullptr;
    std::unique_ptr<LvglImage> preview_image_cached_ = nullptr;

    void CreateIdleMotionModifier();

    // ── HA expression tracking ──
    bool _ha_active = false;
    char _ha_emotion_state[32] = {0};
    int _ha_decorator_ids[4] = {-1, -1, -1, -1};
    int _ha_decorator_count = 0;
    bool _ha_eyes_hidden = false;
    bool _ha_mouth_open = false;
    int _ha_mouth_rotation_side = -1;  // +1 or -1 for dizzy mouth animation

    // Xiaozhi status (for high-priority avoidance)
    enum class XiaozhiStatus { UNKNOWN, STANDBY, LISTENING, SPEAKING };
    XiaozhiStatus _xiaozhi_status = XiaozhiStatus::UNKNOWN;

    // Helpers (no Modifiable params — helpers call GetStackChan() internally)
    void removeHaDecorators();
    void addHaDecorator(const char* decorator);
    bool hasHighPriorityExpression();

protected:
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

public:
    StackChanAvatarDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width, int height,
                           int offset_x, int offset_y, bool mirror_x, bool mirror_y, bool swap_xy);
    virtual ~StackChanAvatarDisplay();

    void InitializeLcdThemes();

    // Override Display methods to control Robot
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void ClearChatMessages() override;
    virtual void SetPreviewImage(std::unique_ptr<LvglImage> image) override;
    virtual void UpdateStatusBar(bool update_all = false) override;
    virtual void SetupUI() override;
    virtual void SetTheme(Theme* theme) override;
    virtual void SetStatus(const char* status) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;

    // ── HA emotion expression control (thread-safe via DisplayLockGuard) ──
    bool applyEmotionExpression(const char* emotion, const char* decorator);
    void clearEmotionExpression();
    bool isExpressionActive() const;
    const char* getActiveExpressionState() const;
    void applyDizzyPreset();
    void animateDizzyMouth();

    void LvglLock();
    void LvglUnlock();
    lv_disp_t* GetLvglDisplay();
};
