#pragma once

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <memory>
#include <string>
#include "display.h"
#include "expression_emote.h"

namespace emote {

// Flush progress, for a board that wants to notice a display which has stopped flushing.
// GetMicrosecondsSinceLastFlush returns -1 before the first flush, which is not the same
// answer as a large number.
uint32_t GetFlushCount();
int64_t GetMicrosecondsSinceLastFlush();

class EmoteDisplay : public Display {
public:

    EmoteDisplay(esp_lcd_panel_handle_t panel, esp_lcd_panel_io_handle_t panel_io, int width,
                 int height);
    virtual ~EmoteDisplay();

    virtual bool SetEmotion(const char* emotion) override;
    virtual void SetStatus(const char* status) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void SetTheme(Theme* theme) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;
    virtual void UpdateStatusBar(bool update_all = false) override;
    virtual void SetPowerSaveMode(bool on) override;
    // PHOTO_PREVIEW: 拍照后把这一帧铺满屏幕 5 秒，再让回表情。
    // 摄像头（EspVideo::Capture）拍完会把解码好的 RGB565 帧交给这里。
    virtual void SetPreviewImage(std::unique_ptr<LvglImage> image) override;

    bool StopAnimDialog();
    virtual bool InsertAnimDialog(const char* emoji_name, uint32_t duration_ms) override;
    virtual bool MountAssets(const char* partition_label) override;
    virtual void UnmountAssets() override;
    virtual bool GetAssetData(const std::string& name, const uint8_t*& data, size_t& size) override;
    virtual void LoadAssets() override;

    void RefreshAll();

    // Get emote handle for internal use
    emote_handle_t GetEmoteHandle() const { return emote_handle_; }

private:
    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    // PHOTO_PREVIEW: 一张照片要占满屏幕显示几秒，用 gfx 的自定义 image 对象承载，
    // 这样它和表情走同一条渲染/刷屏链路，不会和渲染任务抢 panel。
    struct PreviewState;
    static void OnPreviewTimeout(void* arg);
    void HidePreview();

    emote_handle_t emote_handle_ = nullptr;
    int width_ = 0;
    int height_ = 0;
    PreviewState* preview_ = nullptr;
};

}  // namespace emote
