#include "emote_display.h"

// Standard C++ headers
#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstring>
#include <memory>
#include <tuple>
#include <unordered_map>

// Standard C headers
#include <sys/time.h>
#include <time.h>

// ESP-IDF headers
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <lvgl.h>

// FreeRTOS headers
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project headers
#include "assets.h"
#include "assets/lang_config.h"
#include "board.h"
#include "expression_emote.h"
#include "gfx.h"

namespace emote {

// ============================================================================
// Constants and Type Definitions
// ============================================================================

static const char* TAG = "EmoteDisplay";

// ============================================================================
// Forward Declarations
// ============================================================================

class EmoteDisplay;

// ============================================================================
// Helper Functions
// ============================================================================

static bool OnFlushIoReady(const esp_lcd_panel_io_handle_t panel_io,
                           esp_lcd_panel_io_event_data_t* const edata, void* user_ctx) {
    emote_handle_t handle = static_cast<emote_handle_t>(user_ctx);
    if (handle) {
        emote_notify_flush_finished(handle);
    }
    return true;
}

// Flush bookkeeping, so that a display which has stopped flushing can be noticed.
//
// Nothing in the tree watches the render task: no task is subscribed to the task watchdog and
// CONFIG_ESP_TASK_WDT_PANIC is off, and a task blocked on an event group yields the CPU, so the
// idle-task watchdog sees nothing wrong. These two values are the minimum a board needs to tell
// a live display from a wedged one.
static std::atomic<uint32_t> s_flush_count{0};
static std::atomic<int64_t> s_last_flush_us{0};

// Flush callback for emote
static void OnFlushCallback(int x_start, int y_start, int x_end, int y_end, const void* data,
                            emote_handle_t handle) {
    esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)emote_get_user_data(handle);

    // Every exit from here must end in a flush-finished report, one way or the other.
    //
    // The renderer clears the flush-done bit, calls this, then waits on that bit with
    // portMAX_DELAY (esp_emote_gfx, gfx_render.c) - and the only thing that sets it is
    // emote_notify_flush_finished, which is reached only from the panel's on_color_trans_done
    // callback. So a block that never reaches the bus is not a dropped frame: the render task
    // waits for it forever, holding the graphics lock, and every caller of gfx_emote_lock
    // queues up behind it. The device keeps answering pings and WebSocket pongs the whole
    // time, because those are other tasks - so it looks alive and the screen is simply frozen
    // on one frame. Reporting completion here is safe: the bit is set before the renderer
    // reaches its wait, and a pre-set bit makes that wait return at once.
    if (panel == nullptr) {
        emote_notify_flush_finished(handle);
        return;
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "draw_bitmap failed (%s) for (%d,%d)-(%d,%d); reporting flush done so the "
                      "render task is not left waiting for a transfer that never started",
                 esp_err_to_name(err), x_start, y_start, x_end, y_end);
        emote_notify_flush_finished(handle);
        return;
    }

    s_flush_count.fetch_add(1, std::memory_order_relaxed);
    s_last_flush_us.store(esp_timer_get_time(), std::memory_order_relaxed);
}

uint32_t GetFlushCount() {
    return s_flush_count.load(std::memory_order_relaxed);
}

int64_t GetMicrosecondsSinceLastFlush() {
    int64_t last = s_last_flush_us.load(std::memory_order_relaxed);
    if (last == 0) {
        return -1;  // nothing has been flushed yet; not the same as "a long time ago"
    }
    return esp_timer_get_time() - last;
}

// ============================================================================
// Graphics Initialization Functions
// ============================================================================

static emote_handle_t InitializeEmote(const esp_lcd_panel_handle_t panel, const int width,
                                      const int height) {
    if (!panel) {
        ESP_LOGE(TAG, "Invalid panel");
        return nullptr;
    }

    emote_config_t emote_cfg = {
        .flags =
            {
                .swap = true,
                .double_buffer = true,
                .buff_dma = false,
            },
        .gfx_emote =
            {
                .h_res = width,
                .v_res = height,
                .fps = 30,
            },
        .buffers =
            {
                .buf_pixels = static_cast<size_t>(width * 16),
            },
        .task =
            {
                .task_priority = 5,
                .task_stack = 6 * 1024,
                .task_affinity = 0,
                .task_stack_in_ext = false,
            },
        .flush_cb = OnFlushCallback,
        .user_data = (void*)panel,
    };

    emote_handle_t emote_handle = emote_init(&emote_cfg);
    if (!emote_handle) {
        ESP_LOGE(TAG, "Failed to initialize emote");
        return nullptr;
    }

    return emote_handle;
}

// ============================================================================
// EmoteDisplay Class Implementation
// ============================================================================

// 一张照片要占满屏幕显示几秒，用 gfx 的自定义 image 对象承载：
// 它和表情走同一条渲染/刷屏链路，不会和渲染任务抢 panel。
//
// 双缓冲：局域网图传是连续出帧（约 1~2fps），每帧都要换一张图。如果只留一块缓冲，
// 写新帧时渲染任务可能正在读旧帧，只能先 visible=false 停一拍——那一拍就会露出
// 底下的表情，于是屏幕在「摄像头画面 / 表情」之间来回跳。两块缓冲交替用，
// 写后块、显前块，既不用收起对象也没有撕裂。
struct EmoteDisplay::PreviewState {
    gfx_obj_t* obj = nullptr;
    gfx_image_dsc_t dsc[2] = {};
    uint16_t* buf[2] = {nullptr, nullptr};  // 铺满屏幕后的 RGB565 像素，gfx 直接从这里取
    int front = 0;                          // 当前正在显示的那一块
    esp_timer_handle_t timer = nullptr;
};

// ============================================================================
// BATTERY_BADGE: 右上角电量角标
// ============================================================================
//
// 和 PHOTO_PREVIEW 同一套路：表情每帧整屏重绘，直接往 panel 上画会被下一帧抹掉，
// 所以做成 gfx 的 image 子对象挂在显示树上，让渲染任务每帧顺手把它画上去。
//
// 区别在格式：这里用 RGB565A8（独立 alpha 平面），角标大部分像素是透明的，
// 背景透出去，不会在脸上糊一块方块。注意混色路径（gfx_blend_color_mix）会把源
// 当作**普通小端 RGB565** 处理、自己按 ctx->swap 翻字节，所以缓冲里存未翻转的值；
// 而不带掩码的 PHOTO_PREVIEW 走裸 memcpy，必须预先翻转——两者正好相反。
struct EmoteDisplay::BatteryBadge {
    gfx_obj_t* obj = nullptr;
    uint16_t* rgb = nullptr;    // 与 alpha 同一块内存，alpha 紧跟其后
    uint8_t* alpha = nullptr;
    gfx_image_dsc_t dsc = {};
};

namespace {

constexpr int kBadgeW = 44;
constexpr int kBadgeH = 20;
constexpr int kBadgeMargin = 5;

uint16_t PackRgb565(uint8_t r, uint8_t g, uint8_t b) {
    return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// 圆角矩形命中测试：四角按圆心距离算，中间直接放行
bool InRoundedRect(int x, int y, int x0, int y0, int x1, int y1, int radius) {
    if (x < x0 || x > x1 || y < y0 || y > y1) {
        return false;
    }
    const int cx = x < x0 + radius ? x0 + radius : (x > x1 - radius ? x1 - radius : x);
    const int cy = y < y0 + radius ? y0 + radius : (y > y1 - radius ? y1 - radius : y);
    const int dx = x - cx;
    const int dy = y - cy;
    return dx * dx + dy * dy <= radius * radius;
}

// 充电时叠在电量上的小闪电，7×8，bit6 是最左边一列
constexpr uint8_t kBoltRows[8] = {0x38, 0x30, 0x60, 0x7C, 0x18, 0x30, 0x60, 0x40};
constexpr int kBoltW = 7;
constexpr int kBoltH = 8;

}  // namespace

void EmoteDisplay::SetBatteryStatus(int percent, bool charging) {
    if (emote_handle_ == nullptr || width_ <= 0) {
        return;
    }
    const int clamped = percent < 0 ? 0 : (percent > 100 ? 100 : percent);
    if (badge_ != nullptr && clamped == badge_percent_ && charging == badge_charging_) {
        return;
    }
    badge_percent_ = clamped;
    badge_charging_ = charging;
    PaintBatteryBadge(clamped, charging);
}

void EmoteDisplay::PaintBatteryBadge(int percent, bool charging) {
    if (badge_ == nullptr) {
        const size_t rgb_bytes = static_cast<size_t>(kBadgeW) * kBadgeH * 2;
        const size_t alpha_bytes = static_cast<size_t>(kBadgeW) * kBadgeH;
        auto* state = new BatteryBadge();
        uint8_t* mem = static_cast<uint8_t*>(
            heap_caps_malloc(rgb_bytes + alpha_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (mem == nullptr) {
            mem = static_cast<uint8_t*>(heap_caps_malloc(rgb_bytes + alpha_bytes, MALLOC_CAP_8BIT));
        }
        state->obj = mem != nullptr
                         ? emote_create_obj_by_type(emote_handle_, EMOTE_OBJ_TYPE_IMAGE, "battery_badge")
                         : nullptr;
        if (state->obj == nullptr) {
            // 资源区还没挂好或内存不够都会走到这里，下次调用再试
            ESP_LOGW(TAG, "battery badge: create failed, retry later");
            heap_caps_free(mem);
            delete state;
            badge_percent_ = -1;
            return;
        }
        state->rgb = reinterpret_cast<uint16_t*>(mem);
        state->alpha = mem + rgb_bytes;

        state->dsc.header.magic = C_ARRAY_HEADER_MAGIC;
        state->dsc.header.cf = GFX_COLOR_FORMAT_RGB565A8;
        state->dsc.header.flags = 0;
        state->dsc.header.w = kBadgeW;
        state->dsc.header.h = kBadgeH;
        state->dsc.header.stride = kBadgeW * 2;
        state->dsc.header.reserved = 0;
        state->dsc.data = mem;
        state->dsc.data_size = static_cast<uint32_t>(rgb_bytes + alpha_bytes);
        badge_ = state;
    }

    const uint16_t white = PackRgb565(255, 255, 255);
    const uint16_t level_color = percent <= 20 ? PackRgb565(255, 59, 48)
                                               : (percent <= 50 ? PackRgb565(255, 179, 0)
                                                                : PackRgb565(76, 217, 100));

    constexpr int kFillX0 = 5;
    constexpr int kFillX1 = 34;
    constexpr int kFillSpan = kFillX1 - kFillX0 + 1;
    int filled = kFillSpan * percent / 100;
    if (percent > 0 && filled == 0) {
        filled = 1;  // 只要还有电，就留一格，别显示成空壳
    }

    for (int y = 0; y < kBadgeH; y++) {
        for (int x = 0; x < kBadgeW; x++) {
            uint16_t color = 0;
            uint8_t alpha = 0;
            // 黑色半透明底衬：保证在亮色表情上也看得清
            if (InRoundedRect(x, y, 0, 0, kBadgeW - 1, kBadgeH - 1, 7)) {
                alpha = 70;
            }
            // 电池外壳与内腔
            if (InRoundedRect(x, y, 2, 2, 37, 17, 5)) {
                if (!InRoundedRect(x, y, 4, 4, 35, 15, 4)) {
                    color = white;
                    alpha = 235;
                } else if (x >= kFillX0 && x < kFillX0 + filled) {
                    color = level_color;
                    alpha = 240;
                } else {
                    alpha = 0;  // 空的那一截透出底衬
                }
            }
            // 正极触点
            if (x >= 39 && x <= 42 && y >= 7 && y <= 12) {
                color = white;
                alpha = 235;
            }
            const int i = y * kBadgeW + x;
            badge_->rgb[i] = color;
            badge_->alpha[i] = alpha;
        }
    }

    if (charging) {
        constexpr int kX = 16;
        constexpr int kY = 5;
        for (int row = 0; row < kBoltH; row++) {
            for (int col = 0; col < kBoltW; col++) {
                if ((kBoltRows[row] >> (kBoltW - 1 - col)) & 1) {
                    const int i = (kY + row) * kBadgeW + (kX + col);
                    badge_->rgb[i] = white;
                    badge_->alpha[i] = 245;
                }
            }
        }
    }

    emote_lock(emote_handle_);
    gfx_obj_set_pos(badge_->obj, width_ - kBadgeW - kBadgeMargin, kBadgeMargin);
    gfx_obj_set_size(badge_->obj, kBadgeW, kBadgeH);
    gfx_img_set_src(badge_->obj, &badge_->dsc);
    if (!gfx_obj_get_visible(badge_->obj)) {
        gfx_obj_set_visible(badge_->obj, true);
    }
    emote_unlock(emote_handle_);

    // 表情静止时渲染任务可能不再出帧，主动催一帧，否则角标要等到下次动图才更新
    emote_notify_all_refresh(emote_handle_);
}

EmoteDisplay::EmoteDisplay(const esp_lcd_panel_handle_t panel,
                           const esp_lcd_panel_io_handle_t panel_io, const int width,
                           const int height)
    : width_(width), height_(height) {
    emote_handle_ = InitializeEmote(panel, width, height);

    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = OnFlushIoReady,
    };
    esp_lcd_panel_io_register_event_callbacks(panel_io, &cbs, emote_handle_);
}

EmoteDisplay::~EmoteDisplay() {
    if (badge_ != nullptr) {
        // gfx 对象由 emote_deinit 一并销毁，这里只回收像素内存
        heap_caps_free(badge_->rgb);
        delete badge_;
        badge_ = nullptr;
    }
    if (preview_ != nullptr) {
        if (preview_->timer != nullptr) {
            esp_timer_delete(preview_->timer);
        }
        // 图像对象挂在 display 的子对象链上，emote_deinit 会一并销毁
        for (auto* b : preview_->buf) {
            if (b != nullptr) {
                heap_caps_free(b);
            }
        }
        delete preview_;
        preview_ = nullptr;
    }
    if (emote_handle_) {
        emote_deinit(emote_handle_);
        emote_handle_ = nullptr;
    }
}

bool EmoteDisplay::SetEmotion(const char* const emotion) {
    ESP_LOGI(TAG, "SetEmotion: %s", emotion);
    if (emote_handle_ == nullptr || emotion == nullptr || strlen(emotion) == 0) {
        return false;
    }
    // The result was already being produced and discarded. An emotion the mounted asset pack
    // does not carry fails inside emote_set_emoji_animation, and callers had no way to find
    // out: an MCP tool, a server-driven emotion or a board's own avatar mapping all saw the
    // same silence as a success.
    esp_err_t err = emote_set_anim_emoji(emote_handle_, emotion);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "emotion \"%s\" not set: %s", emotion, esp_err_to_name(err));
        return false;
    }
    return true;
}

void EmoteDisplay::SetChatMessage(const char* const role, const char* const content) {
    ESP_LOGI(TAG, "SetChatMessage: %s, %s", role, content);
    if (emote_handle_ && content && strlen(content) > 0) {
        if ((std::strcmp(role, "system") == 0) && std::strstr(content, "xiaozhi.me")) {
            size_t len = strlen(content);
            char* new_content = new char[len + 1];
            strcpy(new_content, content);
            std::replace(new_content, new_content + len, static_cast<char>(0x0A),
                         static_cast<char>(0x20));
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_SYS, new_content);
            delete[] new_content;
        } else {
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_SPEAK, content);
        }
    }
}

void EmoteDisplay::SetStatus(const char* const status) {
    ESP_LOGI(TAG, "SetStatus: %s", status);
    if (emote_handle_ && status && strlen(status) > 0) {
        if (std::strcmp(status, Lang::Strings::LISTENING) == 0) {
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_LISTEN, NULL);
        } else if (std::strcmp(status, Lang::Strings::STANDBY) == 0) {
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_IDLE, NULL);
        } else if (std::strcmp(status, Lang::Strings::SPEAKING) == 0) {
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_SPEAK, NULL);
        } else if (std::strcmp(status, Lang::Strings::ERROR) == 0) {
            emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_SET, NULL);
        }
    }
}

void EmoteDisplay::ShowNotification(const char* notification, int duration_ms) {
    ESP_LOGI(TAG, "ShowNotification: %s", notification);
    if (emote_handle_ && notification && strlen(notification) > 0) {
        emote_set_event_msg(emote_handle_, EMOTE_MGR_EVT_SYS, notification);
    }
}

void EmoteDisplay::UpdateStatusBar(bool update_all) {
    ESP_LOGD(TAG, "UpdateStatusBar: %s", update_all ? "true" : "false");
    if (!emote_handle_) {
        return;
    }
}

void EmoteDisplay::SetPowerSaveMode(bool on) {
    ESP_LOGI(TAG, "SetPowerSaveMode: %s", on ? "ON" : "OFF");
    if (!emote_handle_) {
        return;
    }
}

// ============================================================================
// PHOTO_PREVIEW: 拍照后在屏幕上显示几秒刚拍的照片
// ============================================================================
//
// 走 gfx 的自定义 image 对象而不是直接 esp_lcd_panel_draw_bitmap：
// 表情由 emote 的渲染任务逐帧重绘，绕开它往 panel 上写会被下一帧覆盖；
// 而渲染任务的刷屏等待没有任何公开的暂停接口，硬抢会把它卡在 flush 上。
// 做成子对象后，它排在列表末尾（后创建 = 后绘制），每帧盖在表情上面，
// 收起时置 visible=false，渲染任务自己会把这块区域按背景色重画回表情。

// 一张照片停留多久。改这里就是改体验，注意别和"拍完就开始说话"打架。
static constexpr uint32_t kPreviewDurationMs = 5000;

void EmoteDisplay::OnPreviewTimeout(void* arg) {
    if (arg != nullptr) {
        static_cast<EmoteDisplay*>(arg)->HidePreview();
    }
}

void EmoteDisplay::HidePreview() {
    if (preview_ == nullptr || preview_->obj == nullptr || emote_handle_ == nullptr) {
        return;
    }
    emote_lock(emote_handle_);
    gfx_obj_set_visible(preview_->obj, false);
    emote_unlock(emote_handle_);
    // 图传停掉 5 秒后才会走到这里；流还在的时候这条日志一次都不该出现
    ESP_LOGI(TAG, "preview: hidden, back to the face");
}

void EmoteDisplay::SetPreviewImage(std::unique_ptr<LvglImage> image) {
    if (emote_handle_ == nullptr || image == nullptr) {
        return;
    }
    const lv_img_dsc_t* src = image->image_dsc();
    if (src == nullptr || src->data == nullptr || src->header.w == 0 || src->header.h == 0) {
        ESP_LOGW(TAG, "preview: empty image, ignored");
        return;
    }
    // 摄像头预览这条路径只会给我们 RGB565（YUV/JPEG 在 esp_video 里已经转好了）
    if (src->header.cf != LV_COLOR_FORMAT_RGB565) {
        ESP_LOGW(TAG, "preview: color format %u not supported, only RGB565 is drawn",
                 static_cast<unsigned>(src->header.cf));
        return;
    }
    if (width_ <= 0 || height_ <= 0) {
        return;
    }

    if (preview_ == nullptr) {
        auto state = std::make_unique<PreviewState>();
        const size_t bytes = static_cast<size_t>(width_) * height_ * 2;
        for (int i = 0; i < 2; i++) {
            state->buf[i] = static_cast<uint16_t*>(
                heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
            if (state->buf[i] == nullptr) {
                state->buf[i] = static_cast<uint16_t*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
            }
            if (state->buf[i] == nullptr) {
                ESP_LOGE(TAG, "preview: no memory for a %dx%d frame (%u bytes)", width_, height_,
                         static_cast<unsigned>(bytes));
                return;
            }
        }

        state->obj = emote_create_obj_by_type(emote_handle_, EMOTE_OBJ_TYPE_IMAGE, "photo_preview");
        if (state->obj == nullptr) {
            ESP_LOGE(TAG, "preview: failed to create photo_preview object");
            for (auto* b : state->buf) {
                heap_caps_free(b);
            }
            return;
        }

        // 描述符布局和 lv_image_header_t 完全一致，gfx 只认第一个字节的 magic，
        // 数据按行紧凑排布（gfx 的绘制路径固定用 image_width 当 stride）。
        for (int i = 0; i < 2; i++) {
            state->dsc[i].header.magic = C_ARRAY_HEADER_MAGIC;
            state->dsc[i].header.cf = GFX_COLOR_FORMAT_RGB565;
            state->dsc[i].header.flags = 0;
            state->dsc[i].header.w = static_cast<uint16_t>(width_);
            state->dsc[i].header.h = static_cast<uint16_t>(height_);
            state->dsc[i].header.stride = static_cast<uint16_t>(width_ * 2);
            state->dsc[i].header.reserved = 0;
            state->dsc[i].data = reinterpret_cast<const uint8_t*>(state->buf[i]);
            state->dsc[i].data_size = static_cast<uint32_t>(bytes);
        }

        esp_timer_create_args_t timer_args = {};
        timer_args.callback = OnPreviewTimeout;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "photo_preview";
        if (esp_timer_create(&timer_args, &state->timer) != ESP_OK) {
            ESP_LOGW(TAG, "preview: timer unavailable, the photo will stay on screen");
            state->timer = nullptr;
        }

        preview_ = state.release();
    }

    const int src_w = src->header.w;
    const int src_h = src->header.h;
    const size_t src_stride = src->header.stride != 0 ? src->header.stride : src_w * 2;

    // 屏幕是方的，照片不是：短边铺满、长边居中裁切（cover），不留黑边也不变形
    int crop_w = src_w;
    int crop_h = src_h;
    if (static_cast<int64_t>(src_w) * height_ > static_cast<int64_t>(src_h) * width_) {
        crop_w = static_cast<int>(static_cast<int64_t>(src_h) * width_ / height_);
    } else {
        crop_h = static_cast<int>(static_cast<int64_t>(src_w) * height_ / width_);
    }
    const int x0 = (src_w - crop_w) / 2;
    const int y0 = (src_h - crop_h) / 2;

    // 写后块、显前块：渲染任务这一帧读的仍是上一块，写和读不会撞在同一片内存上，
    // 所以既不用「先隐藏再重显」，也不会看到半新半旧的撕裂帧。
    const int back = preview_->front ^ 1;

    for (int y = 0; y < height_; y++) {
        const int sy = y0 + static_cast<int>(static_cast<int64_t>(y) * crop_h / height_);
        const uint8_t* row = src->data + static_cast<size_t>(sy) * src_stride;
        uint16_t* dst = preview_->buf[back] + static_cast<size_t>(y) * width_;
        for (int x = 0; x < width_; x++) {
            const int sx = x0 + static_cast<int>(static_cast<int64_t>(x) * crop_w / width_);
            uint16_t px;
            memcpy(&px, row + static_cast<size_t>(sx) * 2, sizeof(uint16_t));
            // gfx 的图像绘制路径是裸 memcpy，注释写着 "src_buf is expected to already be
            // in native framebuffer order"——它不会像 EAF 解码器那样调
            // gfx_color_to_native_u16 做转换。而本板 emote 初始化设了 swap=true，
            // 帧缓冲要的是字节翻转的 RGB565；摄像头给的是小端。不翻就是满屏雪花。
            dst[x] = static_cast<uint16_t>((px >> 8) | (px << 8));
        }
    }

    emote_lock(emote_handle_);
    gfx_obj_set_pos(preview_->obj, 0, 0);
    gfx_obj_set_size(preview_->obj, static_cast<uint16_t>(width_),
                     static_cast<uint16_t>(height_));
    gfx_img_set_src(preview_->obj, &preview_->dsc[back]);
    if (!gfx_obj_get_visible(preview_->obj)) {
        gfx_obj_set_visible(preview_->obj, true);
    }
    preview_->front = back;
    emote_unlock(emote_handle_);

    if (preview_->timer != nullptr) {
        // 连拍时重新计时，而不是让第一张的计时器提前把第二张收掉
        esp_timer_stop(preview_->timer);
        esp_timer_start_once(preview_->timer, static_cast<uint64_t>(kPreviewDurationMs) * 1000);
    }

    // 图传连续出帧时这里每秒会调十几次，别刷串口日志；攒够 2 秒汇总一次，
    // 出问题时能看出「帧在更新」还是「画面冻住了」。
    static int64_t s_preview_log_us = 0;
    static uint32_t s_preview_frames = 0;
    s_preview_frames++;
    const int64_t now_us = esp_timer_get_time();
    if (s_preview_log_us == 0 || now_us - s_preview_log_us >= 2000000) {
        ESP_LOGI(TAG, "preview: %" PRIu32 " frames in %" PRId64 " ms, %dx%d cover-cropped from %dx%d",
                 s_preview_frames, (now_us - s_preview_log_us) / 1000, width_, height_, src_w,
                 src_h);
        s_preview_frames = 0;
        s_preview_log_us = now_us;
    }
}

bool EmoteDisplay::MountAssets(const char* partition_label) {
    if (emote_handle_ == nullptr || partition_label == nullptr) {
        return false;
    }
    const emote_data_t data = {
        .type = EMOTE_SOURCE_PARTITION,
        .source =
            {
                .partition_label = partition_label,
            },
        .flags =
            {
                .mmap_enable = true,
            },
    };
    return emote_mount_assets(emote_handle_, &data) == ESP_OK;
}

void EmoteDisplay::UnmountAssets() {
    if (emote_handle_ != nullptr) {
        emote_unmount_assets(emote_handle_);
    }
}

bool EmoteDisplay::GetAssetData(const std::string& name, const uint8_t*& data, size_t& size) {
    if (emote_handle_ == nullptr) {
        return false;
    }
    return emote_get_asset_data_by_name(emote_handle_, name.c_str(), &data, &size) == ESP_OK;
}

void EmoteDisplay::LoadAssets() {
    if (emote_handle_ != nullptr) {
        emote_load_assets(emote_handle_);
    }
}

void EmoteDisplay::SetTheme(Theme* const theme) { ESP_LOGI(TAG, "SetTheme: %p", theme); }

bool EmoteDisplay::Lock(const int timeout_ms) {
    (void)timeout_ms;
    return true;
}

void EmoteDisplay::Unlock() {}

bool EmoteDisplay::StopAnimDialog() {
    ESP_LOGI(TAG, "StopAnimDialog");
    if (emote_handle_) {
        // esp_err_t, where ESP_OK is 0: returned as bool it reports false on success and true on
        // failure. GetAssetData and SetEmotion compare against ESP_OK for the same reason.
        return emote_stop_anim_dialog(emote_handle_) == ESP_OK;
    }
    return false;
}

bool EmoteDisplay::InsertAnimDialog(const char* emoji_name, uint32_t duration_ms) {
    ESP_LOGI(TAG, "InsertAnimDialog: %s, %" PRIu32, emoji_name, duration_ms);
    if (emote_handle_ && emoji_name) {
        return emote_insert_anim_dialog(emote_handle_, emoji_name, duration_ms) == ESP_OK;
    }
    return false;
}

void EmoteDisplay::RefreshAll() {
    if (emote_handle_) {
        emote_notify_all_refresh(emote_handle_);
        return;
    }
}

}  // namespace emote
