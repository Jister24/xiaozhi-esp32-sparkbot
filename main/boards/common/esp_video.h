#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <memory>
#include <thread>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
#include "esp_imgfx_scale.h"
#include "esp_video_init.h"
#include "jpg/image_to_jpeg.h"

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class EspVideo : public Camera {
private:
    struct FrameBuffer {
        uint8_t* data = nullptr;
        size_t len = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        v4l2_pix_fmt_t format = 0;
    } frame_;
    v4l2_pix_fmt_t sensor_format_ = 0;
#ifdef CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    uint16_t sensor_width_ = 0;
    uint16_t sensor_height_ = 0;
#endif  // CONFIG_XIAOZHI_ENABLE_ROTATE_CAMERA_IMAGE
    int video_fd_ = -1;
    bool streaming_on_ = false;
    struct MmapBuffer {
        void* start = nullptr;
        size_t length = 0;
    };
    std::vector<MmapBuffer> mmap_buffers_;
    std::string explain_url_;
    std::string explain_token_;
    std::thread encoder_thread_;

public:
    EspVideo(const esp_video_init_config_t& config);
    ~EspVideo() override;

    virtual void SetExplainUrl(const std::string& url, const std::string& token);
    virtual bool Capture();
    // 翻转控制函数
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::expected<std::string, std::string> Explain(const std::string& question) override;

    // LOCAL_CONTROL_PATCH: 抓取一帧并编码为 JPEG，供局域网遥控使用
    virtual bool GetJpegFrame(std::vector<uint8_t>& out) override;


    // LOCAL_CONTROL_PATCH: 整帧 180° 旋转（传感器倒装）。纯软件实现，
    // 同时作用于 LCD 预览、局域网图传与云端拍照。
    virtual void SetRotate180(bool enabled) override;

    // LOCAL_CONTROL_PATCH: 左右镜像（逐行像素倒排），同样作用在 Capture() 里，
    // 三条取图链路口径一致。默认关闭，由 NVS 里的方向设置决定。
    virtual void SetMirrorHorizontal(bool enabled) override;

private:
    void RotateFrame180();
    void MirrorFrameHorizontal();
    // 缩放后固定是 RGB565，单独给录制模式用的小图旋转
    static void RotateRgb565Frame180(uint8_t* data, size_t len);
    bool rotate180_ = true;
    bool mirror_h_ = false;

    // CAMERA_MODE_SWITCH: 录制模式下把 640x480 降采样到 320x240 再送编码。
    // sensor 固定 640x480 输出：esp_video 的设备层在创建时就把宽高写死了，
    // VIDIOC_S_FMT 改不动尺寸，只能自己缩。结果放在 scale_buf_。
    bool ScaleFrameTo(uint16_t dst_w, uint16_t dst_h);
    bool recording_mode_ = false;
    std::vector<uint8_t> scale_buf_;
    esp_imgfx_scale_handle_t scale_handle_ = nullptr;
    uint16_t scale_src_w_ = 0;
    uint16_t scale_src_h_ = 0;
    uint16_t scale_dst_w_ = 0;
    uint16_t scale_dst_h_ = 0;

public:
    // CAMERA_PERF_PROBE: 分段耗时探针，用于定位抓帧链路的真实瓶颈。
    // 只在 GetJpegFrame()（局域网图传/拍照）里打点，不影响云端拍照链路。
    struct CapturePerf {
        int64_t dbuf_us = 0;     // DQBUF x3 + PSRAM 拷贝 + 字节序转换
        int64_t rotate_us = 0;   // 180° 旋转 + 水平镜像
        int64_t display_us = 0;  // LCD 预览刷新
        int64_t scale_us = 0;    // 录制模式降采样到 320x240
        int64_t jpeg_us = 0;     // 软件 JPEG 编码
        int64_t total_us = 0;    // Capture + JPEG 全程
        uint16_t width = 0;
        uint16_t height = 0;
        size_t raw_len = 0;
        size_t jpeg_len = 0;
    };
    // CAMERA_MODE_SWITCH: 录制模式把取到的帧降采样到 320x240 再编码，
    // 编码量降到 1/4；拍照模式保持 640x480 全分辨率。
    bool SetRecordingMode(bool recording) override;
    bool IsRecordingMode() const override { return recording_mode_; }

    const CapturePerf& GetCapturePerf() const { return perf_; }

    // CAMERA_PERF_PROBE: 局域网图传时不需要刷新 LCD 预览（屏上显示的是表情），
    // 跳过后每帧省掉一次 600KB 分配 + 拷贝 + 刷屏。
    void SetSkipPreview(bool skip) { skip_preview_ = skip; }

private:
    CapturePerf perf_;
    bool skip_preview_ = false;
    // 录制模式下 180° 旋转推迟到降采样之后做：640x480 转一次 37ms，
    // 320x240 只要约 9ms，省下来的时间直接变成帧率。
    bool skip_rotate_ = false;
};
