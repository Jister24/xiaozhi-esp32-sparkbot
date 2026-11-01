#pragma once
#include "sdkconfig.h"

#include <lvgl.h>
#include <memory>
#include <thread>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"
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
    bool rotate180_ = true;
    bool mirror_h_ = false;

public:
};
