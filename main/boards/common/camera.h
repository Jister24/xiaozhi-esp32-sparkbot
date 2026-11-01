#ifndef CAMERA_H
#define CAMERA_H

#include <expected>
#include <string>
#include <vector>  // LOCAL_CONTROL_PATCH

class Camera {
public:
    virtual ~Camera() = default;

    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op

    // LOCAL_CONTROL_PATCH: 图像方向。板载传感器倒装，固件默认整帧旋转 180°。
    // 不支持软件旋转的板型保持 no-op 即可。
    virtual void SetRotate180(bool enabled) { (void)enabled; }
    // LOCAL_CONTROL_PATCH: 只做左右镜像，与 180° 旋转正交。180° 开启时再叠一次
    // 水平镜像，净效果等于纯垂直翻转（传感器上下倒装、左右本来是正的场合）。
    virtual void SetMirrorHorizontal(bool enabled) { (void)enabled; }
    virtual std::expected<std::string, std::string> Explain(const std::string& question) = 0;

    // LOCAL_CONTROL_PATCH: 局域网遥控取图接口。默认实现返回 false，
    // 不支持该能力的板型无需改动。
    virtual bool GetJpegFrame(std::vector<uint8_t>& out) {
        (void)out;
        return false;
    }
};

#endif  // CAMERA_H
