#pragma once

#include <driver/i2c_master.h>

#include <cstdint>

// ===========================================================================
// 六轴 IMU（加速度计 + 陀螺）
//
// 这块板子的 I2C 上挂着 ES8311、摄像头 SCCB 和 IMU（地址 0x68），三者共用
// GPIO4/5 这一条总线。INT 脚没引到 S3，只能轮询采样。
//
// 实测这颗芯片 chip id = 0x27，即 Bosch BMI260（不是 BMI270 的 0x24，
// 也不是官方适配过的 BMI220 的 0x26）。
//
// BMI260 与 BMI270 寄存器地图基本一致，但 init data 不同且**必须上传**：
// 不上传时寄存器写得进、读得回，加速度却恒为 0，很容易误判成硬件坏了。
// 所以初始化流程是「软复位 → 上传 8KB init data → 配 ODR/量程 → 使能」，
// 数据从 0x0C 起 burst 读出。上电后用「静止时加速度模长≈1g」做自检。
//
// 采样率 100Hz。迷宫类体感游戏 50Hz 都够，数据量 1.2KB/s 可忽略。
// ===========================================================================

struct ImuReadout {
    bool ready = false;
    uint8_t chip_id = 0;

    // 原始 LSB。做坐标系标定时看这三个值最直接，别被换算后的角度绕晕。
    int16_t acc_lsb[3] = {0, 0, 0};
    int16_t gyr_lsb[3] = {0, 0, 0};

    // 换算后的物理量
    float acc_g[3] = {0.0f, 0.0f, 0.0f};
    float gyr_dps[3] = {0.0f, 0.0f, 0.0f};

    // 互补滤波后的姿态角（度）。yaw 没有磁力计做绝对参考，会随时间漂移，
    // 只能拿来看短时转动，不能当罗盘用。
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // 相对零点标定的倾角，游戏直接用这个。
    float tilt_x = 0.0f;
    float tilt_y = 0.0f;

    uint32_t seq = 0;
    uint16_t sample_hz = 0;
};

namespace Bmi270Imu {

// 初始化并起 100Hz 采样任务。可在板级构造函数里调用，约阻塞 150ms
// （主要是上传 8KB 配置文件）。
bool Start(i2c_master_bus_handle_t bus, uint8_t addr = 0x68);

// 拷贝最近一帧，线程安全。
bool GetLatest(ImuReadout* out);

// 把当前姿态记为零点，之后 tilt_x / tilt_y 相对它计算。
// 机器人放桌上不一定水平，这个必须能标。
void Zero();

// 串口波形开关，默认开。验证完可以关掉省日志。
void SetSerialLog(bool on);

uint8_t ChipId();

// 0 表示正常；非 0 是 bmi2 的返回码（正数）或 ESP-IDF 错误（负数）。
int LastInitError();

// 拼一段 JSON 供 /imu 接口使用。
// 放在这里而不是 local_control 里，是因为字段含义只有驱动自己清楚。
int FormatJson(char* buf, size_t len);

}  // namespace Bmi270Imu
