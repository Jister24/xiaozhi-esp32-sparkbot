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
// 姿态解算照搬 PX4 `attitude_estimator_q`（四元数非线性互补滤波）：
// 陀螺积分做预测，重力方向叉积做校正，零偏用校正量积分估计并限幅。
// 四元数在这里的最大好处是没有欧拉角奇异、也没有「两路角度互相耦合」，
// 姿态本身在任意角度都准。
//
// 注意倾角输出不是「旋转向量」，而是 roll / pitch 之差：旋转向量会把
// yaw 一起裹进去——手腕边倾边拧时重力方向只转了 39°，旋转向量却涨到
// 59°，实测过读 20°。roll / pitch 由「重力在机体系的方向」定义，绕竖直
// 轴自转不改变它，因此对 yaw 免疫。
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

    // 姿态四元数（机体系 → 世界系，w 在前）。世界系 z 轴朝上，
    // 初始航向任意（没有磁力计，航向本来就不可观测）。
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // 在线估计出的陀螺零偏（dps），已从积分里扣掉。x/y 轴由重力校正
    // 持续约束，可信；z 轴（航向）重力观测不到，估不出来，正常情况会停在
    // 0 附近——所以要靠 kYawDeadbandDps 把残余漂移压住，不能指望这个值。
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

    // 由四元数算出的欧拉角（度）。yaw 会随时间漂移，只能看短时转动。
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    // 相对零点的倾角，游戏直接用这个。就是 roll / pitch 分别减去
    // 零点时的值，所以对「绕竖直轴自转」免疫：边倾边拧手腕不会让读数虚高。
    // 两个轴同时倾很大时向量和会比真实转角略大（30~40° 约 1°，极端斜角
    // 最多 9°），做摇杆可以忽略。
    // 零点默认是「水平面」（上电时不变），也就是 tilt 从开机起就等于
    // 板子相对水平的真实倾角；要改成手持中性位就调一次 Zero()。
    float tilt_x = 0.0f;
    float tilt_y = 0.0f;

    uint32_t seq = 0;
    uint16_t sample_hz = 0;
};

namespace Bmi260Imu {

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

}  // namespace Bmi260Imu
