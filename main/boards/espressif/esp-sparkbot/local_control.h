#pragma once

#include <functional>
#include <string>

// 局域网本地控制服务（不依赖小智云端）
//
// 启动后监听两个端口：
//   8080 — 纯控制接口，响应 <10ms，不会被抓图阻塞
//          GET /            控制页面
//          GET /move?x=&y=  运动，取值 -1.0 ~ 1.0（x 转向，y 前后）
//          GET /stop        急停
//          GET /cmd?c=      下发原始底盘指令，如 d1 / w2 / x0.5 y0.5
//          GET /status      状态 JSON
//   8081 — 图像接口
//          GET /stream      MJPEG 连续流（浏览器 <img> 可直接显示）
//          GET /snapshot    单帧 JPEG
//
// send_command 用于把指令写入履带底盘 UART，由板级代码注入。
// battery_provider 可选，注入后 /status 会带上主体板电池信息。
// chassis_battery_provider 可选，注入后 /status 会带 chassis_* 前缀的底盘电池信息；
// 底盘电池在 C2 那一侧采样，主体板只能通过 UART 问来，问不到就省略字段。
//
// imu_provider 可选，注入后多出两个端点：
//   GET /imu       返回 IMU 当前读数（JSON），体感游戏和标定脚本都用它
//   GET /imu/zero  把当前姿态记为零点
// 这里只收 JSON 字符串而不是具体结构体，是为了让 local_control 不必知道
// IMU 的字段含义——驱动改了字段不用动这个文件。
using BatteryProvider = std::function<bool(int& cell_mv, int& percent, bool& charging)>;
using ImuProvider = std::function<std::string()>;
using ImuZeroAction = std::function<void()>;

void LocalControlStart(std::function<void(const char*)> send_command,
                       BatteryProvider battery_provider = nullptr,
                       BatteryProvider chassis_battery_provider = nullptr,
                       ImuProvider imu_provider = nullptr,
                       ImuZeroAction imu_zero = nullptr);

// 当前是否处在 Wi-Fi 省电状态，也就是「没有远程客户端在用的空闲期」。
// 板级据此在屏幕上换成睡觉表情（见 esp_sparkbot_board.cc 的 UpdateSleepFace），
// 有请求进来的一瞬间就会变回 false，脸也跟着醒。
bool LocalControlPowerSaveOn();

// 上层（application）想不想省电，由板级从 SetPowerSaveLevel 转过来。
//   low_power = true  → 空闲时可以退到 WIFI_PS_MIN_MODEM
//   low_power = false → 上层在用（对话 / 联网 / OTA），必须保持 WIFI_PS_NONE
// 实际档位由 local_control 统一裁决：USB 接主机、最近 2 分钟有客户端请求、
// 或者上层要性能，任一成立就不省电。板级不要自己再调 esp_wifi_set_ps，
// 否则两条路径会互相覆盖。
void LocalControlNotePowerSaveWish(bool low_power);
