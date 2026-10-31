#pragma once

#include <cstdint>

// 履带底盘电量读取。
//
// 底盘那颗 C2 一直在采自己的电池（ADC1_CH3，两倍分压，GPIO2 低电平表示在充电），
// 但原本只拿去喂 RGB 灯，值烂在任务局部变量里、不走总线。这里通过板间 UART 主动
// 问一次，C2 回一行 `B:<mV>:<pct>:<chg>\n`。
//
// 两个前提：
//   - 旧版底盘固件会把收到的命令原样回显。解析只认大写 'B'（命令都是小写），
//     所以新旧固件任意组合都能容忍：旧底盘就是永远读不到，上层把字段省略。
//   - 底盘有心跳看门狗，约 500ms 没有运动指令就停车。查询用的 `b0` 不喂看门狗，
//     所以问电量不会把正在跑的车停住。
class ChassisBattery {
public:
    void Begin();

    // 最近 30 秒内成功读到过一次才算可用
    bool IsValid() const;
    int Millivolts() const { return millivolts_; }
    int Percent() const { return percent_; }
    bool IsCharging() const { return charging_; }

    void ParseLine(const char* line);

private:
    void Poll();
    static void TaskEntry(void* arg);

    volatile int millivolts_ = 0;
    volatile int percent_ = 0;
    volatile bool charging_ = false;
    volatile int64_t last_ok_us_ = 0;
};
