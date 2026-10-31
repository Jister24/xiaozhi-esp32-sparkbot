#include "chassis_battery.h"

#include <cstdio>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

namespace {
constexpr const char* kTag = "SparkBotChassisBatt";
// 5 秒问一次：电量变化慢，没必要更勤；UART 上就两个字节
constexpr int64_t kPollIntervalUs = 5 * 1000 * 1000;
constexpr int64_t kStaleUs = 30 * 1000 * 1000;
}  // namespace

void ChassisBattery::Begin() {
    xTaskCreate(TaskEntry, "chassis_batt", 3072, this, 2, nullptr);
}

void ChassisBattery::TaskEntry(void* arg) {
    static_cast<ChassisBattery*>(arg)->Poll();
}

bool ChassisBattery::IsValid() const {
    const int64_t last = last_ok_us_;
    return last > 0 && esp_timer_get_time() - last < kStaleUs;
}

void ChassisBattery::ParseLine(const char* line) {
    int mv = 0;
    int pct = 0;
    int chg = 0;
    if (sscanf(line, "B:%d:%d:%d", &mv, &pct, &chg) != 3) {
        return;
    }
    // 3.0~4.4V 之外不可能是锂电池读数，当作噪声丢掉
    if (mv < 3000 || mv > 4400) {
        ESP_LOGW(kTag, "忽略可疑读数: %d mV", mv);
        return;
    }
    millivolts_ = mv;
    percent_ = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
    charging_ = chg != 0;
    last_ok_us_ = esp_timer_get_time();
    ESP_LOGI(kTag, "底盘电量 %d mV / %d%% / %s", millivolts_, percent_,
             charging_ ? "充电中" : "放电中");
}

void ChassisBattery::Poll() {
    uint8_t buf[128];
    char line[24];
    int n = 0;
    int64_t last_ping_us = 0;

    while (true) {
        const int64_t now = esp_timer_get_time();
        if (now - last_ping_us >= kPollIntervalUs) {
            last_ping_us = now;
            const char kQuery[] = "b0";
            uart_write_bytes(ECHO_UART_PORT_NUM, kQuery, 2);
        }

        const int len = uart_read_bytes(ECHO_UART_PORT_NUM, buf, sizeof(buf),
                                        pdMS_TO_TICKS(200));
        for (int i = 0; i < len; i++) {
            const char c = static_cast<char>(buf[i]);
            if (c == 'B') {
                // 任何位置出现的 'B' 都当成一行的开头，避免半截帧卡住解析
                line[0] = c;
                n = 1;
                continue;
            }
            if (n == 0) {
                continue;  // 回显、别的噪声，直接丢
            }
            if (c == '\n') {
                line[n] = '\0';
                ParseLine(line);
                n = 0;
            } else if (c != '\r' && n < static_cast<int>(sizeof(line)) - 1) {
                line[n++] = c;
            } else if (c != '\r') {
                n = 0;  // 太长了，不可能是应答
            }
        }
    }
}
