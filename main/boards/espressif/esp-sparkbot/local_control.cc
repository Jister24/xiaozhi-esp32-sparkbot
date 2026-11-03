#include "local_control.h"

#include <esp_http_server.h>
#include <sdkconfig.h>

#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) || \
    defined(CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG)
#include <driver/usb_serial_jtag.h>
#define SPARKBOT_HAVE_USB_SERIAL_JTAG 1
#endif

#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lwip/etharp.h>
#include <lwip/ip4_addr.h>
#include <lwip/netif.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "board.h"
#include "camera.h"
#include "display/emote_display.h"  // EMOTE_STYLE: /status 上报刷屏计数，便于远程判断屏幕是否在刷新
#include "settings.h"

namespace {

constexpr const char* TAG = "local_ctrl";

// 控制端口与图像端口分离：ESP-IDF 的 httpd 是单任务串行处理，
// 若把 MJPEG 长连接和运动指令放在同一实例，抓一帧的耗时会把指令阻塞住。
constexpr uint16_t kCtrlPort = 8080;
constexpr uint16_t kCamPort = 8081;

constexpr size_t kMaxJpegBytes = 200 * 1024;

std::function<void(const char*)> g_send;
BatteryProvider g_battery;
BatteryProvider g_chassis_battery;
ImuProvider g_imu;
ImuZeroAction g_imu_zero;
httpd_handle_t g_ctrl_server = nullptr;
httpd_handle_t g_cam_server = nullptr;
SemaphoreHandle_t g_cam_lock = nullptr;

float g_speed_x = 0.0f;
float g_speed_y = 0.0f;
const char* g_ip = "0.0.0.0";
int64_t g_boot_us = 0;

// 板载 GC2145 装反了，默认把画面转 180°；开关持久化在 NVS 里，掉电不丢
constexpr const char* kNvsNamespace = "local_ctrl";
constexpr const char* kNvsRotateKey = "rot180";
// 实测：上下是正的、左右是反的（180° 已经校正了上下，但把左右也镜像了），
// 所以再叠一次左右镜像，净效果等于只做垂直翻转。默认开启。
constexpr const char* kNvsMirrorKey = "mirrorh";
bool g_rot180 = true;
bool g_mirror_h = true;

// 正在推流的客户端数。远程隧道断掉时，httpd 那边可能还有一条卡住的流；
// 把它暴露到 /status 上，一眼能看出「连不上是因为流没释放」。
int g_stream_clients = 0;

// ---------------------------------------------------------------------------
// 公共工具
// ---------------------------------------------------------------------------

void AddCommonHeaders(httpd_req_t* req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

bool QueryValue(httpd_req_t* req, const char* key, char* out, size_t out_len) {
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    return httpd_query_key_value(query, key, out, out_len) == ESP_OK;
}

bool QueryFloat(httpd_req_t* req, const char* key, float* value) {
    char raw[24];
    if (!QueryValue(req, key, raw, sizeof(raw))) {
        return false;
    }
    char* end = nullptr;
    float parsed = strtof(raw, &end);
    if (end == raw || !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

float Clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void SendMotion(float x, float y) {
    float nx = Clamp(x, -1.0f, 1.0f);
    float ny = Clamp(y, -1.0f, 1.0f);
    bool changed = (nx != g_speed_x) || (ny != g_speed_y);
    g_speed_x = nx;
    g_speed_y = ny;
    char cmd[32];
    // 底盘侧 500ms 收不到 x/y 指令就会自动停车，所以重复指令必须照发，
    // 这里只在方向/速度真正变化时打日志，避免按住时刷屏。
    snprintf(cmd, sizeof(cmd), "x%.2f y%.2f", nx, ny);
    if (g_send) {
        g_send(cmd);
    }
    if (changed) {
        ESP_LOGI(TAG, "move %s", cmd);
    }
}

void SendRaw(const char* cmd) {
    if (g_send && cmd != nullptr) {
        g_send(cmd);
        ESP_LOGI(TAG, "raw cmd: %s", cmd);
    }
}

esp_err_t ReplyJson(httpd_req_t* req, const char* body) {
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

esp_err_t ReplyText(httpd_req_t* req, const char* text) {
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, text);
}

// ---------------------------------------------------------------------------
// Wi-Fi 省电模式：USB 供电 / 有客户端 / 上层在用，就退出；纯电池静置 2 分钟再回去
// ---------------------------------------------------------------------------
// 三档实际落到什么（见 78__esp-wifi-connect/wifi_station.cc）：
//   PERFORMANCE = WIFI_PS_NONE       射频常开
//   BALANCED    = WIFI_PS_MIN_MODEM  每 100ms 醒一次
//   LOW_POWER   = WIFI_PS_MAX_MODEM  监听间隔被拉到约 1 秒
//
// MAX_MODEM 在这台机器上是**不能用**的一档：实测它连 ARP 广播请求都收不到，
// 手机邻居表一过期就永远 "Destination Host Unreachable"（100% 丢包），
// RTT 743 / 1464ms。所以省电只退到 MIN_MODEM —— 100ms 唤醒间隔足够接住
// ARP 和 TCP 握手，功耗比 NONE 低，延迟又比 MAX_MODEM 好一个数量级。
//
// 退出省电的条件（任一成立）：
//   · USB 连着主机（插电脑烧固件 / adb / 看串口日志）；
//   · 最近 kPsIdleUs（2 分钟）内有 HTTP 请求（姿态页 20Hz 轮询时恒成立）；
//   · 上层（application）当前要求 PERFORMANCE —— 对话中、联网握手、OTA 都算。
constexpr int64_t kPsIdleUs = 120LL * 1000 * 1000;  // 纯电池静置 2 分钟

volatile int64_t g_last_request_us = 0;
bool g_ps_active = false;               // 当前是否真的处在「已退出省电」状态
volatile bool g_want_low_power = true;  // 上层意愿：true = 允许省电
volatile bool g_usb_present = false;    // USB 是否连着真主机（不是充电器）
volatile int32_t g_idle_sec = -1;       // 诊断用，-1 = 从来没收到过请求

// usb_serial_jtag_is_connected() 看的是 USB 枚举状态：插电脑会被枚举 → true，
// 插充电头/充电宝时 D+/D- 不通信、不会枚举 → false。这正是「插着 USB」想要的语义：
// 接着电脑说明可能有人在调试，别省电；插着充电器没人在用，该省还是省。
bool UsbHostConnected() {
#ifdef SPARKBOT_HAVE_USB_SERIAL_JTAG
    return usb_serial_jtag_is_connected();
#else
    return false;
#endif
}

// Wi-Fi 没起来时 esp_wifi_set_ps 会报 ESP_ERR_WIFI_NOT_STARTED，这是正常的：
// 本任务在联网前就已经在跑了，下一轮重试即可，不要刷警告。
void ApplyPowerMode(bool active) {
    if (active == g_ps_active) {
        return;
    }
    esp_err_t err = esp_wifi_set_ps(active ? WIFI_PS_NONE : WIFI_PS_MIN_MODEM);
    if (err == ESP_OK) {
        g_ps_active = active;
        ESP_LOGI(TAG, "wifi power save %s (usb=%d idle=%lds app_low_power=%d)",
                 active ? "off" : "on", (int)g_usb_present, (long)g_idle_sec,
                 (int)g_want_low_power);
    }
}

// 每个 HTTP 请求进来都调一次。发现已经掉回省电就当场切回来，
// 不等后台任务那 500ms 的轮询——第一帧遥控指令不该再吃一次唤醒延迟。
void NoteClientActivity() {
    g_last_request_us = esp_timer_get_time();
    ApplyPowerMode(true);
}

void PowerModeTask(void*) {
    while (true) {
        const int64_t now = esp_timer_get_time();
        const int64_t last = g_last_request_us;
        g_usb_present = UsbHostConnected();
        g_idle_sec = (last == 0) ? -1 : static_cast<int32_t>((now - last) / 1000000);
        const bool client_active = last != 0 && (now - last) < kPsIdleUs;
        // 上层在要 PERFORMANCE（对话 / 联网 / OTA）时也按活跃处理：
        // 这些场景丢一帧语音比省那几毫安严重得多。
        const bool app_active = !g_want_low_power;
        ApplyPowerMode(g_usb_present || client_active || app_active);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// 画面方向
// ---------------------------------------------------------------------------

bool LoadRotate180() {
    Settings settings(kNvsNamespace, false);
    return settings.GetBool(kNvsRotateKey, true);  // 默认转正
}

void ApplyRotate180(bool enabled, bool persist) {
    g_rot180 = enabled;
    Camera* camera = Board::GetInstance().GetCamera();
    if (camera != nullptr) {
        camera->SetRotate180(enabled);
    }
    if (persist) {
        Settings settings(kNvsNamespace, true);
        settings.SetBool(kNvsRotateKey, enabled);
    }
    ESP_LOGI(TAG, "rotate180 = %d", (int)enabled);
}

bool LoadMirrorHorizontal() {
    Settings settings(kNvsNamespace, false);
    return settings.GetBool(kNvsMirrorKey, true);  // 默认左右镜像
}

void ApplyMirrorHorizontal(bool enabled, bool persist) {
    g_mirror_h = enabled;
    Camera* camera = Board::GetInstance().GetCamera();
    if (camera != nullptr) {
        camera->SetMirrorHorizontal(enabled);
    }
    if (persist) {
        Settings settings(kNvsNamespace, true);
        settings.SetBool(kNvsMirrorKey, enabled);
    }
    ESP_LOGI(TAG, "mirror_h = %d", (int)enabled);
}

// ---------------------------------------------------------------------------
// 摄像头取帧（与云端拍照共用一把锁，避免 frame_ 被并发改写）
// ---------------------------------------------------------------------------

bool CaptureJpeg(std::vector<uint8_t>& out) {
    out.clear();
    Camera* camera = Board::GetInstance().GetCamera();
    if (camera == nullptr) {
        return false;
    }
    if (xSemaphoreTake(g_cam_lock, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGW(TAG, "camera busy, skip frame");
        return false;
    }
    bool ok = camera->GetJpegFrame(out);
    xSemaphoreGive(g_cam_lock);
    if (!ok || out.empty() || out.size() > kMaxJpegBytes) {
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// 控制端口 8080
// ---------------------------------------------------------------------------

esp_err_t HandleMove(httpd_req_t* req) {
    NoteClientActivity();
    float x = 0.0f;
    float y = 0.0f;
    bool has_x = QueryFloat(req, "x", &x);
    bool has_y = QueryFloat(req, "y", &y);
    if (!has_x && !has_y) {
        AddCommonHeaders(req);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "need x and/or y");
    }
    if (!has_x) x = g_speed_x;
    if (!has_y) y = g_speed_y;
    SendMotion(x, y);
    return ReplyText(req, "ok");
}

esp_err_t HandleStop(httpd_req_t* req) {
    NoteClientActivity();
    SendMotion(0.0f, 0.0f);
    return ReplyText(req, "stopped");
}

esp_err_t HandleCmd(httpd_req_t* req) {
    NoteClientActivity();
    char cmd[32];
    if (!QueryValue(req, "c", cmd, sizeof(cmd))) {
        AddCommonHeaders(req);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "need c");
    }
    SendRaw(cmd);
    return ReplyText(req, "ok");
}

esp_err_t HandleRot(httpd_req_t* req) {
    NoteClientActivity();
    char raw[8];
    bool on = !g_rot180;  // 不带参数就切换
    if (QueryValue(req, "on", raw, sizeof(raw))) {
        on = (raw[0] == '1' || raw[0] == 't' || raw[0] == 'T' || raw[0] == 'y');
    }
    ApplyRotate180(on, true);
    return ReplyJson(req, g_rot180 ? "{\"rot\":true}" : "{\"rot\":false}");
}

esp_err_t HandleMirror(httpd_req_t* req) {
    NoteClientActivity();
    char raw[8];
    bool on = !g_mirror_h;  // 不带参数就切换
    if (QueryValue(req, "on", raw, sizeof(raw))) {
        on = (raw[0] == '1' || raw[0] == 't' || raw[0] == 'T' || raw[0] == 'y');
    }
    ApplyMirrorHorizontal(on, true);
    return ReplyJson(req, g_mirror_h ? "{\"mirror\":true}" : "{\"mirror\":false}");
}

esp_err_t HandleStatus(httpd_req_t* req) {
    NoteClientActivity();
    char body[480];
    char flush[64] = "";
    char battery[64] = "";
    char chassis[80] = "";
    if (g_battery) {
        int cell_mv = 0;
        int percent = 0;
        bool charging = false;
        if (g_battery(cell_mv, percent, charging)) {
            snprintf(battery, sizeof(battery), ",\"batt\":%d,\"batt_mv\":%d,\"charging\":%s",
                     percent, cell_mv, charging ? "true" : "false");
        }
    }
    if (g_chassis_battery) {
        int cell_mv = 0;
        int percent = 0;
        bool charging = false;
        if (g_chassis_battery(cell_mv, percent, charging)) {
            snprintf(chassis, sizeof(chassis),
                     ",\"chassis_batt\":%d,\"chassis_batt_mv\":%d,\"chassis_charging\":%s",
                     percent, cell_mv, charging ? "true" : "false");
        }
    }
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
    // 表情动画走的是"帧被推上面板"这条路，计数在长就跑说明渲染没卡住。
    int64_t since_us = emote::GetMicrosecondsSinceLastFlush();
    snprintf(flush, sizeof(flush), ",\"flush\":%lu,\"last_flush_ms\":%lld",
             static_cast<unsigned long>(emote::GetFlushCount()),
             static_cast<long long>(since_us < 0 ? -1 : since_us / 1000));
#endif
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"ip\":\"%s\",\"x\":%.2f,\"y\":%.2f,\"uptime_s\":%lld,"
             "\"camera\":%s,\"rot\":%s,\"mirror\":%s,\"recording\":%s,\"streams\":%d,"
             "\"ps\":\"%s\"%s%s%s}",
             g_ip, g_speed_x, g_speed_y,
             (long long)((esp_timer_get_time() - g_boot_us) / 1000000),
             Board::GetInstance().GetCamera() != nullptr ? "true" : "false",
             g_rot180 ? "true" : "false", g_mirror_h ? "true" : "false",
             (Board::GetInstance().GetCamera() != nullptr &&
              Board::GetInstance().GetCamera()->IsRecordingMode())
                 ? "true"
                 : "false",
             g_stream_clients,
             g_ps_active ? "none" : "min_modem",
             flush, battery, chassis);
    return ReplyJson(req, body);
}

// 网络自检：关联的 AP + 省电档位 + lwIP 的 ARP 表。
//
// 「别的设备都能 ping 通，就机器人不通」这种问题，两边各测各的会互相矛盾：
// 有 ARP 缓存的一方（Mac）一直"正常"，没缓存的一方（手机）永远失败。把机器人
// 自己的 ARP 表吐出来，才能分清是「请求根本没收到」还是「收到了没回包」。
//
// 特别值钱的是 ARP_TABLE_SIZE 只有 10：lwIP 在表满时收到「不在表里的主机」发来的
// ARP 请求，会先 update 失败然后**直接 return，不回包**，没有任何日志——症状和
// 「网络不通」一模一样。
esp_err_t HandleNet(httpd_req_t* req) {
    NoteClientActivity();
    char body[1024];
    int n = 0;

    char ssid[33] = "";
    char bssid[24] = "";
    int rssi = 0;
    int channel = 0;
    wifi_ap_record_t ap = {};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        memcpy(ssid, ap.ssid, sizeof(ap.ssid));
        ssid[32] = '\0';
        snprintf(bssid, sizeof(bssid), "%02x:%02x:%02x:%02x:%02x:%02x", ap.bssid[0], ap.bssid[1],
                 ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]);
        rssi = ap.rssi;
        channel = ap.primary;
    }

    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);

    n += snprintf(body + n, sizeof(body) - n,
                  "{\"ssid\":\"%s\",\"bssid\":\"%s\",\"rssi\":%d,\"ch\":%d,\"ps\":%d,"
                  "\"usb\":%d,\"idle_s\":%ld,\"want_low\":%d,"
                  "\"arp_size\":%d,\"arp\":[",
                  ssid, bssid, rssi, channel, static_cast<int>(ps), (int)g_usb_present,
                  (long)g_idle_sec, (int)g_want_low_power, ARP_TABLE_SIZE);

    bool first = true;
    for (size_t i = 0; i < ARP_TABLE_SIZE && n < static_cast<int>(sizeof(body)) - 96; i++) {
        ip4_addr_t* ip = nullptr;
        struct netif* netif_of_entry = nullptr;
        struct eth_addr* eth = nullptr;
        if (!etharp_get_entry(i, &ip, &netif_of_entry, &eth) || ip == nullptr || eth == nullptr) {
            continue;
        }
        const uint8_t* o = reinterpret_cast<const uint8_t*>(eth);
        n += snprintf(body + n, sizeof(body) - n,
                      "%s{\"ip\":\"" IPSTR "\",\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\"}",
                      first ? "" : ",", IP2STR(ip), o[0], o[1], o[2], o[3], o[4], o[5]);
        first = false;
    }
    snprintf(body + n, sizeof(body) - n, "]}");
    return ReplyJson(req, body);
}

// ---------------------------------------------------------------------------
// 任务 CPU 占用：量「IMU 采样到底吃多少 CPU、会不会挤到语音唤醒」
// ---------------------------------------------------------------------------
// sdkconfig 里 CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y，每个任务都有
// ulRunTimeCounter（32 位微秒，约 71 分钟回绕）。回绕不影响无符号差分，
// 所以这里只算「距上一次 /tasks 请求这段时间」的占比，不做长期平均——
// 想量某个任务，连着请求两次即可，间隔就是统计窗口。
//
// 输出 cpu 是千分比（0~1000），栈是历史最小剩余（字，×4 得字节）。
// idle0 / idle1 的占比直接反映还有多少余量：两个加起来接近 1000 就说明
// 整机大部分时间在空转，任何后台任务都没挤到关键路径。
constexpr int kMaxTaskSnapshots = 48;

struct TaskCpuSnapshot {
    char name[16];
    uint32_t counter;
    uint32_t stack_words;
};

TaskCpuSnapshot g_task_prev[kMaxTaskSnapshots];
int g_task_prev_count = 0;
int64_t g_task_prev_us = 0;

const TaskCpuSnapshot* FindPrevSnapshot(const char* name) {
    for (int i = 0; i < g_task_prev_count; i++) {
        if (strncmp(g_task_prev[i].name, name, sizeof(g_task_prev[i].name)) == 0) {
            return &g_task_prev[i];
        }
    }
    return nullptr;
}

esp_err_t HandleTasks(httpd_req_t* req) {
    NoteClientActivity();
    const UBaseType_t need = uxTaskGetNumberOfTasks() + 4;
    TaskStatus_t* list = static_cast<TaskStatus_t*>(malloc(sizeof(TaskStatus_t) * need));
    if (list == nullptr) {
        return ReplyJson(req, "{\"ok\":false,\"err\":\"oom\"}");
    }
    const UBaseType_t count = uxTaskGetSystemState(list, need, nullptr);
    const int64_t now = esp_timer_get_time();
    const int64_t window_us = g_task_prev_us == 0 ? 0 : now - g_task_prev_us;

    // 第一遍算窗口内的总运行时间，第二遍才能换算成千分比
    uint64_t total = 0;
    for (UBaseType_t i = 0; i < count; i++) {
        const TaskCpuSnapshot* prev = FindPrevSnapshot(list[i].pcTaskName);
        if (prev != nullptr) {
            total += static_cast<uint32_t>(list[i].ulRunTimeCounter - prev->counter);
        }
    }

    static char body[2560];
    int n = snprintf(body, sizeof(body), "{\"window_ms\":%ld,\"tasks\":[",
                     static_cast<long>(window_us / 1000));
    bool first = true;
    for (UBaseType_t i = 0; i < count && n < static_cast<int>(sizeof(body)) - 96; i++) {
        const TaskCpuSnapshot* prev = FindPrevSnapshot(list[i].pcTaskName);
        const uint32_t delta =
            prev == nullptr ? 0 : static_cast<uint32_t>(list[i].ulRunTimeCounter - prev->counter);
        const uint32_t permille =
            (total == 0) ? 0 : static_cast<uint32_t>(delta * 1000ULL / total);
        n += snprintf(body + n, sizeof(body) - n,
                      "%s{\"n\":\"%s\",\"cpu\":%lu,\"stk\":%lu,\"pri\":%u}", first ? "" : ",",
                      list[i].pcTaskName, static_cast<unsigned long>(permille),
                      static_cast<unsigned long>(list[i].usStackHighWaterMark),
                      list[i].uxBasePriority);
        first = false;
    }
    snprintf(body + n, sizeof(body) - n, "]}");

    // 保存本次快照，供下次差分
    g_task_prev_count = 0;
    for (UBaseType_t i = 0; i < count && g_task_prev_count < kMaxTaskSnapshots; i++) {
        TaskCpuSnapshot& s = g_task_prev[g_task_prev_count++];
        snprintf(s.name, sizeof(s.name), "%s", list[i].pcTaskName);
        s.counter = list[i].ulRunTimeCounter;
        s.stack_words = list[i].usStackHighWaterMark;
    }
    g_task_prev_us = now;

    free(list);
    return ReplyJson(req, body);
}

// CAMERA_MODE_SWITCH: 录制/拍照两种取图模式。录制模式换 320x240 YUV422
// （约 9fps，给 NAS 录像用），拍照模式回 640x480 RGB565（给大模型看图）。
// 切换要停流重建缓冲，所以有客户端正在拉流时直接拒绝，让调用方先断开。
esp_err_t HandleRecord(httpd_req_t* req) {
    NoteClientActivity();
    Camera* camera = Board::GetInstance().GetCamera();
    if (camera == nullptr) {
        return ReplyJson(req, "{\"ok\":false,\"err\":\"no camera\"}");
    }
    float on = 1.0f;
    QueryFloat(req, "on", &on);
    if (g_stream_clients > 0) {
        return ReplyJson(req, "{\"ok\":false,\"err\":\"stream active, stop it first\"}");
    }
    if (!camera->SetRecordingMode(on != 0.0f)) {
        return ReplyJson(req, "{\"ok\":false,\"err\":\"switch failed\"}");
    }
    char body[96];
    snprintf(body, sizeof(body), "{\"ok\":true,\"recording\":%s}", on != 0.0f ? "true" : "false");
    return ReplyJson(req, body);
}

// 体感游戏和标定脚本读这个。JSON 由驱动拼，这里只负责转发，
// 免得 IMU 字段一改就得同步两个文件。
esp_err_t HandleImu(httpd_req_t* req) {
    NoteClientActivity();
    if (!g_imu) {
        return ReplyJson(req, "{\"ready\":false,\"err\":\"imu disabled\"}");
    }
    const std::string body = g_imu();
    if (body.empty()) {
        return ReplyJson(req, "{\"ready\":false,\"err\":\"no sample\"}");
    }
    return ReplyJson(req, body.c_str());
}

esp_err_t HandleImuZero(httpd_req_t* req) {
    NoteClientActivity();
    if (!g_imu_zero) {
        return ReplyJson(req, "{\"ok\":false,\"err\":\"imu disabled\"}");
    }
    g_imu_zero();
    return ReplyJson(req, "{\"ok\":true}");
}

// 最小可用控制台：方向键 / WASD、速度档位、实时画面
const char* kIndexHtml = R"HTML(<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>SparkBot 遥控</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;font-family:-apple-system,"PingFang SC",sans-serif;background:#14161a;color:#e8eaed;
display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:16px}
h1{font-size:15px;font-weight:600;margin:0 0 4px;letter-spacing:.5px}
#stat{font-size:12px;color:#8a9099;margin-bottom:12px;font-variant-numeric:tabular-nums}
#view{width:min(92vw,340px);aspect-ratio:1;background:#000;border-radius:14px;object-fit:cover;
border:1px solid #262a31}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;width:min(92vw,340px);margin-top:16px}
.pad button{aspect-ratio:1.15;border:1px solid #2c313a;background:#1c2027;color:#e8eaed;
border-radius:12px;font-size:20px;transition:.08s;cursor:pointer}
.pad button:active,.pad button.on{background:#3b6cf0;border-color:#3b6cf0}
.pad .empty{visibility:hidden}
.row{display:flex;gap:8px;width:min(92vw,340px);margin-top:10px}
.row button{flex:1;padding:13px;border-radius:12px;border:1px solid #2c313a;background:#1c2027;
color:#e8eaed;font-size:14px;cursor:pointer}
.row button.stop{background:#4a1f24;border-color:#6b2a31;color:#ffb4b4;font-weight:600}
.speed{display:flex;align-items:center;gap:10px;width:min(92vw,340px);margin-top:14px;font-size:12px;color:#8a9099}
input[type=range]{flex:1;accent-color:#3b6cf0}
.wide{width:min(92vw,340px);margin-top:12px;padding:11px;border-radius:12px;border:1px solid #2c313a;
background:#1c2027;color:#8a9099;font-size:12.5px;cursor:pointer}
</style></head><body>
<h1>ESP-SparkBot 局域网遥控</h1>
<div id="stat">连接中…</div>
<img id="view" alt="camera">
<div class="pad">
  <span class="empty"></span><button data-mv="0,1">▲</button><span class="empty"></span>
  <button data-mv="-1,0">◀</button><span class="empty"></span><button data-mv="1,0">▶</button>
  <span class="empty"></span><button data-mv="0,-1">▼</button><span class="empty"></span>
</div>
<div class="row">
  <button id="dance">跳舞</button>
  <button id="light">灯光</button>
  <button class="stop" id="stop">急停</button>
</div>
<div class="speed"><span>速度</span><input type="range" id="spd" min="10" max="100" value="100"><span id="spdv">100%</span></div>
<button class="wide" id="rot">画面方向：已转正 180°</button>
<button class="wide" id="mir">左右镜像：已开启</button>
<script>
const H=location.hostname, C=8080, V=8081;
const stat=document.getElementById('stat'), view=document.getElementById('view');
const spd=document.getElementById('spd'), spdv=document.getElementById('spdv');
const rotBtn=document.getElementById('rot'), mirBtn=document.getElementById('mir');
let speed=1, rot=true, mirror=true;
spd.oninput=()=>{speed=spd.value/100; spdv.textContent=spd.value+'%';};

// 传感器倒装，固件默认把画面转 180°；这里可以现场切回原始方向对比
function paintRot(){rotBtn.textContent='画面方向：'+(rot?'已转正 180°':'原始（未旋转）');}
// 上下正了但左右反，用独立开关再镜像一次；和 180° 叠加等于只做垂直翻转
function paintMir(){mirBtn.textContent='左右镜像：'+(mirror?'已开启':'已关闭');}
function restartStream(){view.src='http://'+H+':'+V+'/stream?t='+Date.now();}
paintRot(); paintMir();
rotBtn.onclick=async()=>{
  try{
    const r=await fetch('http://'+H+':'+C+'/rot?on='+(rot?0:1),{cache:'no-store'});
    const j=await r.json();
    rot=!!j.rot; paintRot(); restartStream();
  }catch(e){}
};
mirBtn.onclick=async()=>{
  try{
    const r=await fetch('http://'+H+':'+C+'/mirror?on='+(mirror?0:1),{cache:'no-store'});
    const j=await r.json();
    mirror=!!j.mirror; paintMir(); restartStream();
  }catch(e){}
};

// 图像流走独立端口，断线后自动重连
let tries=0;
view.onerror=()=>{tries++; stat.textContent='图像未就绪，重试中 ('+tries+')';
  setTimeout(restartStream,1200);};
view.onload=()=>{tries=0;};
restartStream();

function cmd(p){return fetch('http://'+H+':'+C+p,{cache:'no-store'}).catch(()=>{});}
function move(x,y){cmd('/move?x='+(x*speed).toFixed(2)+'&y='+(y*speed).toFixed(2));}

// 底盘 500ms 收不到 x/y 指令就自动停车，所以按住期间必须周期重发，
// 否则按一次只会走半秒。
const REPEAT_MS=200;
let holdTimer=null;
function holdStart(x,y){
  holdStop(false);
  move(x,y);
  holdTimer=setInterval(()=>move(x,y),REPEAT_MS);
}
function holdStop(withStop){
  if(holdTimer){clearInterval(holdTimer); holdTimer=null;}
  if(withStop) cmd('/stop');
}

document.querySelectorAll('[data-mv]').forEach(b=>{
  const [x,y]=b.dataset.mv.split(',').map(Number);
  const end=()=>{
    if(!b.classList.contains('on')) return;
    b.classList.remove('on'); holdStop(true);
  };
  b.addEventListener('pointerdown',e=>{e.preventDefault(); b.classList.add('on'); holdStart(x,y);});
  b.addEventListener('pointerup',end);
  b.addEventListener('pointercancel',end);
  b.addEventListener('pointerleave',end);
});
document.getElementById('stop').onclick=()=>holdStop(true);
document.getElementById('dance').onclick=()=>cmd('/cmd?c=d1');
document.getElementById('light').onclick=()=>cmd('/cmd?c=w2');

// 底盘 left=y+x / right=y-x：y 为正=前进，x 为正=右转
const keys={KeyW:[0,1],KeyS:[0,-1],KeyA:[-1,0],KeyD:[1,0],
  ArrowUp:[0,1],ArrowDown:[0,-1],ArrowLeft:[-1,0],ArrowRight:[1,0]};
const held=new Set();
addEventListener('keydown',e=>{const k=keys[e.code]; if(!k||held.has(e.code))return;
  e.preventDefault(); held.add(e.code); holdStart(k[0],k[1]);});
addEventListener('keyup',e=>{if(keys[e.code]&&held.delete(e.code)) holdStop(true);});

async function poll(){
  try{const r=await fetch('http://'+H+':'+C+'/status',{cache:'no-store'});
    const j=await r.json();
    stat.textContent=j.ip+' · 速度 '+Math.round(speed*100)+'% · 在线 '+
      (j.uptime_s/60).toFixed(0)+' 分钟'+
      (typeof j.batt==='number' ? ' · 本体 '+j.batt+'%'+(j.charging?'⚡':'') : '')+
      (typeof j.chassis_batt==='number' ? ' · 底盘 '+j.chassis_batt+'%'+(j.chassis_charging?'⚡':'') : '');
    if(typeof j.rot==='boolean'&&j.rot!==rot){rot=j.rot; paintRot();}
    if(typeof j.mirror==='boolean'&&j.mirror!==mirror){mirror=j.mirror; paintMir();}
  }catch(e){stat.textContent='设备离线';}
  setTimeout(poll,2000);
}
poll();
</script></body></html>)HTML";

esp_err_t HandleIndex(httpd_req_t* req) {
    NoteClientActivity();
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
// 图像端口 8081
// ---------------------------------------------------------------------------

esp_err_t HandleSnapshot(httpd_req_t* req) {
    NoteClientActivity();
    std::vector<uint8_t> jpeg;
    if (!CaptureJpeg(jpeg)) {
        AddCommonHeaders(req);
        httpd_resp_set_status(req, "503 Service Unavailable");
        return httpd_resp_sendstr(req, "camera not ready");
    }
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "image/jpeg");
    return httpd_resp_send(req, (const char*)jpeg.data(), jpeg.size());
}

esp_err_t HandleStream(httpd_req_t* req) {
    static const char* kBoundary = "sparkbotframe";
    NoteClientActivity();
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=sparkbotframe");
    httpd_resp_set_hdr(req, "X-Accel-Buffering", "no");
    g_stream_clients++;

    int failed = 0;
    for (;;) {
        NoteClientActivity();  // 长连接期间持续续期，别看着看着掉回省电
        std::vector<uint8_t> jpeg;
        if (!CaptureJpeg(jpeg)) {
            // 连续失败说明摄像头异常或客户端已断，主动收尾，别把槽位一直占着
            if (++failed > 50) {
                ESP_LOGW(TAG, "camera unresponsive, closing stream");
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        failed = 0;
        char part[128];
        int n = snprintf(part, sizeof(part),
                         "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                         kBoundary, (unsigned)jpeg.size());
        if (httpd_resp_send_chunk(req, part, n) != ESP_OK) {
            break;  // 客户端断开
        }
        if (httpd_resp_send_chunk(req, (const char*)jpeg.data(), jpeg.size()) != ESP_OK) {
            break;
        }
        // 约 4~5 fps，兼顾流畅度与 A/V 主链路的 CPU 余量
        // CAMERA_PERF_PROBE: 60ms 固定节流是实测帧率的主要天花板，先去掉看纯上限
        vTaskDelay(pdMS_TO_TICKS(0));
    }
    g_stream_clients--;
    ESP_LOGI(TAG, "stream client disconnected, active=%d", g_stream_clients);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// 启动流程
// ---------------------------------------------------------------------------

bool RegisterRoute(httpd_handle_t server, const char* uri, httpd_method_t method,
                    esp_err_t (*handler)(httpd_req_t*)) {
    // 逐字段赋值而非指定初始化器，规避不同 IDF 版本 httpd_uri_t 字段顺序差异
    httpd_uri_t route = {};
    route.uri = uri;
    route.method = method;
    route.handler = handler;
    route.user_ctx = nullptr;
    const esp_err_t err = httpd_register_uri_handler(server, &route);
    if (err != ESP_OK) {
        // 别吞掉：handler 数量超上限、URI 重复都会走到这里，而对外表现都只是 404
        ESP_LOGE("local_ctrl", "注册路由 %s 失败：%s", uri, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

bool StartControlServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kCtrlPort;
    config.ctrl_port = kCtrlPort + 100;  // 内部事件用，避免与图传实例撞车
    config.stack_size = 6144;
    // App 每 3 秒轮询一次 /status，远程隧道还会额外占用几条连接；
    // 默认 4 个槽位在「隧道 + 本地」并存时会被占满，新连接直接被拒（表现为设备离线）。
    config.max_open_sockets = 6;
    config.lru_purge_enable = true;
    // 默认只给 8 个 URI handler，而下面一共注册 9 个。第 9 个会静默注册失败
    // ——HTTP 直接 404，没有任何日志，看起来就像「这个端点没实现」。
    // /imu/zero 就是这么丢的。以后再加端点也不用回来改这里。
    config.max_uri_handlers = 16;
    // httpd 只有一条任务，串行跑所有 handler：任何一个连接只要请求头没收齐，
    // 任务就阻塞在这个 socket 的 recv 上，期间**所有端点**（含 20Hz 的 /imu）
    // 全在排队。默认 5 秒，弱网下一次丢包足以让姿态流冻住一秒以上——这正是
    // App 里"偶尔卡顿 + 时延突然跳到 1000ms"的来源。压到 1 秒，客户端断连
    // 也会立刻把 recv 唤醒，最坏阻塞时间不再由默认值兜底。
    config.recv_wait_timeout = 1;
    config.send_wait_timeout = 1;

    if (httpd_start(&g_ctrl_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "control server start failed");
        return false;
    }

    RegisterRoute(g_ctrl_server, "/", HTTP_GET, HandleIndex);
    RegisterRoute(g_ctrl_server, "/move", HTTP_GET, HandleMove);
    RegisterRoute(g_ctrl_server, "/stop", HTTP_GET, HandleStop);
    RegisterRoute(g_ctrl_server, "/cmd", HTTP_GET, HandleCmd);
    RegisterRoute(g_ctrl_server, "/rot", HTTP_GET, HandleRot);
    RegisterRoute(g_ctrl_server, "/mirror", HTTP_GET, HandleMirror);
    RegisterRoute(g_ctrl_server, "/status", HTTP_GET, HandleStatus);
    RegisterRoute(g_ctrl_server, "/net", HTTP_GET, HandleNet);
    RegisterRoute(g_ctrl_server, "/tasks", HTTP_GET, HandleTasks);
    RegisterRoute(g_ctrl_server, "/record", HTTP_GET, HandleRecord);
    // 只有板级真的注入了 IMU 才挂这两个路由，没注入的板子上访问会 404，
    // 不会返回一个误导性的 ready:false。
    if (g_imu) {
        RegisterRoute(g_ctrl_server, "/imu", HTTP_GET, HandleImu);
    }
    if (g_imu_zero) {
        RegisterRoute(g_ctrl_server, "/imu/zero", HTTP_GET, HandleImuZero);
    }

    ESP_LOGI(TAG, "control server on :%d", kCtrlPort);
    return true;
}

bool StartCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kCamPort;
    config.ctrl_port = kCamPort + 100;
    config.stack_size = 10240;  // 抓帧 + JPEG 编码调用较深
    // 图传是长连接：一条被隧道占着、一条本地在看，很快就没槽位了。
    config.max_open_sockets = 5;
    config.lru_purge_enable = true;

    if (httpd_start(&g_cam_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "camera server start failed");
        return false;
    }

    RegisterRoute(g_cam_server, "/stream", HTTP_GET, HandleStream);
    RegisterRoute(g_cam_server, "/snapshot", HTTP_GET, HandleSnapshot);

    ESP_LOGI(TAG, "camera server on :%d", kCamPort);
    return true;
}

// 网络就绪前启动 httpd 会绑到空接口，所以单独轮询等待 DHCP
void BootstrapTask(void*) {
    esp_netif_ip_info_t ip_info = {};
    bool ready = false;

    for (int i = 0; i < 240; ++i) {  // 最多等 120s
        // 句柄必须在每轮重新获取：本任务在板级构造期就创建了，那时
        // WifiManager 还没初始化网络接口，提前取一次会拿到 nullptr，
        // 之后循环里一直用空值，永远等不到 IP。
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK &&
            ip_info.ip.addr != 0) {
            ready = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!ready) {
        ESP_LOGE(TAG, "no IP after 120s, local control service skipped");
        vTaskDelete(nullptr);
        return;
    }

    static char ip_buf[16];
    snprintf(ip_buf, sizeof(ip_buf), IPSTR, IP2STR(&ip_info.ip));
    g_ip = ip_buf;
    ESP_LOGI(TAG, "device ip = %s", g_ip);

    // 摄像头此时已初始化，可以把上次保存的画面方向应用上去
    ApplyRotate180(LoadRotate180(), false);
    ApplyMirrorHorizontal(LoadMirrorHorizontal(), false);

    StartControlServer();
    StartCameraServer();
    // 服务能收请求了才开始看有没有人用；Wi-Fi 早已连上，set_ps 不会报未初始化
    xTaskCreate(PowerModeTask, "wifi_ps_ctl", 3072, nullptr, 2, nullptr);
    ESP_LOGI(TAG, "local control ready: http://%s:%d/", g_ip, kCtrlPort);

    vTaskDelete(nullptr);
}

}  // namespace

// 上层（application.cc）的省电意愿转到这里登记，板级不再自己调 esp_wifi_set_ps。
// 真正的档位由 PowerModeTask 统一裁决——USB 供电、有没有客户端、上层意愿三条一起看。
// 分成两处各调一次会互相覆盖：之前板级硬压 PERFORMANCE，就把「纯电池静置 2 分钟
// 回省电」这条彻底压死了（永远 NONE，待机电流下不来）。
void LocalControlNotePowerSaveWish(bool low_power) {
    if (g_want_low_power == low_power) {
        return;
    }
    g_want_low_power = low_power;
    ESP_LOGI(TAG, "app power save wish: %s", low_power ? "low" : "performance");
}

// 板级要判断「现在是不是低功耗」，所以把状态开放出去。
// g_ps_active 只在省电与否切换时改写，读一个 bool 不会撕裂；读到旧值最多让
// 睡脸晚 500ms 出现或消失，无所谓。
bool LocalControlPowerSaveOn() {
    return !g_ps_active;
}

void LocalControlStart(std::function<void(const char*)> send_command,
                       BatteryProvider battery_provider,
                       BatteryProvider chassis_battery_provider,
                       ImuProvider imu_provider,
                       ImuZeroAction imu_zero) {
    if (g_send) {
        ESP_LOGW(TAG, "already started");
        return;
    }
    g_send = std::move(send_command);
    g_battery = std::move(battery_provider);
    g_chassis_battery = std::move(chassis_battery_provider);
    g_imu = std::move(imu_provider);
    g_imu_zero = std::move(imu_zero);
    g_boot_us = esp_timer_get_time();
    g_cam_lock = xSemaphoreCreateMutex();
    if (g_cam_lock == nullptr) {
        ESP_LOGE(TAG, "failed to create camera mutex");
        return;
    }
    xTaskCreate(BootstrapTask, "local_ctrl_boot", 4096, nullptr, 3, nullptr);
}
