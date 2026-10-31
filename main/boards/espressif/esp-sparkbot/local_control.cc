#include "local_control.h"

#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

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
bool g_rot180 = true;

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
    SendMotion(0.0f, 0.0f);
    return ReplyText(req, "stopped");
}

esp_err_t HandleCmd(httpd_req_t* req) {
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
    char raw[8];
    bool on = !g_rot180;  // 不带参数就切换
    if (QueryValue(req, "on", raw, sizeof(raw))) {
        on = (raw[0] == '1' || raw[0] == 't' || raw[0] == 'T' || raw[0] == 'y');
    }
    ApplyRotate180(on, true);
    return ReplyJson(req, g_rot180 ? "{\"rot\":true}" : "{\"rot\":false}");
}

esp_err_t HandleStatus(httpd_req_t* req) {
    char body[384];
    char flush[64] = "";
    char battery[64] = "";
    if (g_battery) {
        int cell_mv = 0;
        int percent = 0;
        bool charging = false;
        if (g_battery(cell_mv, percent, charging)) {
            snprintf(battery, sizeof(battery), ",\"batt\":%d,\"batt_mv\":%d,\"charging\":%s",
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
             "\"camera\":%s,\"rot\":%s%s%s}",
             g_ip, g_speed_x, g_speed_y,
             (long long)((esp_timer_get_time() - g_boot_us) / 1000000),
             Board::GetInstance().GetCamera() != nullptr ? "true" : "false",
             g_rot180 ? "true" : "false", flush, battery);
    return ReplyJson(req, body);
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
<script>
const H=location.hostname, C=8080, V=8081;
const stat=document.getElementById('stat'), view=document.getElementById('view');
const spd=document.getElementById('spd'), spdv=document.getElementById('spdv');
const rotBtn=document.getElementById('rot');
let speed=1, rot=true;
spd.oninput=()=>{speed=spd.value/100; spdv.textContent=spd.value+'%';};

// 传感器倒装，固件默认把画面转 180°；这里可以现场切回原始方向对比
function paintRot(){rotBtn.textContent='画面方向：'+(rot?'已转正 180°':'原始（未旋转）');}
function restartStream(){view.src='http://'+H+':'+V+'/stream?t='+Date.now();}
paintRot();
rotBtn.onclick=async()=>{
  try{
    const r=await fetch('http://'+H+':'+C+'/rot?on='+(rot?0:1),{cache:'no-store'});
    const j=await r.json();
    rot=!!j.rot; paintRot(); restartStream();
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
      (typeof j.batt==='number' ? ' · 电量 '+j.batt+'%'+(j.charging?'（充电中）':'') : '');
    if(typeof j.rot==='boolean'&&j.rot!==rot){rot=j.rot; paintRot();}
  }catch(e){stat.textContent='设备离线';}
  setTimeout(poll,2000);
}
poll();
</script></body></html>)HTML";

esp_err_t HandleIndex(httpd_req_t* req) {
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

// ---------------------------------------------------------------------------
// 图像端口 8081
// ---------------------------------------------------------------------------

esp_err_t HandleSnapshot(httpd_req_t* req) {
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
    AddCommonHeaders(req);
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=sparkbotframe");
    httpd_resp_set_hdr(req, "X-Accel-Buffering", "no");

    for (;;) {
        std::vector<uint8_t> jpeg;
        if (!CaptureJpeg(jpeg)) {
            vTaskDelay(pdMS_TO_TICKS(300));
            // 连续失败说明摄像头异常或客户端已断，主动收尾
            continue;
        }
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
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    ESP_LOGI(TAG, "stream client disconnected");
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
    return httpd_register_uri_handler(server, &route) == ESP_OK;
}

bool StartControlServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kCtrlPort;
    config.ctrl_port = kCtrlPort + 100;  // 内部事件用，避免与图传实例撞车
    config.stack_size = 6144;
    config.max_open_sockets = 4;
    config.lru_purge_enable = true;

    if (httpd_start(&g_ctrl_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "control server start failed");
        return false;
    }

    RegisterRoute(g_ctrl_server, "/", HTTP_GET, HandleIndex);
    RegisterRoute(g_ctrl_server, "/move", HTTP_GET, HandleMove);
    RegisterRoute(g_ctrl_server, "/stop", HTTP_GET, HandleStop);
    RegisterRoute(g_ctrl_server, "/cmd", HTTP_GET, HandleCmd);
    RegisterRoute(g_ctrl_server, "/rot", HTTP_GET, HandleRot);
    RegisterRoute(g_ctrl_server, "/status", HTTP_GET, HandleStatus);

    ESP_LOGI(TAG, "control server on :%d", kCtrlPort);
    return true;
}

bool StartCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = kCamPort;
    config.ctrl_port = kCamPort + 100;
    config.stack_size = 10240;  // 抓帧 + JPEG 编码调用较深
    config.max_open_sockets = 3;
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

    StartControlServer();
    StartCameraServer();
    ESP_LOGI(TAG, "local control ready: http://%s:%d/", g_ip, kCtrlPort);

    vTaskDelete(nullptr);
}

}  // namespace

void LocalControlStart(std::function<void(const char*)> send_command,
                       BatteryProvider battery_provider) {
    if (g_send) {
        ESP_LOGW(TAG, "already started");
        return;
    }
    g_send = std::move(send_command);
    g_battery = std::move(battery_provider);
    g_boot_us = esp_timer_get_time();
    g_cam_lock = xSemaphoreCreateMutex();
    if (g_cam_lock == nullptr) {
        ESP_LOGE(TAG, "failed to create camera mutex");
        return;
    }
    xTaskCreate(BootstrapTask, "local_ctrl_boot", 4096, nullptr, 3, nullptr);
}
