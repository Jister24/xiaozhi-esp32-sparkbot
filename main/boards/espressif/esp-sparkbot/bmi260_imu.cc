#include "bmi260_imu.h"

#include "bmi260_init_data.h"

#include <esp_log.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstdio>
#include <cstring>

// ===========================================================================
// 这块板子实测 chip id = 0x27，即 Bosch BMI260（不是 BMI270 的 0x24，
// 也不是官方适配过的 BMI220 的 0x26）。上游 example 的驱动写着
// `if (data != 0x24 && data != 0x26)` 直接拒绝，所以官方代码跑不了这颗。
//
// BMI260 与 BMI270 寄存器地图基本一致，但有两点关键差异：
//   1. chip id 不同（0x27 vs 0x24）
//   2. **init data 不同**，且必须上传。实测：不上传时 ACC_CONF 能写能回读、
//      PWR_CTRL 也置位成功，但加速度三个轴恒为 0 —— 现象很像「传感器坏了」，
//      其实只是没喂配置。init data 存在 bmi260_init_data.h（8192 字节）。
//
// 于是这里绕开组件的 SensorAPI（它只编译了 BMI270 变体，chip id 校验会拦）：
//   1. 软复位
//   2. 把 8KB init data 按 64 字节分段写进 INIT_DATA，完成后置 INIT_CTRL=1，
//      再轮询 INTERNAL_STATUS.message 等它变成 0x01
//   3. 配 ODR / 量程 / 使能
//   4. 从 0x0C 起连续读 12 字节拿 acc + gyr
// 这样 0x27 能直接用；若换回 0x24 的 BMI270，走组件导出的 bmi270_config_file。
//
// 上电后会自检：连续读若干帧算加速度模长，静止时应接近 1g。
// 这条日志能一眼判断这颗芯片到底能不能用。
// ===========================================================================

// 由 bmi270_sensor 组件导出的 BMI270 init data（8192 字节）。
// 只在 chip id = 0x24 时使用，避免把 260 的配置喂给 270。
extern "C" const uint8_t bmi270_config_file[];

namespace {

const char* kTag = "Imu";

// ±4g → 8192 LSB/g；±500dps → 65.536 LSB/dps
constexpr float kAccLsbPerG = 8192.0f;
constexpr float kGyrLsbPerDps = 65.536f;
constexpr float kRadToDeg = 57.29577951f;
constexpr float kDegToRad = 1.0f / kRadToDeg;

constexpr uint32_t kSamplePeriodMs = 10;  // 100Hz
constexpr int kLogEveryN = 10;            // 每 10 帧一行 = 10Hz

// —— 姿态滤波参数，对应 PX4 attitude_estimator_q 的 ATT_W_* ——
//
// 这几个数是用 firmware/tools/attitude_proto.py 在合成数据上扫出来的
// （10 个随机种子取最差），不是抄来的默认值：
//   kAccWeight 0.6 → 静止误差 0.39°，手持晃动（0.3g 线加速度）时 2.1°
//   再大（1.0，即 PX4 默认）：晃动期劣化到 3.1°；
//   再小（0.4）：晃动期只 1.4°，但冷启动要 5.8s 才收敛——
//   我们第一帧就用加速度对齐姿态，冷启动不靠增益，所以选偏小的 0.6。
constexpr float kAccWeight = 0.6f;
// 零偏限幅 ±1.2dps（PX4 ATT_BIAS_MAX 是 0.05rad/s≈2.9dps）。
// 实测这颗的零偏都在 0.7dps 以内，限太宽只会在机动时留下更大的残余。
constexpr float kGyroBiasMaxDps = 1.2f;
// 加速度门控：只有模长落在这一带才认为「此刻只受重力」。
// 手持晃动叠上线加速度，模长会跑出这个区间，那几帧的重力方向不可信。
constexpr float kAccGateLow = 0.88f;
constexpr float kAccGateHigh = 1.12f;
// 静止判定：三轴陀螺都小于这个值。手持时最慢的抖动也有 2~3dps，
// 留 1.5 就不会把「手稳住了但还在慢慢转」误判成静止。
constexpr float kStillRateDps = 1.5f;
// 连续静止这么多帧（100Hz 下 = 1 秒）才更新零偏，避免路过静止状态就抓。
constexpr int kStillFrames = 100;
// 静止时陀螺读数里剩下的那点偏置会让航向一直爬，低于这个值就不积分航向。
// 代价是极慢的转向（<0.5dps）会被吃掉——摇杆用不到，可以接受。
constexpr float kYawDeadbandDps = 0.5f;

// BMI270 / BMI260 共用同一套寄存器地图
constexpr uint8_t kRegChipId = 0x00;
constexpr uint8_t kRegAccData = 0x0C;  // 0x0C..0x11，小端 int16
constexpr uint8_t kRegGyrData = 0x12;  // 0x12..0x17
// 内部状态在 0x21，message 在低 4 位（0x01 = init ok）。
// 注意别写成 0x1A：那是 SENSORTIME_2，永远是 0，会让人误判 init 一直没成功。
constexpr uint8_t kRegInternalStatus = 0x21;
constexpr uint8_t kRegAccConf = 0x40;
constexpr uint8_t kRegAccRange = 0x41;
constexpr uint8_t kRegGyrConf = 0x42;
constexpr uint8_t kRegGyrRange = 0x43;
// init data 上传用的四个寄存器
constexpr uint8_t kRegInitCtrl = 0x59;
constexpr uint8_t kRegInitAddr0 = 0x5B;  // 字地址低 4 位
constexpr uint8_t kRegInitAddr1 = 0x5C;  // 字地址高位
constexpr uint8_t kRegInitData = 0x5E;   // 数据口，写完自动递增
// 注意这两个的顺序：PWR_CONF 在 0x7C，PWR_CTRL 在 0x7D。写反了会变成
// 「以为在使能传感器，其实在改省电配置」，数据恒为 0 且看不出任何错误。
constexpr uint8_t kRegPwrConf = 0x7C;
constexpr uint8_t kRegPwrCtrl = 0x7D;
constexpr uint8_t kRegCmd = 0x7E;

constexpr uint8_t kCmdSoftReset = 0xB6;
constexpr uint8_t kInitStatusOk = 0x01;
// 分段长度必须是偶数：init data 按 16 位字写入，地址以一「字」为单位。
constexpr uint32_t kInitBurst = 64;

// ACC_CONF: bit7=filter_perf | bit6:4=bwp(normal=0x02) | bit3:0=odr(100Hz=0x08)
constexpr uint8_t kAccConf100Hz = 0x80 | (0x02 << 4) | 0x08;  // 0xA8
// GYR_CONF: bit7=filter_perf | bit6=noise_perf | bit5:4=bwp(normal=0x02) | bit3:0=odr(100Hz=0x08)
constexpr uint8_t kGyrConf100Hz = 0x80 | 0x40 | (0x02 << 4) | 0x08;  // 0xE8
constexpr uint8_t kAccRange4G = 0x01;
constexpr uint8_t kGyrRange500Dps = 0x02;
// PWR_CTRL: temp_en | acc_en | gyr_en
constexpr uint8_t kPwrCtrlAll = 0x08 | 0x04 | 0x02;

struct I2cCtx {
    i2c_master_dev_handle_t dev = nullptr;
};

I2cCtx g_ctx;
bool g_started = false;
int g_init_err = 0;
uint8_t g_chip_id = 0;
bool g_serial_log = true;
const char* g_mode = "unknown";

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
ImuReadout g_latest;
// 零点请求由 Zero() 置位、采样任务在下一帧消费，避免跨任务共享运行中的四元数
volatile bool g_zero_request = false;

bool WriteReg(uint8_t reg, uint8_t value) {
    if (g_ctx.dev == nullptr) {
        return false;
    }
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(g_ctx.dev, buf, sizeof(buf), 50) == ESP_OK;
}

bool WriteRegs(uint8_t reg, const uint8_t* data, size_t len) {
    if (g_ctx.dev == nullptr || data == nullptr || len == 0 || len > 64) {
        return false;
    }
    uint8_t buf[65];
    buf[0] = reg;
    std::memcpy(buf + 1, data, len);
    return i2c_master_transmit(g_ctx.dev, buf, len + 1, 50) == ESP_OK;
}

bool ReadRegs(uint8_t reg, uint8_t* out, size_t len) {
    if (g_ctx.dev == nullptr || out == nullptr || len == 0) {
        return false;
    }
    return i2c_master_transmit_receive(g_ctx.dev, &reg, 1, out, len, 50) == ESP_OK;
}

uint8_t ReadReg(uint8_t reg) {
    uint8_t v = 0;
    ReadRegs(reg, &v, 1);
    return v;
}

// 一次 burst 读完 acc + gyr，保证两组数据来自同一个采样时刻。
bool ReadRaw(int16_t acc[3], int16_t gyr[3]) {
    uint8_t buf[12];
    if (!ReadRegs(kRegAccData, buf, sizeof(buf))) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        acc[i] = static_cast<int16_t>(buf[i * 2] | (buf[i * 2 + 1] << 8));
        gyr[i] = static_cast<int16_t>(buf[6 + i * 2] | (buf[7 + i * 2] << 8));
    }
    return true;
}

// 把 0x00..0x7F 整段打出来。排硬件问题时这比任何猜测都快：
//   0x00 chip id、0x02 ERR_REG、0x03 STATUS、0x18~0x1A SENSORTIME、
//   0x21 INTERNAL_STATUS、0x40~0x43 量程/ODR、0x7C PWR_CONF、0x7D PWR_CTRL。
// 逐字节读而不是一次 burst 读完：这颗芯片在长 burst 上会返回错位的重复值
// （实测 128 字节读到大量 0x80），逐字节才是可信的。
void DumpRegisters(const char* when) {
    uint8_t buf[128];
    for (int i = 0; i < 128; i++) {
        buf[i] = ReadReg(static_cast<uint8_t>(i));
        if (i == 0 && (buf[0] == 0x00 || buf[0] == 0xFF)) {
            ESP_LOGW(kTag, "%s：寄存器 dump 读不到有效 chip id", when);
            return;
        }
    }
    ESP_LOGI(kTag, "寄存器 dump（%s）", when);
    for (int row = 0; row < 8; row++) {
        char line[64];
        int p = 0;
        for (int i = 0; i < 16; i++) {
            p += std::snprintf(line + p, sizeof(line) - p, "%02X ", buf[row * 16 + i]);
        }
        ESP_LOGI(kTag, "  %02X: %s", row * 16, line);
    }
}

// 软复位后轮询 chip id。连续复位时这颗芯片偶尔会短暂不应答，只读一次会
// 误判成「芯片掉线」，白白浪费一次尝试机会。
bool WaitChipReady(uint32_t timeout_ms) {
    const uint32_t t0 = esp_log_timestamp();
    while (esp_log_timestamp() - t0 < timeout_ms) {
        const uint8_t id = ReadReg(kRegChipId);
        if (id != 0x00 && id != 0xFF) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

// 上传前的公共准备：关传感器、关省电、INIT_CTRL=0。
bool BeginUpload() {
    if (!WriteReg(kRegPwrCtrl, 0x00)) return false;
    if (!WriteReg(kRegPwrConf, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(2));  // 手册要求 ≥500us
    return WriteReg(kRegInitCtrl, 0x00);
}

// INIT_CTRL=1 之后等 init 引擎结算。message: 0x01=init_ok，0x02=init_err，
// 0x03=drv_err（固件不被这颗芯片认可），0x00=还没开始。
bool WaitInitReady(uint8_t* out_msg) {
    for (int i = 0; i < 60; i++) {  // 最多 300ms，手册说 150ms 内完成
        const uint8_t st = ReadReg(kRegInternalStatus);
        if (out_msg != nullptr) *out_msg = static_cast<uint8_t>(st & 0x0F);
        if ((st & 0x0F) == kInitStatusOk) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

// 分段上传：每 burst 字节一段，先写「字地址」（单位是 2 字节），
// 再往 INIT_DATA 连写，口地址自增。
bool UploadRanged(const uint8_t* data, uint32_t len, uint32_t burst, bool verify) {
    if (data == nullptr || len == 0 || (len % 2) != 0 || burst == 0 ||
        (burst % 2) != 0 || burst > kInitBurst) {
        return false;
    }
    if (!BeginUpload()) return false;

    for (uint32_t off = 0; off < len; off += burst) {
        const uint32_t n = (len - off < burst) ? (len - off) : burst;
        const uint32_t word_addr = off / 2;  // 手册：地址按 字节数/2 递增
        const uint8_t addr[2] = {static_cast<uint8_t>(word_addr & 0x0F),
                                 static_cast<uint8_t>((word_addr >> 4) & 0xFF)};
        if (!WriteRegs(kRegInitAddr0, addr, sizeof(addr))) {
            ESP_LOGE(kTag, "init data 写地址失败（off=%lu）",
                     static_cast<unsigned long>(off));
            return false;
        }
        if (!WriteRegs(kRegInitData, data + off, n)) {
            ESP_LOGE(kTag, "init data 写数据失败（off=%lu）",
                     static_cast<unsigned long>(off));
            return false;
        }
    }

    if (verify) {
        const uint8_t zero_addr[2] = {0, 0};
        uint8_t readback[16] = {0};
        if (WriteRegs(kRegInitAddr0, zero_addr, sizeof(zero_addr)) &&
            ReadRegs(kRegInitData, readback, sizeof(readback))) {
            char got[64];
            char want[64];
            int gp = 0;
            int wp = 0;
            for (int i = 0; i < 16 && i < static_cast<int>(len); i++) {
                gp += std::snprintf(got + gp, sizeof(got) - gp, "%02X ", readback[i]);
                wp += std::snprintf(want + wp, sizeof(want) - wp, "%02X ", data[i]);
            }
            // 一致时只在 debug 级别留痕；不一致说明镜像没落到正确位置，
            // 这时必须打出来——它是「init_err 到底是不是数据错位」的证据。
            if (std::memcmp(readback, data, 16) == 0) {
                ESP_LOGD(kTag, "init RAM 回读一致：%s", got);
            } else {
                ESP_LOGW(kTag, "init RAM 回读不一致：%s| 期望：%s", got, want);
            }
        }
    }

    return WriteReg(kRegInitCtrl, 0x01);
}

// 整包上传：手册说主机若能一次写完 8KB，就不必碰 INIT_ADDR，直接从 0x5E
// 连续灌下去。分段写在部分兼容件上会拼错镜像（表现为 init_err），
// 所以这条路径必须留。8KB 缓冲放静态区，避免撑爆任务栈。
bool UploadSingleBurst(const uint8_t* data, uint32_t len) {
    constexpr uint32_t kMax = 8192;
    static uint8_t s_buf[kMax + 1];
    if (data == nullptr || len == 0 || len > kMax) {
        return false;
    }
    if (!BeginUpload()) return false;

    s_buf[0] = kRegInitData;
    std::memcpy(s_buf + 1, data, len);
    const esp_err_t err = i2c_master_transmit(g_ctx.dev, s_buf, len + 1, 2000);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "整包写 INIT_DATA 失败：%s", esp_err_to_name(err));
        return false;
    }
    return WriteReg(kRegInitCtrl, 0x01);
}

// 复位 + 上传一次。msg 输出芯片给的真实 message，用于区分失败类型。
bool TryUpload(const uint8_t* data, uint32_t len, uint32_t burst, bool single,
               const char* label) {
    WriteReg(kRegCmd, kCmdSoftReset);
    vTaskDelay(pdMS_TO_TICKS(10));  // 手册要求 ≥2ms

    if (!WaitChipReady(100)) {
        ESP_LOGE(kTag, "%s：复位后 100ms 内 chip id 一直无效", label);
        return false;
    }

    const uint32_t t0 = esp_log_timestamp();
    const bool wrote = single ? UploadSingleBurst(data, len)
                              : UploadRanged(data, len, burst, true);
    uint8_t msg = 0xFF;
    const bool ok = wrote && WaitInitReady(&msg);
    const uint32_t cost = esp_log_timestamp() - t0;

    if (ok) {
        ESP_LOGI(kTag, "init data 就绪：%s，%lu 字节，耗时 %lu ms", label,
                 static_cast<unsigned long>(len), static_cast<unsigned long>(cost));
        return true;
    }
    // 0x02=初始化错误、0x03=固件不被认可，这两种基本可以断定镜像与这颗芯片不匹配。
    ESP_LOGW(kTag,
             "init data 失败：%s，message=0x%02X（0x01=ok 0x02=init_err 0x03=drv_err），"
             "耗时 %lu ms",
             label, msg, static_cast<unsigned long>(cost));
    return false;
}

// 软复位、上传 init data，再配好 ODR / 量程 / 使能。
// 返回 false 说明这颗芯片连基础寄存器都不认，只能放弃。
bool RawConfigure() {
    // BMI260 必须喂 init data，否则 PWR_CTRL 置位成功但数据恒为 0。
    // 先按 260 的镜像试不同粒度；都不行再拿 BMI270 的镜像赌一次
    // （部分「260」其实是 270 换标）。每轮都从软复位开始，避免残留状态。
    bool loaded = false;
    if (g_chip_id == 0x24) {
        loaded = TryUpload(bmi270_config_file, 8192, kInitBurst, false, "BMI270 fw/整包");
    } else {
        // 先按 260 的镜像试：分段 64B → 分段 2B → 整包 8KB。
        // 整包放最后是因为它最符合手册，但要用 8KB 静态缓冲。
        loaded = TryUpload(Bmi260InitData::kData, Bmi260InitData::kSize, kInitBurst, false,
                           "BMI260 fw/64B");
        if (!loaded) {
            loaded = TryUpload(Bmi260InitData::kData, Bmi260InitData::kSize, 2, false,
                               "BMI260 fw/2B");
        }
        if (!loaded) {
            loaded = TryUpload(Bmi260InitData::kData, Bmi260InitData::kSize, 0, true,
                               "BMI260 fw/整包");
        }
        if (!loaded) {
            loaded = TryUpload(bmi270_config_file, 8192, kInitBurst, false, "BMI270 fw/64B");
            if (loaded) {
                g_mode = "BMI260+BMI270fw";
            }
        }
        if (!loaded) {
            loaded = TryUpload(bmi270_config_file, 8192, 0, true, "BMI270 fw/整包");
            if (loaded) {
                g_mode = "BMI260+BMI270fw";
            }
        }
    }
    if (!loaded) {
        // 不直接放弃：继续把 ODR/量程/PWR_CTRL 写下去，后面的寄存器 dump
        // 能看出芯片到底接受哪些配置，是定位问题的关键证据。
        ESP_LOGE(kTag, "所有 init data 组合都加载失败，继续配置并 dump 现场");
    }

    // 关掉 advanced power save，否则传感器在两次采样之间会休眠，
    // 读出来的是陈旧值或直接不动。上传时已写过，这里重写更稳。
    WriteReg(kRegPwrConf, 0x00);
    vTaskDelay(pdMS_TO_TICKS(2));

    if (!WriteReg(kRegAccConf, kAccConf100Hz)) return false;
    if (!WriteReg(kRegAccRange, kAccRange4G)) return false;
    if (!WriteReg(kRegGyrConf, kGyrConf100Hz)) return false;
    if (!WriteReg(kRegGyrRange, kGyrRange500Dps)) return false;

    // 回读确认：真按下去和读到的不一致，说明写根本没生效。
    if (ReadReg(kRegAccConf) != kAccConf100Hz) {
        ESP_LOGW(kTag, "ACC_CONF 回读不一致（读到 0x%02X）", ReadReg(kRegAccConf));
        return false;
    }

    if (!WriteReg(kRegPwrCtrl, kPwrCtrlAll)) return false;
    vTaskDelay(pdMS_TO_TICKS(50));  // 等第一帧有效数据

    // 使能位能不能生效，是「配置成功」与「芯片是死的」之间的分界线。
    const uint8_t pwr = ReadReg(kRegPwrCtrl);
    ESP_LOGI(kTag, "PWR_CTRL 写 0x%02X 回读 0x%02X | STATUS=0x%02X ERR=0x%02X",
             kPwrCtrlAll, pwr, ReadReg(0x03), ReadReg(0x02));
    return true;
}

// 静止时加速度模长应当接近 1g。这一步是判断「这颗芯片能不能用」的唯一硬标准，
// 比 chip id 有意义得多——id 不匹配但数据正常，游戏照样能跑。
bool SelfCheck(float* out_g) {
    float sum = 0.0f;
    int ok = 0;
    int16_t last_acc[3] = {0, 0, 0};
    int16_t last_gyr[3] = {0, 0, 0};
    bool got_any = false;
    for (int i = 0; i < 8; i++) {
        int16_t acc[3];
        int16_t gyr[3];
        if (ReadRaw(acc, gyr)) {
            std::memcpy(last_acc, acc, sizeof(last_acc));
            std::memcpy(last_gyr, gyr, sizeof(last_gyr));
            got_any = true;
            const float x = acc[0] / kAccLsbPerG;
            const float y = acc[1] / kAccLsbPerG;
            const float z = acc[2] / kAccLsbPerG;
            const float mag = std::sqrt(x * x + y * y + z * z);
            // 明显不可能的值（全 0 或打满）不计入
            if (mag > 0.05f && mag < 8.0f) {
                sum += mag;
                ok++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (ok < 4) {
        // 把原始 LSB 打出来：全 0 说明没喂 init data 或传感器没在采样，
        // 打满/乱跳说明总线或量程配置有问题。这行是唯一的现场证据。
        ESP_LOGE(kTag,
                 "自检取样无效：I2C %s，原始 acc=[%d,%d,%d] gyr=[%d,%d,%d]，有效帧 %d/8",
                 got_any ? "正常" : "读失败", last_acc[0], last_acc[1], last_acc[2],
                 last_gyr[0], last_gyr[1], last_gyr[2], ok);
        return false;
    }
    *out_g = sum / ok;
    return (*out_g > 0.7f && *out_g < 1.3f);
}

void AppendBar(char* out, float value, float full_scale) {
    constexpr int kW = 11;
    constexpr int kMid = 5;
    float norm = value / full_scale;
    if (norm > 1.0f) norm = 1.0f;
    if (norm < -1.0f) norm = -1.0f;
    const int mag = static_cast<int>(std::lroundf(std::fabs(norm) * kMid));
    for (int i = 0; i < kW; i++) {
        if (i == kMid) {
            out[i] = '|';
        } else if (i < kMid) {
            out[i] = (value < 0.0f && (kMid - i) <= mag) ? '<' : '.';
        } else {
            out[i] = (value > 0.0f && (i - kMid) <= mag) ? '>' : '.';
        }
    }
    out[kW] = '\0';
}

// ===========================================================================
// 姿态解算：四元数非线性互补滤波（PX4 attitude_estimator_q 的移植）
//
// 为什么不用欧拉角：两个角度各自减零点看着简单，但① 绕机身轴大角度转时
// 数值失真，② 零点不在水平面上时「角度相减」不等于相对转角，③ pitch
// 接近 ±90° 会奇异。四元数没有这些问题，代价是几十行乘加。
//
// 每帧四步（dt 固定 10ms）：
//   1. 预测：把陀螺读数（扣掉零偏）当角速度，右乘一个增量四元数
//   2. 校正：估计出的重力方向 k = R(q)ᵀ·(0,0,1) 与实测 â 叉乘得到误差，
//      以 kAccWeight 的比例加进角速度——这就是「互补」的那一半
//   3. 零偏：只在确认静止 1 秒后，直接把当前陀螺读数取反当零偏。
//      PX4 那种「按校正量积分」的做法在这里会跑飞，原因见该处注释。
//   4. 航向死区：低于 kYawDeadbandDps 的 z 轴角速度不积分
// ===========================================================================

struct Quat {
    float w, x, y, z;
};

Quat QuatMul(const Quat& a, const Quat& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

Quat QuatNormalize(const Quat& q) {
    const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-9f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float inv = 1.0f / n;
    return {q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

// R(q)ᵀ·v，即把世界系向量转到机体系。姿态解算里要的是「重力在机体系的方向」。
void QuatRotateInv(const Quat& q, const float v[3], float out[3]) {
    const float cw = q.w;
    const float cx = -q.x;
    const float cy = -q.y;
    const float cz = -q.z;
    const float tx = 2.0f * (cy * v[2] - cz * v[1]);
    const float ty = 2.0f * (cz * v[0] - cx * v[2]);
    const float tz = 2.0f * (cx * v[1] - cy * v[0]);
    out[0] = v[0] + cw * tx + (cy * tz - cz * ty);
    out[1] = v[1] + cw * ty + (cz * tx - cx * tz);
    out[2] = v[2] + cw * tz + (cx * ty - cy * tx);
}

// 旋转向量（弧度）→ 四元数。这是精确的指数映射，不是一阶近似，
// 所以 100dps 时单帧 1° 的步长也不会积出偏差。
Quat QuatFromRotVec(const float rv[3]) {
    const float ang = std::sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
    if (ang < 1e-9f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float half = 0.5f * ang;
    const float s = std::sin(half) / ang;
    return {std::cos(half), rv[0] * s, rv[1] * s, rv[2] * s};
}

// 角度差归一到 ±180°。零点存的是绝对角，两者相减在 ±180 附近会跳。
float WrapDeg(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// 冷启动对齐：找到一个旋转，把实测「上」方向转到世界系 z 轴。
// 航向随便定（没有磁力计，本来就不可观测），倾斜一上电就是准的，
// 不用靠滤波慢慢收敛。
Quat QuatAlignUp(const float up[3]) {
    float n = std::sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    if (n < 1e-6f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    const float ux = up[0] / n;
    const float uy = up[1] / n;
    const float uz = up[2] / n;
    // up × (0,0,1)
    const float ax = uy * 1.0f - uz * 0.0f;
    const float ay = uz * 0.0f - ux * 1.0f;
    const float az = 0.0f;
    const float axis_len = std::sqrt(ax * ax + ay * ay);
    const float dot = uz;
    if (axis_len < 1e-6f) {
        if (dot > 0.0f) {
            return {1.0f, 0.0f, 0.0f, 0.0f};
        }
        return {0.0f, 1.0f, 0.0f, 0.0f};  // 倒置：绕 x 轴转 180°
    }
    const float half = 0.5f * std::acos(dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot));
    const float s = std::sin(half) / axis_len;
    return {std::cos(half), ax * s, ay * s, az * s};
}

// 欧拉角（度）。roll / pitch 等价于「重力在机体系的方向」——
//   roll  = atan2(u_y, u_z)，pitch = atan2(-u_x, √(u_y²+u_z²))
// 其中 u = R(q)ᵀ·(0,0,1) 是重力在机体系的方向。写成下面这种形式只是
// 省一次矩阵乘，两者在数学上完全一样。
// pitch 用 atan2 而不是 asin(2(wy-zx))：asin 在 ±90° 附近抖，且过了
// ±90° 会折回来，倾角输出靠这两个角，不能留这种坑。
void QuatToEuler(const Quat& q, float* roll, float* pitch, float* yaw) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    const float uy = 2.0f * (y * z + w * x);          // 重力的机体系 y 分量
    const float uz = 1.0f - 2.0f * (x * x + y * y);   // 重力的机体系 z 分量
    const float ux = 2.0f * (x * z - w * y);          // 重力的机体系 x 分量
    *roll = std::atan2(uy, uz) * kRadToDeg;
    *pitch = std::atan2(-ux, std::sqrt(uy * uy + uz * uz)) * kRadToDeg;
    *yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * kRadToDeg;
}

void SampleTask(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t seq = 0;
    int log_div = 0;
    int read_fail = 0;

    // 滤波器状态：全部是本任务私有的，共享只通过 g_latest / g_zero_request
    Quat q{1.0f, 0.0f, 0.0f, 0.0f};
    // 零点默认「水平面」= 不改任何东西，tilt 就从开机起等于绝对倾角。
    // 要改成手持中性位就调 Zero()，那时才写入这两个角。
    float roll_zero = 0.0f;
    float pitch_zero = 0.0f;
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};   // dps
    int still_cnt = 0;                          // 连续静止帧数
    bool aligned = false;
    const float dt = kSamplePeriodMs / 1000.0f;

    while (true) {
        int16_t acc[3];
        int16_t gyr[3];
        if (ReadRaw(acc, gyr)) {
            read_fail = 0;
            const float ax = acc[0] / kAccLsbPerG;
            const float ay = acc[1] / kAccLsbPerG;
            const float az = acc[2] / kAccLsbPerG;
            const float gx = gyr[0] / kGyrLsbPerDps;
            const float gy = gyr[1] / kGyrLsbPerDps;
            const float gz = gyr[2] / kGyrLsbPerDps;

            // —— 第 0 帧：用加速度把姿态直接对齐，倾斜不用等收敛 ——
            // 只对齐姿态，不动零点：零点默认就是水平面（roll/pitch 都为 0）。
            // 这里顺手把零点也设成本帧的话，tilt 就变成「相对开机摆法」，
            // 同一个物理姿势开两次机读数不同，没法当标定基准。
            if (!aligned) {
                const float up[3] = {ax, ay, az};
                q = QuatAlignUp(up);
                aligned = true;
                ESP_LOGI(kTag, "姿态已对齐，零点 = 水平面（要改成手持中性位请调 /imu/zero）");
            }

            // —— 第 1 步：预测。陀螺读数 + 零偏估计 = 实际角速度 ——
            float corr[3] = {gx + gyro_bias[0], gy + gyro_bias[1], gz + gyro_bias[2]};

            // —— 第 2 步：重力校正 ——
            // 只在「几乎只受重力」时才信加速度：手持晃动叠上的线加速度
            // 会把模长顶出 1g 附近，那几帧的重力方向是错的，必须丢掉。
            float err[3] = {0.0f, 0.0f, 0.0f};
            const float acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
            bool acc_ok = (acc_norm > kAccGateLow && acc_norm < kAccGateHigh);
            if (acc_ok) {
                const float world_up[3] = {0.0f, 0.0f, 1.0f};
                float k[3];
                QuatRotateInv(q, world_up, k);      // 估计的"上"在机体系
                const float inv = 1.0f / acc_norm;
                const float a[3] = {ax * inv, ay * inv, az * inv};
                // 误差 = k × â。符号取负：叉积方向是把 k 转向 â 的轴，
                // 而我们是要把估计转回去，所以叠加时取反。
                err[0] = k[1] * a[2] - k[2] * a[1];
                err[1] = k[2] * a[0] - k[0] * a[2];
                err[2] = k[0] * a[1] - k[1] * a[0];
                // ★ 必须乘 kRadToDeg：err 是「单位误差的角速度」，量纲是 rad/s；
                // 而 corr 里装的是陀螺读数，量纲是 dps。少乘这一步，校正强度
                // 只有设计值的 1/57.3，滤波器几乎纯靠陀螺积分——姿态跟加速度计
                // 差着好几度、收敛慢几十倍，而代码看上去完全正常。
                for (int i = 0; i < 3; i++) {
                    corr[i] -= err[i] * kAccWeight * kRadToDeg;
                }
            }

            // —— 第 3 步：零偏。只做「静止时直接抓」，不做在线积分学习 ——
            //
            // 曾经按 PX4 的思路用校正量积分去学零偏（attitude_estimator_q 的
            // ATT_W_GYRO_BIAS），实机跑 50 秒就把三轴全都顶到 ±1.2dps 限幅：
            // 手持慢速倾斜只有十几 dps，会被当成零偏学进去，然后往积分里灌
            // 1.2dps 的假角速度，12 秒积出 14°，倾角直接错 18°。
            // 而且 x/y 的零偏本来就被重力校正钉死了，学它没有收益；z 轴
            // （航向）重力观测不到，也学不出来。所以干脆不学。
            //
            // 真正有用的是「静止时把当前陀螺读数当零偏」：手放着不动一秒，
            // gz 立刻归零，航向漂移从根上没了，比死区干净。
            const bool still = acc_ok && std::fabs(gx) < kStillRateDps &&
                               std::fabs(gy) < kStillRateDps &&
                               std::fabs(gz) < kStillRateDps;
            still_cnt = still ? still_cnt + 1 : 0;
            if (still_cnt == kStillFrames) {
                gyro_bias[0] = -gx;   // 积分里用的是 gx + bias，所以取负
                gyro_bias[1] = -gy;
                gyro_bias[2] = -gz;
                for (int i = 0; i < 3; i++) {
                    if (gyro_bias[i] > kGyroBiasMaxDps) gyro_bias[i] = kGyroBiasMaxDps;
                    if (gyro_bias[i] < -kGyroBiasMaxDps) gyro_bias[i] = -kGyroBiasMaxDps;
                }
                ESP_LOGI(kTag, "静止 1s，零偏更新 %+.2f %+.2f %+.2f dps",
                         gyro_bias[0], gyro_bias[1], gyro_bias[2]);
            }

            // 航向死区：静止时陀螺残余零偏会让 yaw 一直爬
            if (std::fabs(corr[2]) < kYawDeadbandDps) {
                corr[2] = 0.0f;
            }

            // —— 第 4 步：把角速度积进四元数（右乘，机体系增量） ——
            // ★ corr 的单位是 **度/秒**（陀螺 dps + 校正项），乘 dt 得到「度」；
            // 而 QuatFromRotVec 要的是 **弧度**。少乘 kDegToRad 就是拿「度」
            // 当「弧度」用，每帧积分被放大 57.3 倍：
            //   · 静止时 gyro≈0，积分项也≈0，靠重力校正还能把姿态钉住，
            //     于是所有静态标定看起来都正常（收敛快、读数准）——极具欺骗性；
            //   · 一旦动起来（20dps 倾斜），真实 0.2°/帧被积成 11.5°/帧，
            //     4 秒能转过 1700°，实测手持倾斜 40° 时估计值冲出 35°、
            //     单帧跳 22°，也就是「手机端看到的姿态在跳变」。
            // 仿真复刻验证：修复前 max|err| 35.06°、单帧跳变 22.72°；
            //               修复后 max|err| 0.20°、单帧跳变 0.20°。
            const float rv[3] = {corr[0] * dt * kDegToRad, corr[1] * dt * kDegToRad,
                                 corr[2] * dt * kDegToRad};
            q = QuatNormalize(QuatMul(q, QuatFromRotVec(rv)));

            // —— 输出：欧拉角给日志看，倾角是「相对零点的 roll/pitch 差」 ——
            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            QuatToEuler(q, &roll, &pitch, &yaw);

            // 零点请求放在算完欧拉角之后：零点要同时记住姿态角。
            if (g_zero_request) {
                g_zero_request = false;
                roll_zero = roll;
                pitch_zero = pitch;
                ESP_LOGI(kTag, "零点已更新：roll=%+.1f° pitch=%+.1f°", roll_zero, pitch_zero);
            }

            // 倾角为什么不用「q_rel 的旋转向量 x/y 分量」？
            // 因为那个量含 yaw：手腕一边倾一边拧（绕竖直轴自转），重力方向
            // 只转了 39°，旋转向量却涨到 59°——实测直接过读 20°。
            // 而 roll / pitch 是由「重力在机体系的方向」定义的：绕竖直轴自转
            // 不改变重力在机体系的方向，所以这两个角对 yaw 天然免疫。
            // 代价是两个轴同时倾很大时，向量和会比真实转角略大（30~40° 时
            // 约 1°，极端斜角最多 9°），比 20° 的 yaw 污染小一个量级。
            const float tilt_x = WrapDeg(roll - roll_zero);
            const float tilt_y = WrapDeg(pitch - pitch_zero);

            ImuReadout snap;
            snap.ready = true;
            snap.chip_id = g_chip_id;
            for (int i = 0; i < 3; i++) {
                snap.acc_lsb[i] = acc[i];
                snap.gyr_lsb[i] = gyr[i];
                snap.gyro_bias[i] = gyro_bias[i];
            }
            snap.acc_g[0] = ax;
            snap.acc_g[1] = ay;
            snap.acc_g[2] = az;
            snap.gyr_dps[0] = gx;
            snap.gyr_dps[1] = gy;
            snap.gyr_dps[2] = gz;
            snap.q[0] = q.w;
            snap.q[1] = q.x;
            snap.q[2] = q.y;
            snap.q[3] = q.z;
            snap.roll = roll;
            snap.pitch = pitch;
            snap.yaw = yaw;
            snap.tilt_x = tilt_x;
            snap.tilt_y = tilt_y;
            snap.seq = seq + 1;
            snap.sample_hz = static_cast<uint16_t>(1000 / kSamplePeriodMs);

            portENTER_CRITICAL(&g_mux);
            g_latest = snap;
            portEXIT_CRITICAL(&g_mux);

            if (g_serial_log && ++log_div >= kLogEveryN) {
                log_div = 0;
                char bar_x[12];
                char bar_y[12];
                AppendBar(bar_x, snap.tilt_x, 30.0f);
                AppendBar(bar_y, snap.tilt_y, 30.0f);
                ESP_LOGI(kTag,
                         "x[%s] y[%s] R%+6.1f P%+6.1f | a[%+.2f %+.2f %+.2f]g g[%+6.1f %+6.1f %+6.1f]",
                         bar_x, bar_y, roll, pitch, ax, ay, az, gx, gy, gz);
            }
            seq++;
        } else if (++read_fail == 50) {
            // 连续半秒读不到再报，避免偶发一次超时刷屏
            ESP_LOGW(kTag, "连续读 I2C 失败，传感器可能掉线");
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kSamplePeriodMs));
    }
}

}  // namespace

namespace Bmi260Imu {

bool Start(i2c_master_bus_handle_t bus, uint8_t addr) {
    if (g_started) {
        return true;
    }
    if (bus == nullptr) {
        ESP_LOGE(kTag, "I2C 总线为空");
        g_init_err = -1;
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 400000;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &g_ctx.dev);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "挂载 I2C 设备失败: %s", esp_err_to_name(err));
        g_init_err = err;
        return false;
    }

    uint8_t id = ReadReg(kRegChipId);
    g_chip_id = id;
    if (id == 0x00 || id == 0xFF) {
        ESP_LOGE(kTag, "chip id = 0x%02X，读不到有效值：传感器没焊、地址不对或总线没通", id);
        g_init_err = -2;
        return false;
    }

    // 只做提示，不做拦截——0x27 这类非标件 id 对不上，但数据可能是好的，
    // 下面用加速度模长自检来判定能不能用。
    if (id == 0x24) {
        g_mode = "BMI270";
        ESP_LOGI(kTag, "chip id = 0x24 (BMI270)");
    } else if (id == 0x26) {
        g_mode = "BMI220";
        ESP_LOGW(kTag, "chip id = 0x26，是 BMI220 而非 BMI270（官方也适配了这一款）");
    } else if (id == 0x27) {
        g_mode = "BMI260";
        ESP_LOGI(kTag, "chip id = 0x27 (BMI260)，用 BMI260 init data");
    } else {
        g_mode = "compat";
        ESP_LOGW(kTag,
                 "chip id = 0x%02X，不是 BMI270(0x24) / BMI220(0x26) / BMI260(0x27)，"
                 "按 BMI260 兼容件处理",
                 id);
    }

    if (!RawConfigure()) {
        // 软复位没生效也可能是这颗芯片cmd寄存器不同，再给一次机会：
        // 不复位、直接沿用上电默认状态试试。
        ESP_LOGW(kTag, "软复位/配置未确认生效，尝试直接用当前状态采样");
    }

    float g_mag = 0.0f;
    if (!SelfCheck(&g_mag)) {
        // 只有出问题才打寄存器快照：正常开机不该刷这一屏，
        // 但真坏了又有现场可看。
        DumpRegisters("自检失败现场");
        ESP_LOGE(kTag,
                 "自检失败：加速度模长 %.2fg（应接近 1.0g）。这颗芯片读不出有效姿态数据",
                 g_mag);
        g_init_err = -3;
        return false;
    }
    ESP_LOGI(kTag, "自检通过：静止加速度 %.2fg ✓ 传感器可用（%s）", g_mag, g_mode);

    g_latest.ready = false;
    g_latest.chip_id = g_chip_id;
    g_init_err = 0;
    g_started = true;

    if (xTaskCreate(SampleTask, "imu_sample", 3072, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(kTag, "采样任务创建失败");
        g_started = false;
        g_init_err = -4;
        return false;
    }

    ESP_LOGI(kTag,
             "就绪：100Hz ±4g/±500dps，四元数互补滤波（acc=%.2f，零偏静止抓取±%.1fdps）",
             kAccWeight, kGyroBiasMaxDps);
    return true;
}

bool GetLatest(ImuReadout* out) {
    if (out == nullptr) {
        return false;
    }
    portENTER_CRITICAL(&g_mux);
    *out = g_latest;
    portEXIT_CRITICAL(&g_mux);
    return out->ready;
}

void Zero() {
    // 真正取零点的是采样任务（它持有滤波器状态）。这里只置位，
    // 下一帧（≤10ms）生效，误差可以忽略，换来的是不用跨任务抢四元数。
    if (!g_started) {
        ESP_LOGW(kTag, "IMU 未启动，忽略零点请求");
        return;
    }
    g_zero_request = true;
}

void SetSerialLog(bool on) {
    g_serial_log = on;
    ESP_LOGI(kTag, "串口波形 %s", on ? "开" : "关");
}

uint8_t ChipId() { return g_chip_id; }

int LastInitError() { return g_init_err; }

int FormatJson(char* buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return 0;
    }
    ImuReadout r;
    const bool ok = GetLatest(&r);

    int n;
    if (!ok) {
        n = std::snprintf(buf, len,
                          "{\"ready\":false,\"chip_id\":%u,\"mode\":\"%s\",\"err\":%d}",
                          static_cast<unsigned>(g_chip_id), g_mode, g_init_err);
    } else {
        n = std::snprintf(
            buf, len,
            "{\"ready\":true,\"chip_id\":%u,\"mode\":\"%s\",\"hz\":%u,\"seq\":%lu,"
            "\"acc_g\":[%.4f,%.4f,%.4f],\"gyr_dps\":[%.2f,%.2f,%.2f],"
            "\"acc_lsb\":[%d,%d,%d],"
            "\"q\":[%.4f,%.4f,%.4f,%.4f],\"gbias\":[%.2f,%.2f,%.2f],"
            "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
            "\"tilt_x\":%.2f,\"tilt_y\":%.2f}",
            static_cast<unsigned>(r.chip_id), g_mode,
            static_cast<unsigned>(r.sample_hz),
            static_cast<unsigned long>(r.seq), r.acc_g[0], r.acc_g[1], r.acc_g[2],
            r.gyr_dps[0], r.gyr_dps[1], r.gyr_dps[2], r.acc_lsb[0], r.acc_lsb[1],
            r.acc_lsb[2], r.q[0], r.q[1], r.q[2], r.q[3],
            r.gyro_bias[0], r.gyro_bias[1], r.gyro_bias[2],
            r.roll, r.pitch, r.yaw, r.tilt_x, r.tilt_y);
    }
    if (n < 0) {
        return 0;
    }
    return (static_cast<size_t>(n) < len) ? n : static_cast<int>(len - 1);
}

}  // namespace Bmi260Imu
