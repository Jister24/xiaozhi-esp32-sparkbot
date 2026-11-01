#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
#include "chassis_battery.h"
#include "config.h"
#include "local_control.h"  // LOCAL_CONTROL_PATCH
#include "display/lcd_display.h"
#include "display/emote_display.h"  // EMOTE_STYLE
#include "mcp_server.h"
#include "settings.h"
#include "wifi_board.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/touch_sens.h>
#include <driver/uart.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <cstdlib>
#include <cstring>
#include <string>

#include "esp_video.h"

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ===========================================================================
// 电池检测（主体板）
//
// 官方固件不读电池、原理图未公开，引脚是实测扫出来的：把 ADC1/ADC2 所有空闲
// 通道逐个采样，只有 GPIO14（ADC2_CH3）读数稳定且量级像电池分压——
// 满电插 USB 时引脚 2084 mV、抖动 <10 mV，按等值电阻分压还原即 4.17 V。
// 其余空闲脚要么恒为 0，要么抖动上百 LSB（悬空）。I2C 上也只有 ES8311 与
// BMI270，没有 IP5306 / MAX17048 之类的电量计。
//
// 充电状态没有引到 S3（底盘侧的 C2 才知道自己那块电池在不在充），所以这里
// 只能靠电压趋势做启发式判断：连续 3 分钟电压抬升或逼近满电即认为在充电。
// ===========================================================================
class SparkBotBattery {
public:
    bool Begin() {
        adc_oneshot_unit_init_cfg_t unit_cfg = {};
        unit_cfg.unit_id = BATTERY_ADC_UNIT;
        unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
        esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &unit_);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "ADC 单元创建失败: %s", esp_err_to_name(err));
            return false;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {};
        chan_cfg.atten = ADC_ATTEN_DB_12;
        chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
        err = adc_oneshot_config_channel(unit_, BATTERY_ADC_CHANNEL, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "ADC 通道配置失败: %s", esp_err_to_name(err));
            return false;
        }

        // 曲线拟合校准：没有校准表也能出数，只是绝对精度差几十 mV，
        // 对电量百分比影响不大，所以失败不致命。
        adc_cali_curve_fitting_config_t cali_cfg = {};
        cali_cfg.unit_id = BATTERY_ADC_UNIT;
        cali_cfg.atten = ADC_ATTEN_DB_12;
        cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_) != ESP_OK) {
            cali_ = nullptr;
        }

        ready_ = true;
        ESP_LOGI(kTag, "电池检测就绪，cali=%d", cali_ != nullptr);
        return true;
    }

    bool IsReady() const { return ready_; }

    // 采样一次并更新平滑值。16 次取平均可以把 ±5 LSB 的抖动压到 1 LSB 量级。
    void Update() {
        if (!ready_) {
            return;
        }
        int sum = 0;
        int ok = 0;
        for (int i = 0; i < kSamples; i++) {
            int raw = 0;
            if (adc_oneshot_read(unit_, BATTERY_ADC_CHANNEL, &raw) != ESP_OK) {
                continue;
            }
            sum += raw;
            ok++;
        }
        if (ok == 0) {
            return;
        }
        int pin_mv = sum / ok;
        if (cali_ != nullptr) {
            int mv = 0;
            if (adc_cali_raw_to_voltage(cali_, sum / ok, &mv) == ESP_OK) {
                pin_mv = mv;
            }
        }
        const float cell = pin_mv * (BATTERY_UPPER_RESISTOR + BATTERY_LOWER_RESISTOR) /
                           BATTERY_LOWER_RESISTOR;
        // 一阶低通：充电/放电都慢，没必要让显示跟着负载跳。
        cell_mv_ = (cell_mv_ <= 0) ? static_cast<int>(cell)
                                   : static_cast<int>(cell_mv_ * 0.7f + cell * 0.3f);

        history_[history_index_] = cell_mv_;
        history_index_ = (history_index_ + 1) % kHistory;
        if (history_count_ < kHistory) {
            history_count_++;
        }
    }

    int CellMillivolts() const { return cell_mv_; }

    int Percent() const {
        if (cell_mv_ <= 0) {
            return 0;
        }
        // 锂电开路电压 → 剩余电量，取自 adc_battery_estimation 的默认曲线
        static const struct {
            int mv;
            int pct;
        } kOcv[] = {
            {4160, 100}, {4070, 90}, {3990, 80}, {3900, 70}, {3820, 60}, {3720, 50},
            {3610, 40},  {3530, 30}, {3380, 20}, {3200, 10}, {2850, 0},
        };
        constexpr int kCount = sizeof(kOcv) / sizeof(kOcv[0]);
        if (cell_mv_ >= kOcv[0].mv) {
            return 100;
        }
        for (int i = 0; i < kCount - 1; i++) {
            if (cell_mv_ <= kOcv[i].mv && cell_mv_ >= kOcv[i + 1].mv) {
                const int dv = kOcv[i].mv - kOcv[i + 1].mv;
                const int dp = kOcv[i].pct - kOcv[i + 1].pct;
                return kOcv[i + 1].pct + (cell_mv_ - kOcv[i + 1].mv) * dp / dv;
            }
        }
        return 0;
    }

    // 纯启发式：插着 USB 时电压会被充电电路顶高，3 分钟内上涨超过 12 mV，
    // 或已经贴在满电电压上，就认为在充电。拔掉后电压回落即判定为放电。
    bool IsCharging() const {
        if (cell_mv_ <= 0) {
            return false;
        }
        if (cell_mv_ >= kFullMv - 10) {
            return true;
        }
        if (history_count_ < kHistory) {
            return false;
        }
        // history_index_ 指向下一个要写入的槽位，也就是 3 分钟前那一个样本
        const int oldest = history_[history_index_];
        return cell_mv_ - oldest >= 12;
    }

private:
    static constexpr const char* kTag = "sparkbot_batt";
    static constexpr int kSamples = 16;
    static constexpr int kHistory = 36;  // 5s × 36 = 3 分钟
    static constexpr int kFullMv = 4160;

    adc_oneshot_unit_handle_t unit_ = nullptr;
    adc_cali_handle_t cali_ = nullptr;
    bool ready_ = false;
    int cell_mv_ = -1;
    int history_[kHistory] = {};
    int history_index_ = 0;
    int history_count_ = 0;
};
// ===== BATTERY END =====

#define TAG "esp_sparkbot"

class SparkBotEs8311AudioCodec : public Es8311AudioCodec {
private:
public:
    SparkBotEs8311AudioCodec(void* i2c_master_handle, i2c_port_t i2c_port, int input_sample_rate,
                             int output_sample_rate, gpio_num_t mclk, gpio_num_t bclk,
                             gpio_num_t ws, gpio_num_t dout, gpio_num_t din, gpio_num_t pa_pin,
                             uint8_t es8311_addr, bool use_mclk = true)
        : Es8311AudioCodec(i2c_master_handle, i2c_port, input_sample_rate, output_sample_rate, mclk,
                           bclk, ws, dout, din, pa_pin, es8311_addr, use_mclk = true) {}

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) {
            return;
        }
        if (enable) {
            Es8311AudioCodec::EnableOutput(enable);
        } else {
            // Nothing todo because the display io and PA io conflict
        }
    }
};

class EspSparkBot : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Display* display_;
    EspVideo* camera_;
    SparkBotBattery battery_;
    ChassisBattery chassis_battery_;  // 走 UART 问底盘那颗 C2
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
    emote::EmoteDisplay* emote_display_ = nullptr;  // 电量角标要往它上面推
#endif
    light_mode_t light_mode_ = LIGHT_MODE_ALWAYS_ON;
    // 触摸按键：FPC 铜箔贴在顶部与左右两侧，直接进 S3 的触摸通道 1/2/3
    touch_sensor_handle_t touch_sens_ = nullptr;
    touch_channel_handle_t touch_chan_[3] = {nullptr, nullptr, nullptr};
    // 回调里只置标志，不在中断上下文做任何耗时动作。
    // 用 inline static 省掉类外定义；回调是无捕获 lambda，只能碰静态成员。
    inline static bool touch_active_[3] = {false, false, false};
    inline static uint32_t touch_hits_[3] = {0, 0, 0};
    bool sleep_face_on_ = false;  // 低功耗时是否已经把脸换成睡觉

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags =
                {
                    .enable_internal_pullup = 1,
                },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_GPIO;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_GPIO;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // -----------------------------------------------------------------------
    // 触摸按键读取（FPC 上的铜箔电极，不是屏幕触摸）
    // 官方结构：顶部一块（因摄像头被切成左右两片，串接在同一个引脚）+ 左右
    // 侧面各一块，共 3 路，分别接 GPIO1/2/3，也就是 S3 的触摸通道 1/2/3。
    // S3 自带电容触摸外设（SOC_TOUCH_SENSOR_SUPPORTED=1，V2，最多 14 通道），
    // 所以板上没有触摸 IC，普通 GPIO 数字读法永远读不出东西，必须走触摸外设。
    // -----------------------------------------------------------------------
    bool EnsureTouchReady() {
        if (touch_sens_ != nullptr) {
            return true;
        }
        // 不用 TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG 宏：它内部是 GNU 风格的
        // 指定初始化器，套在 C++ 的数组初始化上编不过，这里逐字段赋同样的默认值。
        static touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM];
        sample_cfg[0].charge_times = 500;
        sample_cfg[0].charge_volt_lim_h = TOUCH_VOLT_LIM_H_2V2;
        sample_cfg[0].charge_volt_lim_l = TOUCH_VOLT_LIM_L_0V5;
        sample_cfg[0].idle_conn = TOUCH_IDLE_CONN_GND;
        sample_cfg[0].bias_type = TOUCH_BIAS_TYPE_SELF;

        touch_sensor_config_t sens_cfg = {};
        sens_cfg.power_on_wait_us = 256;
        sens_cfg.meas_interval_us = 32.0f;
        sens_cfg.max_meas_time_us = 0;
        sens_cfg.sample_cfg_num = TOUCH_SAMPLE_CFG_NUM;
        sens_cfg.sample_cfg = sample_cfg;
        if (touch_sensor_new_controller(&sens_cfg, &touch_sens_) != ESP_OK) {
            ESP_LOGE(TAG, "touch controller init failed");
            touch_sens_ = nullptr;
            return false;
        }
        for (int i = 0; i < 3; ++i) {
            touch_channel_config_t chan_cfg = {};
            chan_cfg.active_thresh[0] = 2000;
            chan_cfg.charge_speed = TOUCH_CHARGE_SPEED_7;
            chan_cfg.init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT;
            if (touch_sensor_new_channel(touch_sens_, i + 1, &chan_cfg, &touch_chan_[i]) != ESP_OK) {
                ESP_LOGE(TAG, "touch channel %d init failed", i + 1);
                return false;
            }
        }
        // 官方示例 Step 3：显式配置滤波器。缺了这一步 smooth 数据可能不随触摸更新。
        touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
        touch_sensor_config_filter(touch_sens_, &filter_cfg);
        if (touch_sensor_enable(touch_sens_) != ESP_OK) {
            return false;
        }
        // 初扫几次把基准值（benchmark）建立起来，否则第一次读到的数据没有参考
        for (int k = 0; k < 5; ++k) {
            touch_sensor_trigger_oneshot_scanning(touch_sens_, 2000);
        }
        // 官方示例 Step 5：回调。这是判断"有没有摸到"最直接的证据。
        touch_event_callbacks_t callbacks = {};
        callbacks.on_active = [](touch_sensor_handle_t, const touch_active_event_data_t* e, void*) -> bool {
            int idx = (int)e->chan_id - 1;
            if (idx >= 0 && idx < 3) {
                touch_active_[idx] = true;
                touch_hits_[idx] = touch_hits_[idx] + 1;
            }
            return false;
        };
        callbacks.on_inactive = [](touch_sensor_handle_t, const touch_inactive_event_data_t* e, void*) -> bool {
            int idx = (int)e->chan_id - 1;
            if (idx >= 0 && idx < 3) {
                touch_active_[idx] = false;
            }
            return false;
        };
        touch_sensor_register_callbacks(touch_sens_, &callbacks, nullptr);
        // 官方示例 Step 7：连续扫描。手动单次触发时滤波收敛不充分，读数几乎不响应。
        touch_sensor_start_continuous_scanning(touch_sens_);
        return true;
    }

    // ?scan=1 —— 把所有触摸通道（CH1~CH13）一次性建起来读一遍基准值。
    // 目的：判断 GPIO1/2/3 上到底有没有接东西。如果某几个通道的基准值明显
    // 高于其余通道，说明那几个脚挂了电极；如果 14 个通道读数都差不多，
    // 那就是内部走线差异，电极其实没接在这三个脚上。
    // 跳过 CH4/CH5（I2C 的 GPIO4/5，避免动到音频 codec 的总线）。
    bool ScanAllChannels(std::string& out) {
        if (!EnsureTouchReady()) {
            out = "{\"ok\":false,\"err\":\"touch init failed\"}";
            return true;
        }
        touch_sensor_stop_continuous_scanning(touch_sens_);
        touch_sensor_disable(touch_sens_);

        struct Item { int id; touch_channel_handle_t h; bool owned; };
        static Item items[16];
        int n = 0;
        touch_channel_config_t chan_cfg = {};
        chan_cfg.active_thresh[0] = 2000;
        chan_cfg.charge_speed = TOUCH_CHARGE_SPEED_7;
        chan_cfg.init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT;

        for (int id = 1; id <= 13; ++id) {
            if (id == 4 || id == 5) {
                continue;
            }
            if (id <= 3) {
                items[n].id = id;
                items[n].h = touch_chan_[id - 1];
                items[n].owned = false;
                n++;
                continue;
            }
            touch_channel_handle_t h = nullptr;
            if (touch_sensor_new_channel(touch_sens_, id, &chan_cfg, &h) == ESP_OK) {
                items[n].id = id;
                items[n].h = h;
                items[n].owned = true;
                n++;
            }
        }

        touch_sensor_enable(touch_sens_);
        for (int k = 0; k < 8; ++k) {
            touch_sensor_trigger_oneshot_scanning(touch_sens_, 2000);
        }

        out = "{\"ok\":true,\"channels\":[";
        bool first = true;
        for (int i = 0; i < n; ++i) {
            uint32_t b[TOUCH_SAMPLE_CFG_NUM] = {};
            uint32_t v[TOUCH_SAMPLE_CFG_NUM] = {};
            touch_channel_read_data(items[i].h, TOUCH_CHAN_DATA_TYPE_BENCHMARK, b);
            touch_channel_read_data(items[i].h, TOUCH_CHAN_DATA_TYPE_SMOOTH, v);
            touch_chan_info_t info = {};
            touch_sensor_get_channel_info(items[i].h, &info);
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "%s{\"ch\":%d,\"gpio\":%d,\"bench\":%lu,\"smooth\":%lu}",
                     first ? "" : ",", items[i].id, (int)info.chan_gpio,
                     (unsigned long)b[0], (unsigned long)v[0]);
            first = false;
            out += buf;
        }
        out += "]}";

        for (int i = 0; i < n; ++i) {
            if (items[i].owned) {
                touch_sensor_del_channel(items[i].h);
            }
        }
        // 删完恢复连续扫描
        touch_sensor_start_continuous_scanning(touch_sens_);
        return true;
    }

    // ?touch=1&ms=8000 —— 读三路触摸通道。手指按机身顶部或左右侧面时，
    // 被按那一路的 smooth 会明显低于 bench；按住不放，min 就是触摸时的值。
    bool ProbeTouch(const char* query, std::string& out) {
        if (strstr(query, "touch=") == nullptr) {
            return false;
        }
        if (!EnsureTouchReady()) {
            out = "{\"ok\":false,\"err\":\"touch init failed\"}";
            return true;
        }
        int sample_ms = ParseQueryInt(query, "ms", 0);
        if (sample_ms < 0) sample_ms = 0;
        if (sample_ms > 20000) sample_ms = 20000;

        uint32_t bench[3] = {0, 0, 0};
        uint32_t smooth[3] = {0, 0, 0};
        uint32_t raw[3] = {0, 0, 0};
        uint32_t vmin[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
        uint32_t vmax[3] = {0, 0, 0};
        uint32_t rmin[3] = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
        uint32_t rmax[3] = {0, 0, 0};

        // 加 ?series=1 时把每轮 smooth 逐点记下来：峰值可能只持续一两轮，
        // 只看 min/max 的话短暂触摸会被平均掉，看不出是哪一路在动。
        const bool series = strstr(query, "series=") != nullptr;
        constexpr int kMaxTrace = 400;
        static uint32_t trace[3][kMaxTrace];
        int trace_n = 0;

        for (int i = 0; i < 3; ++i) {
            touch_hits_[i] = 0;
        }
        int rounds = sample_ms > 0 ? (sample_ms / 50) : 1;
        for (int r = 0; r < rounds; ++r) {
            for (int i = 0; i < 3; ++i) {
                uint32_t b[TOUCH_SAMPLE_CFG_NUM] = {};
                uint32_t v[TOUCH_SAMPLE_CFG_NUM] = {};
                uint32_t w[TOUCH_SAMPLE_CFG_NUM] = {};
                if (touch_channel_read_data(touch_chan_[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, b) == ESP_OK) {
                    bench[i] = b[0];
                }
                if (touch_channel_read_data(touch_chan_[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, v) == ESP_OK) {
                    smooth[i] = v[0];
                    if (v[0] < vmin[i]) vmin[i] = v[0];
                    if (v[0] > vmax[i]) vmax[i] = v[0];
                }
                // RAW 是未经平滑的原始读数，smooth 不动但 raw 动 = 阈值/滤波把信号吃掉了
                if (touch_channel_read_data(touch_chan_[i], TOUCH_CHAN_DATA_TYPE_RAW, w) == ESP_OK) {
                    raw[i] = w[0];
                    if (w[0] < rmin[i]) rmin[i] = w[0];
                    if (w[0] > rmax[i]) rmax[i] = w[0];
                }
            }
            if (series && trace_n < kMaxTrace) {
                for (int i = 0; i < 3; ++i) {
                    trace[i][trace_n] = smooth[i];
                }
                trace_n++;
            }
            if (sample_ms > 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        out = "{\"ok\":true,\"rounds\":";
        out += std::to_string(rounds);
        out += ",\"channels\":[";
        for (int i = 0; i < 3; ++i) {
            char buf[256];
            snprintf(buf, sizeof(buf),
                     "%s{\"ch\":%d,\"gpio\":%d,\"bench\":%lu,\"smooth\":%lu,"
                     "\"min\":%lu,\"max\":%lu,\"drop_pct\":%.1f,"
                     "\"raw\":%lu,\"raw_min\":%lu,\"raw_max\":%lu,"
                     "\"active\":%s,\"hits\":%lu}",
                     i ? "," : "", i + 1, i + 1,
                     (unsigned long)bench[i], (unsigned long)smooth[i],
                     (unsigned long)(vmin[i] == 0xFFFFFFFFu ? 0 : vmin[i]),
                     (unsigned long)vmax[i],
                     bench[i] > 0 ? (100.0 * (double)(bench[i] - (vmin[i] == 0xFFFFFFFFu ? bench[i] : vmin[i])) / (double)bench[i]) : 0.0,
                     (unsigned long)raw[i],
                     (unsigned long)(rmin[i] == 0xFFFFFFFFu ? 0 : rmin[i]),
                     (unsigned long)rmax[i],
                     touch_active_[i] ? "true" : "false",
                     (unsigned long)touch_hits_[i]);
            out += buf;
        }
        out += "]";
        if (series) {
            out += ",\"trace\":{";
            for (int i = 0; i < 3; ++i) {
                if (i) {
                    out += ",";
                }
                out += "\"ch" + std::to_string(i + 1) + "\":[";
                for (int k = 0; k < trace_n; ++k) {
                    if (k) {
                        out += ",";
                    }
                    out += std::to_string(trace[i][k]);
                }
                out += "]";
            }
            out += "}";
        }
        out += "}";
        return true;
    }

    // -----------------------------------------------------------------------
    // 硬件摸底（临时工具，摸清 FPC 触摸排线接哪儿之后可删）
    //   GET /probe?ms=3000
    // I2C 部分扫描 GPIO4/5 这条总线上所有响应的地址——屏幕触摸 IC 九成挂这里；
    // GPIO 部分看几个真正空闲的脚：先上拉读一次、再下拉读一次，两次不同说明
    // 是浮空脚（没接东西或只接了排针），相同说明有外部驱动；给了 ms 就再连续
    // 采样一段时间，报低电平次数与边沿数，按一下触摸板就能看出是哪个脚在动。
    // -----------------------------------------------------------------------
    static int ParseQueryInt(const char* query, const char* key, int fallback) {
        if (query == nullptr) return fallback;
        size_t key_len = strlen(key);
        const char* p = query;
        while ((p = strstr(p, key)) != nullptr) {
            // 要求 key 前面是开头或 '&'，避免 "ms" 命中 "samples"
            if (p == query || p[-1] == '&' || p[-1] == '?') {
                const char* value = p + key_len;
                if (*value == '=') {
                    value++;
                }
                return atoi(value);
            }
            p += key_len;
        }
        return fallback;
    }

    static const char* TouchChipHint(uint16_t addr) {
        switch (addr) {
            case 0x14: return "GT911?";
            case 0x5D: return "GT911?";
            case 0x15: return "CST816/CST716?";
            case 0x5A: return "CSTxxx?";
            case 0x38: return "FT6236/FT6336?";
            case 0x70: return "FT5x06?";
            case 0x48: return "NS2009/TSC2007?";
            case 0x41: return "STMPE610?";
            case 0x18: return "ES8311(已知)";
            case 0x30: return "OV2640(已知)";
            case 0x3C: return "GC2145(已知)";
            default:   return "";
        }
    }

    // 读某个 I2C 从机的单个 8 位寄存器；地址不存在或器件不应答就返回 false
    bool ReadReg8(uint16_t addr, uint8_t reg, uint8_t& value, bool keep_device) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = addr;
        dev_cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev) != ESP_OK) {
            return false;
        }
        esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &value, 1, 50);
        i2c_master_bus_rm_device(dev);
        (void)keep_device;
        return err == ESP_OK;
    }

    // ?dump=2A,68,76 —— 对每个 7 位地址打几个常用的 ID 寄存器。
    // 触摸控制器的识别点基本都在这些位置：CAP1203 用 FD/FE/FF，
    // CST816 用 A7，FT6x36 用 A8，MPU6050 用 75，BMP280 用 D0。
    static int ParseHexByte(const char* p) {
        int v = 0;
        for (int i = 0; i < 2 && *p; ++i) {
            char c = *p++;
            int d;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
            else break;
            v = (v << 4) | d;
        }
        return v;
    }

    // ?watch=68&from=00&to=3F&ms=8000
    // 在一段时间里反复读同一段寄存器，只把「值发生过变化」的寄存器报出来。
    // 判断某个 I2C 器件是不是触摸控制器的捷径：让它盯着，人手指按一下屏幕，
    // 谁在动就一目了然，不必先猜型号再翻手册。
    // ?map=68 —— 把某个从机的 0x00~0xFF 全部读出来（分 4 段 burst 读）。
    // 认不出型号时看这张表最快：触摸控制器的坐标寄存器静止时通常都是 0，
    // 状态寄存器会固定在 0x00/0x80 之类，一眼能认出来。
    // ?r16=14:8140&n=8 —— 用 16 位寄存器地址读一段数据。
    // GT911 / GT9271 这类触摸控制器只认 16 位寄存器地址，用 8 位读会直接失败，
    // 在扫描结果里就看不见它们。GT911 的产品 ID 在 0x8140，应该是 "911" 三个 ASCII。
    bool ReadReg16(const char* query, std::string& out) {
        const char* p = strstr(query, "r16=");
        if (p == nullptr) {
            return false;
        }
        unsigned int addr = 0;
        unsigned int reg = 0;
        if (sscanf(p + 4, "%x:%x", &addr, &reg) != 2) {
            return false;
        }
        if (addr < 0x08 || addr > 0x77) {
            return false;
        }
        int n = ParseQueryInt(query, "n", 8);
        if (n < 1) n = 1;
        if (n > 32) n = 32;

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = static_cast<uint16_t>(addr);
        dev_cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev) != ESP_OK) {
            return false;
        }
        uint8_t regbuf[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)};
        uint8_t buf[32];
        esp_err_t err = i2c_master_transmit_receive(dev, regbuf, 2, buf, n, 100);
        i2c_master_bus_rm_device(dev);

        char head[64];
        snprintf(head, sizeof(head), "{\"ok\":%s,\"addr\":\"0x%02X\",\"reg\":\"0x%04X\",\"bytes\":[",
                 err == ESP_OK ? "true" : "false", addr, reg);
        out = head;
        for (int i = 0; i < n; ++i) {
            char one[8];
            if (err == ESP_OK) {
                snprintf(one, sizeof(one), "%s\"%02X\"", i ? "," : "", buf[i]);
            } else {
                snprintf(one, sizeof(one), "%s\"--\"", i ? "," : "");
            }
            out += one;
        }
        out += "],\"ascii\":\"";
        if (err == ESP_OK) {
            for (int i = 0; i < n; ++i) {
                char c = static_cast<char>(buf[i]);
                out += (c >= 0x20 && c < 0x7F) ? c : '.';
            }
        }
        out += "\"}";
        return true;
    }

    bool MapRegisters(const char* query, std::string& out) {
        const char* p = strstr(query, "map=");
        if (p == nullptr) {
            return false;
        }
        int addr = ParseHexByte(p + 4);
        if (addr < 0x08 || addr > 0x77) {
            return false;
        }
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = static_cast<uint16_t>(addr);
        dev_cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev) != ESP_OK) {
            return false;
        }
        out = "{\"ok\":true,\"map\":\"0x";
        char hdr[8];
        snprintf(hdr, sizeof(hdr), "%02X\",", addr);
        out += hdr;
        out += "\"bytes\":[";
        bool first = true;
        int ok_count = 0;
        for (int base = 0; base < 256; base += 32) {
            uint8_t reg = static_cast<uint8_t>(base);
            uint8_t chunk[32];
            esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, chunk, sizeof(chunk), 100);
            char buf[512];
            int n = 0;
            for (int i = 0; i < 32; ++i) {
                if (err == ESP_OK) {
                    n += snprintf(buf + n, sizeof(buf) - n, "%s\"%02X\"", first ? "" : ",", chunk[i]);
                    first = false;
                    ok_count++;
                } else {
                    n += snprintf(buf + n, sizeof(buf) - n, "%s\"--\"", first ? "" : ",");
                    first = false;
                }
            }
            out += buf;
        }
        i2c_master_bus_rm_device(dev);
        out += "],\"read_ok\":";
        out += std::to_string(ok_count);
        out += "}";
        return true;
    }

    bool WatchRegisters(const char* query, std::string& out) {
        const char* p = strstr(query, "watch=");
        if (p == nullptr) {
            return false;
        }
        int addr = ParseHexByte(p + 6);
        int from = 0x00;
        int to = 0x3F;
        int watch_ms = 8000;
        const char* q = strstr(query, "from=");
        if (q) from = ParseHexByte(q + 5);
        q = strstr(query, "to=");
        if (q) to = ParseHexByte(q + 3);
        watch_ms = ParseQueryInt(query, "ms", watch_ms);
        if (addr < 0x08 || addr > 0x77 || from < 0 || to > 0xFF || to < from) {
            return false;
        }
        int len = to - from + 1;
        if (len > 64) len = 64;
        if (watch_ms < 500) watch_ms = 500;
        if (watch_ms > 20000) watch_ms = 20000;

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = static_cast<uint16_t>(addr);
        dev_cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev) != ESP_OK) {
            return false;
        }

        constexpr int kMaxDistinct = 6;
        struct Slot { uint8_t vals[kMaxDistinct]; int n; int ok; };
        static Slot slots[64];  // 只在 probe 里用，单线程访问，静态避免吃栈
        for (int i = 0; i < len; ++i) {
            slots[i].n = 0;
            slots[i].ok = 0;
        }

        uint8_t buf[64];
        int rounds = watch_ms / 100;
        for (int r = 0; r < rounds; ++r) {
            uint8_t reg = static_cast<uint8_t>(from);
            if (i2c_master_transmit_receive(dev, &reg, 1, buf, len, 100) == ESP_OK) {
                for (int i = 0; i < len; ++i) {
                    slots[i].ok++;
                    bool found = false;
                    for (int k = 0; k < slots[i].n; ++k) {
                        if (slots[i].vals[k] == buf[i]) { found = true; break; }
                    }
                    if (!found && slots[i].n < kMaxDistinct) {
                        slots[i].vals[slots[i].n++] = buf[i];
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        i2c_master_bus_rm_device(dev);

        out = "{\"ok\":true,\"addr\":\"0x";
        char hdr[8];
        snprintf(hdr, sizeof(hdr), "%02X\",", addr);
        out += hdr;
        out += "\"rounds\":";
        out += std::to_string(rounds);
        out += ",\"changed\":[";
        bool first = true;
        for (int i = 0; i < len; ++i) {
            if (slots[i].ok == 0 || slots[i].n <= 1) {
                continue;
            }
            char buf2[192];
            int n = snprintf(buf2, sizeof(buf2), "%s{\"reg\":\"0x%02X\",\"vals\":[",
                             first ? "" : ",", from + i);
            first = false;
            for (int k = 0; k < slots[i].n; ++k) {
                n += snprintf(buf2 + n, sizeof(buf2) - n, "%s\"0x%02X\"",
                              k ? "," : "", slots[i].vals[k]);
            }
            snprintf(buf2 + n, sizeof(buf2) - n, "]}");
            out += buf2;
        }
        out += "]}";
        return true;
    }

    bool DumpRegisters(const char* query, std::string& out) {
        const char* p = strstr(query, "dump=");
        if (p == nullptr) {
            return false;
        }
        p += 5;
        const uint8_t regs[] = {0x00, 0x01, 0x75, 0xA7, 0xA8, 0xD0, 0xFD, 0xFE, 0xFF};

        out = "{\"ok\":true,\"dump\":[";
        bool first_dev = true;
        while (*p) {
            while (*p == ',' || *p == ' ') ++p;
            if (!*p) break;
            int addr = ParseHexByte(p);
            while (*p && *p != ',' && *p != ' ') ++p;
            if (addr < 0x08 || addr > 0x77) continue;

            char buf[320];
            int n = snprintf(buf, sizeof(buf), "%s{\"hex\":\"0x%02X\",\"hint\":\"%s\",\"regs\":{",
                             first_dev ? "" : ",", addr, TouchChipHint(addr));
            first_dev = false;
            bool first_reg = true;
            for (uint8_t reg : regs) {
                uint8_t value = 0;
                if (!ReadReg8(addr, reg, value, false)) {
                    continue;
                }
                n += snprintf(buf + n, sizeof(buf) - n, "%s\"0x%02X\":\"0x%02X\"",
                              first_reg ? "" : ",", reg, value);
                first_reg = false;
            }
            snprintf(buf + n, sizeof(buf) - n, "}}");
            out += buf;
        }
        out += "]}";
        return true;
    }

    bool ProbeHardware(const char* query, std::string& out) {
        if (strstr(query, "scan=") != nullptr) {
            return ScanAllChannels(out);
        }
        if (ProbeTouch(query, out)) {
            return true;
        }
        if (ReadReg16(query, out)) {
            return true;
        }
        if (MapRegisters(query, out)) {
            return true;
        }
        if (WatchRegisters(query, out)) {
            return true;
        }
        if (WatchRegisters(query, out)) {
            return true;
        }
        if (DumpRegisters(query, out)) {
            return true;
        }
        int sample_ms = ParseQueryInt(query, "ms", 0);
        if (sample_ms < 0) sample_ms = 0;
        if (sample_ms > 20000) sample_ms = 20000;

        // 只发地址能收到 ACK 的地址有一堆假阳性（同一次上电两次扫描结果都不一样），
        // 所以再加一道「真能读出寄存器」的过滤，读不出来的直接不报。
        out = "{\"ok\":true,\"i2c\":[";
        bool first = true;
        for (uint16_t addr = 0x08; addr < 0x78; ++addr) {
            if (i2c_master_probe(i2c_bus_, addr, 20) != ESP_OK) {
                continue;
            }
            uint8_t probe_value = 0;
            if (!ReadReg8(addr, 0x00, probe_value, false)) {
                continue;
            }
            char buf[96];
            snprintf(buf, sizeof(buf), "%s{\"hex\":\"0x%02X\",\"hint\":\"%s\",\"r00\":\"0x%02X\"}",
                     first ? "" : ",", addr, TouchChipHint(addr), probe_value);
            first = false;
            out += buf;
        }
        out += "],\"sampled_ms\":";
        out += std::to_string(sample_ms);
        out += ",\"gpio\":[";

        // 只有 1/2/3 是真正没被占的脚：0 是 BOOT 键，4/5 是 I2C，6~18 摄像头，
        // 21/43/44/47 屏，19/20 是 USB，26~37 被 flash+8MB octal PSRAM 吃掉。
        const int pins[] = {1, 2, 3};
        bool first_pin = true;
        for (int pin : pins) {
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = 1ULL << pin;
            cfg.mode = GPIO_MODE_INPUT;
            cfg.intr_type = GPIO_INTR_DISABLE;

            cfg.pull_up_en = GPIO_PULLUP_ENABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            gpio_config(&cfg);
            vTaskDelay(pdMS_TO_TICKS(2));
            int up = gpio_get_level(static_cast<gpio_num_t>(pin));

            cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
            gpio_config(&cfg);
            vTaskDelay(pdMS_TO_TICKS(2));
            int down = gpio_get_level(static_cast<gpio_num_t>(pin));

            int low = 0;
            int edges = 0;
            if (sample_ms > 0) {
                // 采样期上拉而非浮空：开漏输出的 INT 脚浮空时会读不稳
                gpio_set_pull_mode(static_cast<gpio_num_t>(pin), GPIO_PULLUP_ONLY);
                int samples = sample_ms / 5;
                int prev = gpio_get_level(static_cast<gpio_num_t>(pin));
                for (int i = 0; i < samples; ++i) {
                    int v = gpio_get_level(static_cast<gpio_num_t>(pin));
                    if (v == 0) low++;
                    if (v != prev) {
                        edges++;
                        prev = v;
                    }
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
            }

            // 恢复浮空输入，避免残留上下拉影响外设
            gpio_set_pull_mode(static_cast<gpio_num_t>(pin), GPIO_FLOATING);

            char buf[160];
            snprintf(buf, sizeof(buf),
                     "%s{\"pin\":%d,\"up\":%d,\"down\":%d,\"floating\":%s,"
                     "\"low\":%d,\"edges\":%d}",
                     first_pin ? "" : ",", pin, up, down,
                     (up != down) ? "true" : "false", low, edges);
            first_pin = false;
            out += buf;
        }
        out += "]}";
        return true;
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_GPIO;
        io_config.dc_gpio_num = DISPLAY_DC_GPIO;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_disp_on_off(panel, true);
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        // EMOTE_STYLE: 表情动画风格。emote 组件直接驱动面板，资源来自 assets 分区里的
        // 240x240 表情包（layout 见 boards/espressif/esp-sparkbot/assets/240_240）。
        emote_display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
        display_ = emote_display_;
#else
        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
#endif
    }

    void InitializeCamera() {
        // DVP pin configuration
        static esp_cam_ctlr_dvp_pin_config_t dvp_pin_config = {
            .data_width = CAM_CTLR_DATA_WIDTH_8,
            .data_io =
                {
                    [0] = SPARKBOT_CAMERA_D0,
                    [1] = SPARKBOT_CAMERA_D1,
                    [2] = SPARKBOT_CAMERA_D2,
                    [3] = SPARKBOT_CAMERA_D3,
                    [4] = SPARKBOT_CAMERA_D4,
                    [5] = SPARKBOT_CAMERA_D5,
                    [6] = SPARKBOT_CAMERA_D6,
                    [7] = SPARKBOT_CAMERA_D7,
                },
            .vsync_io = SPARKBOT_CAMERA_VSYNC,
            .de_io = SPARKBOT_CAMERA_HSYNC,
            .pclk_io = SPARKBOT_CAMERA_PCLK,
            .xclk_io = SPARKBOT_CAMERA_XCLK,
        };

        // 复用 I2C 总线
        esp_video_init_sccb_config_t sccb_config = {
            .init_sccb = false,      // 不初始化新的 SCCB，使用现有的 I2C 总线
            .i2c_handle = i2c_bus_,  // 使用现有的 I2C 总线句柄
            .freq = 100000,          // 100kHz
        };

        // DVP configuration
        esp_video_init_dvp_config_t dvp_config = {
            .sccb_config = sccb_config,
            .reset_pin = SPARKBOT_CAMERA_RESET,
            .pwdn_pin = SPARKBOT_CAMERA_PWDN,
            .dvp_pin = dvp_pin_config,
            .xclk_freq = SPARKBOT_CAMERA_XCLK_FREQ,
        };

        // Main video configuration
        esp_video_init_config_t video_config = {
            .dvp = &dvp_config,
        };

        camera_ = new EspVideo(video_config);

        Settings settings("sparkbot", false);
        // 考虑到部分复刻使用了不可动摄像头的设计，默认启用翻转
        bool camera_flipped = static_cast<bool>(settings.GetInt("camera-flipped", 1));
        camera_->SetHMirror(camera_flipped);
        camera_->SetVFlip(camera_flipped);
    }

    /*
        ESP-SparkBot 的底座
        https://gitee.com/esp-friends/esp_sparkbot/tree/master/example/tank/c2_tracked_chassis
    */
    void InitializeEchoUart() {
        uart_config_t uart_config = {
            .baud_rate = ECHO_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;

        ESP_ERROR_CHECK(
            uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, UART_ECHO_TXD, UART_ECHO_RXD,
                                     UART_ECHO_RTS, UART_ECHO_CTS));

        SendUartMessage("w2");
    }

    void SendUartMessage(const char* command_str) {
        uint8_t len = strlen(command_str);
        uart_write_bytes(ECHO_UART_PORT_NUM, command_str, len);
        ESP_LOGI(TAG, "Sent command: %s", command_str);
    }

    // 采样不必很勤：电量本身变化慢，10 秒一次足够，还能让角标跟得上
    void PushBatteryToDisplay() {
#if CONFIG_USE_EMOTE_MESSAGE_STYLE
        // 屏幕上只显示本体这一块电池：底盘那颗留给 App，小屏塞两行会看不清
        if (emote_display_ != nullptr && battery_.IsReady()) {
            emote_display_->SetBatteryStatus(battery_.Percent(), battery_.IsCharging());
        }
#endif
    }

    void InitializeBattery() {
        // 底盘电量走 UART，与本体电池互不影响，先起起来（读不到就静默降级）
        chassis_battery_.Begin();
        if (!battery_.Begin()) {
            return;
        }
        // 先同步采一次，保证第一个 /status 和角标都有数
        battery_.Update();
        PushBatteryToDisplay();
        xTaskCreate(
            [](void* arg) {
                auto* self = static_cast<EspSparkBot*>(arg);
                while (true) {
                    vTaskDelay(pdMS_TO_TICKS(10000));
                    self->battery_.Update();
                    self->PushBatteryToDisplay();
                }
            },
            "battery", 3072, this, 2, nullptr);
    }

    // 低功耗（Wi-Fi 省电开着，也就是没有远程客户端在用）并且设备空闲时，
    // 屏上换成睡觉表情；有客户端活动、或者开始对话，就换回中立。
    // 两个条件分别来自 local_control 的省电调度和 Application 的设备状态：
    // 只看省电状态的话，对着机器人说话时它还在睡；只看设备状态的话，
    // 手机上遥控时会睡着——都不对，必须同时满足。
    void UpdateSleepFace() {
        if (display_ == nullptr) {
            return;
        }
        const bool want = LocalControlPowerSaveOn() &&
                          Application::GetInstance().GetDeviceState() == kDeviceStateIdle;
        if (want == sleep_face_on_) {
            return;
        }
        sleep_face_on_ = want;
        display_->SetEmotion(want ? "sleepy" : "neutral");
    }

    void StartSleepFaceTask() {
        xTaskCreate(
            [](void* arg) {
                auto* self = static_cast<EspSparkBot*>(arg);
                // 板级是在 Application 单例的构造函数里造出来的，这时候它的
                // GetDeviceState 还不能问，先等单例构造完再开始轮询。
                vTaskDelay(pdMS_TO_TICKS(2000));
                while (true) {
                    self->UpdateSleepFace();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            },
            "sleep_face", 4096, this, 2, nullptr);
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.battery.get_status", "获取电池电量和电压（本体与底盘各一块）",
                           PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string body;
                               if (battery_.IsReady()) {
                                   battery_.Update();
                                   body = "本体电量 " + std::to_string(battery_.Percent()) +
                                          "%，电压 " + std::to_string(battery_.CellMillivolts()) +
                                          " mV";
                               } else {
                                   body = "本体电池检测不可用";
                               }
                               if (chassis_battery_.IsValid()) {
                                   body += "；底盘电量 " +
                                           std::to_string(chassis_battery_.Percent()) + "%，电压 " +
                                           std::to_string(chassis_battery_.Millivolts()) + " mV" +
                                           (chassis_battery_.IsCharging() ? "（充电中）" : "");
                               } else {
                                   body += "；底盘电量读不到";
                               }
                               return body;
                           });

        // 定义设备的属性
        mcp_server.AddTool("self.chassis.get_light_mode", "获取灯光效果编号", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (light_mode_ < 2) {
                                   return 1;
                               } else {
                                   return light_mode_ - 2;
                               }
                           });

        mcp_server.AddTool("self.chassis.go_forward", "前进", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SendUartMessage("x0.0 y1.0");
                               return true;
                           });

        mcp_server.AddTool("self.chassis.go_back", "后退", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SendUartMessage("x0.0 y-1.0");
                               return true;
                           });

        mcp_server.AddTool("self.chassis.turn_left", "向左转", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SendUartMessage("x-1.0 y0.0");
                               return true;
                           });

        mcp_server.AddTool("self.chassis.turn_right", "向右转", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SendUartMessage("x1.0 y0.0");
                               return true;
                           });

        mcp_server.AddTool("self.chassis.dance", "跳舞", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               SendUartMessage("d1");
                               light_mode_ = LIGHT_MODE_MAX;
                               return true;
                           });

        mcp_server.AddTool("self.chassis.switch_light_mode", "打开灯光效果",
                           PropertyList({Property("light_mode", kPropertyTypeInteger, 1, 6)}),
                           [this](const PropertyList& properties) -> ToolResult {
                               char command_str[5] = {'w', 0, 0};
                               char mode =
                                   static_cast<light_mode_t>(properties["light_mode"].value<int>());

                               ESP_LOGI(TAG, "Switch Light Mode: %c", (mode + '0'));

                               if (mode >= 3 && mode <= 8) {
                                   command_str[1] = mode + '0';
                                   SendUartMessage(command_str);
                                   return true;
                               }
                               return std::unexpected("Invalid light mode");
                           });

        mcp_server.AddTool("self.camera.set_camera_flipped", "翻转摄像头图像方向", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("sparkbot", true);
                               // 考虑到部分复刻使用了不可动摄像头的设计，默认启用翻转
                               bool flipped =
                                   !static_cast<bool>(settings.GetInt("camera-flipped", 1));

                               camera_->SetHMirror(flipped);
                               camera_->SetVFlip(flipped);

                               settings.SetInt("camera-flipped", flipped ? 1 : 0);

                               return true;
                           });
    }

public:
    EspSparkBot() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeDisplay();
        InitializeButtons();
        InitializeCamera();
        InitializeEchoUart();
        InitializeBattery();
        InitializeTools();
        // LOCAL_CONTROL_PATCH: 在板级完成 UART 与摄像头初始化后，起局域网遥控服务。
        // lambda 捕获 this 以复用 private 的 SendUartMessage，无需开放可见性。
        LocalControlStart([this](const char* cmd) { SendUartMessage(cmd); },
                          [this](int& cell_mv, int& percent, bool& charging) {
                              if (!battery_.IsReady() || battery_.CellMillivolts() <= 0) {
                                  return false;
                              }
                              cell_mv = battery_.CellMillivolts();
                              percent = battery_.Percent();
                              charging = battery_.IsCharging();
                              return true;
                          },
                          [this](int& cell_mv, int& percent, bool& charging) {
                              if (!chassis_battery_.IsValid()) {
                                  return false;
                              }
                              cell_mv = chassis_battery_.Millivolts();
                              percent = chassis_battery_.Percent();
                              charging = chassis_battery_.IsCharging();
                              return true;
                          },
                          [this](const char* query, std::string& json) {
                              return ProbeHardware(query, json);
                          });

        StartSleepFaceTask();  // 低功耗时打睡脸，见 UpdateSleepFace

        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static SparkBotEs8311AudioCodec audio_codec(
            i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN, AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override { return camera_; }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        if (!battery_.IsReady()) {
            return false;
        }
        const bool is_charging = battery_.IsCharging();
        charging = is_charging;
        discharging = !is_charging;
        level = battery_.Percent();
        return true;
    }
};

DECLARE_BOARD(EspSparkBot);
