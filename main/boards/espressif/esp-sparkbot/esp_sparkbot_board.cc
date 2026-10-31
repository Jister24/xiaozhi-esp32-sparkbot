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

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/uart.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <cstring>

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
                          });

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
