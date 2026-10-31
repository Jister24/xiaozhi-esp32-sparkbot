#include "application.h"
#include "button.h"
#include "codecs/es8311_audio_codec.h"
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

// BATTERY_PROBE: 临时探针。主体板上电池（2000mAh）是否有分压接到 S3 的 ADC，
// 官方原理图（立创 oshwhub）抓不到、官方固件也没有读电池的代码，只能用实测判定。
// 主体已被占用的 GPIO 见 config.h，空闲的只有 1/2/3/14/19/20，其中 19/20 是 USB，
// 14 属 ADC2（Wi-Fi 开启时不可用），故只扫 GPIO1/2/3（ADC1 的 CH0/CH1/CH2）。
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static void BatteryProbeTask(void*) {
    adc_oneshot_unit_handle_t unit = nullptr;
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &unit) != ESP_OK) {
        ESP_LOGE("BATPROBE", "adc unit 创建失败");
        vTaskDelete(nullptr);
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ADC_ATTEN_DB_12;       // 量程约 0~3.1V
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    const adc_channel_t channels[] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2};
    const char* names[] = {"GPIO1", "GPIO2", "GPIO3"};
    for (int i = 0; i < 3; i++) {
        adc_oneshot_config_channel(unit, channels[i], &chan_cfg);
    }

    adc_cali_handle_t cali = nullptr;
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = ADC_UNIT_1;
    cali_cfg.atten = ADC_ATTEN_DB_12;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    bool cali_ok = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK);

    ESP_LOGW("BATPROBE", "开始扫描，cali=%d", (int)cali_ok);
    while (true) {
        for (int i = 0; i < 3; i++) {
            int lo = 4095, hi = 0, sum = 0;
            for (int n = 0; n < 16; n++) {
                int raw = 0;
                if (adc_oneshot_read(unit, channels[i], &raw) != ESP_OK) continue;
                sum += raw;
                if (raw < lo) lo = raw;
                if (raw > hi) hi = raw;
            }
            int avg = sum / 16;
            int mv = 0;
            if (cali_ok) adc_cali_raw_to_voltage(cali, avg, &mv);
            // 悬空引脚：读数接近 0 或来回漂移（max-min 很大）；接了分压：稳定在 1.0~2.2V
            ESP_LOGW("BATPROBE", "%s raw=%d (min=%d max=%d 抖动=%d) -> %d mV",
                     names[i], avg, lo, hi, hi - lo, mv);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void BatteryProbeStart() {
    xTaskCreate(BatteryProbeTask, "bat_probe", 3072, nullptr, 3, nullptr);
}
// ===== BATTERY_PROBE END =====

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
        display_ = new emote::EmoteDisplay(panel, panel_io, DISPLAY_WIDTH, DISPLAY_HEIGHT);
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

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
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
        InitializeTools();
        // LOCAL_CONTROL_PATCH: 在板级完成 UART 与摄像头初始化后，起局域网遥控服务。
        // lambda 捕获 this 以复用 private 的 SendUartMessage，无需开放可见性。
        LocalControlStart([this](const char* cmd) { SendUartMessage(cmd); });
        GetBacklight()->RestoreBrightness();
        BatteryProbeStart();  // BATTERY_PROBE
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
};

DECLARE_BOARD(EspSparkBot);
