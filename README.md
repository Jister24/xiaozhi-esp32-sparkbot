# esp-sparkbot 定制固件

桌面轮式机器人的固件，基于 [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32)，
针对 `espressif/esp-sparkbot` 板子做了摄像头、局域网遥控、电量检测和表情改造。

- 基线：`xiaozhi-esp32` @ `c435f99`
- 芯片：ESP32-S3，240×240 圆屏
- 上游说明见 [README.upstream.md](README.upstream.md)（中文版 `README_zh.md`）

本仓不携带上游历史，第一笔 commit 是上游快照 + 本地改动收拢，之后是本项目自己的演进。
`origin` 仍指向上游，随时可以 `git diff` 比对。

## 相对上游改了什么

| 功能 | 主要位置 | 说明 |
|---|---|---|
| 摄像头 | `main/boards/common/esp_video.cc`、`camera.h` | 板载实际是 **GC2145**（启动日志 PID=0x2145），上游只启了 OV2640 |
| 图像方向 | 同上 | 摄像头装反了，整帧软件旋转 180°（S3 没有 PPA，V4L2 flip 没实现） |
| 局域网遥控 | `main/boards/espressif/esp-sparkbot/local_control.*` | HTTP 控制移动/云台，带图传取帧，不依赖云端 |
| 拍照回显 | `main/display/emote_display.cc` | 拍完把这一帧铺满屏幕 5 秒，再让回表情 |
| 本体电量 | `esp_sparkbot_board.cc`、`config.h` | GPIO14（ADC2_CH3）分压检测，1:1 等值电阻，电芯 = 引脚 × 2 |
| 底盘电量 | `chassis_battery.*` | 底盘在 C2 侧，只能走板间 UART 问，5 秒一帧 |
| 电量角标 | `emote_display.cc` | 表情屏右上角常驻，带百分比数字 |
| Wi-Fi 省电 | `local_control.cc` | 有远程客户端在用就切 `WIFI_PS_NONE`，静默 30 秒回退 `MIN_MODEM` |
| 睡觉表情 | `emote_display.cc`、`assets/240_240/` | 省电且空闲时换睡脸，并飘一个 zzz 叠加层 |
| MCP 拍照 | `main/mcp_server.cc` | emote 显示模式下不该关掉拍照工具 |
| MQTT 组件 | `local_components/mqtt/` | 上游组件包含测试私钥，用去测试的本地副本 + `override_path` 覆盖 |

表情素材 21 个名字映射到 10 个 `.eaf`，其中 `eye_anim` 是 `mirror:auto`——一张 125×160
的图会镜像成一双眼睛，所以 zzz 只能做成独立的 gfx 对象叠上去，画不进同一张素材。

## 编译

需要 **ESP-IDF v6.0.1 以上**（本项目在 `release/v6.1` 上验证）。

```bash
git clone git@github.com:Jister24/xiaozhi-esp32-sparkbot.git
cd xiaozhi-esp32-sparkbot
. $IDF_PATH/export.sh

# 标准方式
python scripts/build.py espressif/esp-sparkbot

# 或直接走 idf.py（目标必须显式给，否则只会编通用代码，不编板级）
idf.py -DIDF_TARGET=esp32s3 -DBOARD_NAME=esp-sparkbot build
idf.py -p /dev/cu.usbmodem* flash monitor
```

## 踩过的坑

- **必须显式 `-DIDF_TARGET=esp32s3`**。缺了它 `idf.py` 也会报「编译成功」，但板级 `.obj`
  一个都没编，烧进去还是老固件。
- `CONFIG_MMAP_FILE_NAME_LENGTH` 必须 ≥ 32（默认 16）。否则 `font_puhui_common_16_4.bin`
  这种长名字会被截断，字体静默加载失败、不报错。已在板级 `config.json` 的
  `sdkconfig_append` 里固定。
- `emote.json` 放在**分辨率目录** `assets/240_240/`，不是 emoji 素材目录；放错则
  `SetEmotion` 全部 not found 但不报错。
- 换了表情素材要先删 `build/expression_assets.bin`，那个打包产物不随素材重建。
- `.eaf` 末帧可能是 `_C` 控制帧，解码时要跳过。

## 硬件要点

- 本体电池分压脚 = **GPIO14（ADC2_CH3）**，与地之间两个等值电阻。实测插 USB 4.19 V /
  拔线 4.03 V。I2C 上没有电量计。S3 的 ADC2 与 Wi-Fi 不冲突，可随时采。
- 底盘电池在 C2 一侧（ADC1_CH3），只喂 RGB 灯不上报。C2 **没有原生 USB**，
  要改它的固件得开壳找 TX/RX/EN/BOOT 焊盘。
- 拔掉 USB 供电会触发整机重启（硬件供电切换），别误判成崩溃。
- 板载摄像头 GC2145 物理装反，靠软件旋转纠正。

## 相关仓库

- [ESP_SPARKBOT](https://github.com/Jister24/ESP_SPARKBOT) —— Android 遥控 App、
  以及给现成上游 checkout 打补丁的 `apply.py`

## 许可

继承上游 MIT，见 [LICENSE](LICENSE)。上游版权归 Shenzhen Xinzhi Future Technology Co., Ltd.。
