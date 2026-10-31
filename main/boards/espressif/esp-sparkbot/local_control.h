#pragma once

#include <functional>

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
void LocalControlStart(std::function<void(const char*)> send_command);
