#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
载重雷达（空车/载货车）检测 —— 麒麟平台重构版

反向工程自原厂 Windows 程序（detectempty.dll / detectloaded.dll / stopdetect.dll /
jt128.dll / calibrate_test.exe）。厂商已注销，故本项目以「功能等价 + 可现场调试」
为目标重建，而非逐指令还原。

包内模块
    配置与数据
        settings.py      配置加载（兼容原厂 config.json / devices.json，另加 channels.json）
        pointcloud.py    点云容器、.xyz 二进制格式（4+N×14）、ROI、角度标定表校验、导出

    算法
        geometry.py      平面拟合 / 内点收紧 / 航向矫正原语
        detect.py        六面拟合 + 11 步判定链 + 停稳检测 + 体积（对应三个 DLL）

    雷达
        radar_packet.py  128 线 UDP 报文解码（对应 jt128.dll）
        radar_source.py  数据源：live / replay / simulate / synthetic
        radar_sim.py     仿真雷达（按几何真值反算报文，用于无硬件闭环验证）
        radar_probe.py   抓包探测：反推距离换算模式 / 角度标定 / z 方向

    业务
        plate.py         车牌读取（走外部相机 HTTP，配置驱动，非雷达）
        service.py       多通道编排 + 检测流水线 + 事件总线
        web/             浏览器看板（标准库 HTTP + SSE，前端零依赖）

入口在 cli/，自检在 tools/selftest.py，单测在 tests/。
用法与现场流程见 README.md 与 docs/。
"""
__version__ = "1.0.0"
