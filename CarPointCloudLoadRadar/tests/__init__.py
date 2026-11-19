# -*- coding: utf-8 -*-
"""
tests — 单元测试

跑法（不需要 pytest，标准库 unittest 即可；装了 pytest 也能直接 pytest tests/）：

    python3 -m unittest discover -s tests -v
    # 或
    python3 tests/run_all.py

覆盖范围
    test_settings.py    配置解析、路径解析、ROI 归一化、字段完备性
    test_pointcloud.py  Frame 基本量、.xyz 往返、AngleCache 校验、PLY/CSV 导出
    test_geometry.py    平面拟合原语（PCA / 内点收紧 / 朝向矫正）
    test_packet.py      报文编解码闭环（仿真真值 → 编码 → 解码 → 比对）
    test_detect.py      ★ 判定链：10 份历史样本逐条比对原版 reason.txt
    test_service.py     事件总线、会话状态机、场景代次切换
"""
