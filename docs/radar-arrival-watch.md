# 按通道记录疑似漏登

记录器周期性读取 LoadRadar 的状态接口，在独立 SQLite 文件记录观测。现场可按通道对照到磅、乙方抓拍和视频，核实漏登。

| 报告原因 | 记录条件 |
| --- | --- |
| `below_arrival_threshold` | 通道空闲、没有分配车次，并连续采样到三个不同的可信帧。车辆点数达到记录下限，仍低于该通道入场阈值。若随后直接生成车次，记录会携带 `passage_id`。 |
| `observation_gap` | 通道处于隔离，距最后完整帧已超过五秒。 |
| `quarantined_occupied` | 通道处于隔离，可信帧的车辆点数达到入场阈值。 |
| `restart_occupied` | LoadRadar 重启或启动期间，通道仍处于隔离，可信帧的车辆点数达到入场阈值。 |

在运行 LoadRadar 的主机上，从项目根目录执行。下例的 15,000 点应按各通道实际入场阈值填写。默认轮询间隔为两秒，记录低点数的下限为 1,500 点。

```sh
python3 tools/radar_arrival_watch.py --db artifacts/radar-arrival-watch.sqlite run \
  --threshold CH2 15000 --threshold CH3 15000 --threshold CH4 15000
```

在另一终端读取报告。省略 `--channel` 可查看全部已记录通道。

```sh
python3 tools/radar_arrival_watch.py --db artifacts/radar-arrival-watch.sqlite report --channel CH2
```

第一行给出最后一次轮询、成功读取时间和最近错误。出现连接错误时，把该时段标为缺少实时观测。后续行给出每段事件的起止时间、点数峰值、入场阈值、末次完整帧、采集实例和车次标识。短于轮询间隔的变化可能未被采到。是否确有车辆漏登，以现场视频、抓拍和到磅记录交叉核对为准。
