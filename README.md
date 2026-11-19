# 车辆点云尺寸与煤量测量

本项目从车辆激光雷达点云测量车厢尺寸和煤炭体积，并提供查询接口。

| 目录 | 职责 |
| --- | --- |
| `CarPointCloudLadarWorker` | 双雷达采集和配对 |
| `CarPointCloud_Analyzer_fixed` | 点云融合、车辆尺寸、底板剖面和拉筋测量 |
| `CarPointCloudLoadRadar` | 载货检测和煤量计算 |
| `CarPointCloudWebApi` | Web 管理界面、DM8 数据访问和查询 API |

两个查询接口为 `GET /api/v1/vehicleDimensions` 和 `GET /api/v1/coalVolumes`。
高低板的几何口径见[底板剖面说明](CarPointCloud_Analyzer_fixed/docs/floor_profile_spec.md)。

## 本地运行

- Worker 和 WebApi 使用 .NET 8。Worker 的 `LadarSdkRoot` 指向单独取得的厂家 SDK，目录结构见 `CarPointCloudLadarWorker/CarPointCloudLadarWorker.csproj`。
- Analyzer 和 LoadRadar 的 Python 依赖在各项目的 `requirements.txt`。LoadRadar 可在项目目录运行 `python cli/run.py --demo`。
- 从各组件的示例配置开始，现场使用时填写自己的地址、标定和密钥。已有 DM8 实例需按当前版本应用项目目录中的迁移 SQL。
- 现场点云、回归数据包和厂家 SDK 由使用方单独提供。无需这些输入的合成用例可直接运行。
