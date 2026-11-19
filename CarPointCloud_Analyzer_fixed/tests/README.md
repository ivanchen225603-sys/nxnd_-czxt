# Analyzer 测试

在 Analyzer 项目目录运行，使用项目 `.venv`。测试覆盖当前算法、接口与发布入口；已退役的历史基准/网格和旧测长算法不再保留对应测试。

| 目录 | 用途 | 项数 | 运行时机 |
| --- | --- | ---: | --- |
| `unit/` | 当前算法、真实故障回归、HTTP/载荷、文件与事务契约 | 945 | 日常修改，默认收集 |
| `release/` | 麒麟打包工具、模板、文件筛选及平台格式 | 5 | 改打包工具、交付前 |
| 全部分组 | 上述全部 | 950 | 完整回归 |

```powershell
# 日常回归：pytest.ini 的 testpaths=tests/unit
.\.venv\Scripts\python.exe -m pytest -q

# 完整回归，包括发布检查
.\.venv\Scripts\python.exe -m pytest -q tests

# 单独验证打包
.\.venv\Scripts\python.exe -m pytest -q tests/release

# 完整覆盖率
.\.venv\Scripts\python.exe -m pytest tests --cov=carpointcloud_analyzer
```

分组通过目录选择，不使用新增 skip、xfail 或静默排除标记。`pytest tests` 始终收集全部分组。

## 2026-11-03 第二轮精简：删除防御与兼容测试

完整集合 1054 → 950（含 release 5）。删除 `tests/compatibility/`（旧导入/pickle）、`test_pcd_validation.py`（PCD 头部与字节校验）、PCD 大小/点数上限、服务路径/相机/单位/批量上限/空白 workflow 校验、通用 500 包装、`Config.validate()` 规则、入库载荷范围与分段总和上限、`actualRibCount` 格式、appsettings 格式/目录位置校验、档案根目录兼容参数等用例。它们只断言已删除的防御。

调整而非删除：测试替身外参补齐真实 `Extrinsic` 字段（服务不再 `getattr` 探测）；手动车牌用例改为替换 `load_trusted_extrinsic`（外参现先于点云加载）；“必须提供 extrinsicPath” 改为经 `analyze_pair_sync` 验证。未放宽任何数值断言，未新增 skip/xfail。

验证：`python -m pytest -q tests` **950 passed**；Ruff、`git diff --check` 通过。测试源码 **13300 → 12850 行**。

## 2026-11-03 删除退役能力对应的测试（历史）

在上一轮 1130 项基础上，删除或合并 71 项，仅剩 1059 项。删除旧基准仓库、网格序列化/损坏/尺寸限制、旧侧壁和左上沿测长用例，以及失效容差策略带来的重复参数组合。

旧 `test_detector_and_grids.py` 中仍适用于当前流程的两个用例移到 `test_load_status_pipeline.py`。拉筋投影、端部补缝、当前内端面测长及真实故障数值回归保留。测试未放宽数值断言或增加跳过标记。源码变化与依赖依据见 [架构说明](../docs/architecture-20261103.md)。

验证：完整 `python -m pytest -q tests` **1059 passed**，0 失败、0 跳过，1 条既有 FastAPI/Starlette 弃用提示；简化夹具与命令帮助后，对应流程/补缝/CLI/HTTP **53 passed**。主包、测试、工具、脚本 Ruff 及 Analyzer `git diff --check` 通过。测试源码 **14067 → 13300 行**，主包 **17947 → 17049 行**。本次没有运行真实雷达/DM8 验收，没有部署、重启、提交或推送。

## 2026-11-03 首轮测试精简记录（历史）

精简前 1228 项，精简后共 1130 项，净减 98 项。默认日常回归 1115 项；另外 15 项只是移动，没有删除。

- 两个拉筋文件由 252 项减到 160 项。孤立支架、偏轴支架、单格扰动、持续断差的全排列改为代表组合；保留原始每个参数取值、左右/镜像、正负偏移、坡度及补点开关。原数值断言和容差不变；不再穷举全部交叉组合。
- 删除 24 项向内部精定位结果人为填入错误类型的用例。保留真实非有限点、缺测、严格 JSON 序列化、杆件错配、端点缺失和分段闭合检查。
- 配置用例改为公开字段的实际非法值，取消遍历私有规则表的测试和辅助函数。91 度底板角、95 度竖直偏差等明确边界仍在公开参数测试中验证。
- 删除仅检查已消失私有输出工厂/配置字段的用例。现有实际文件输出和 HTTP 载荷测试继续覆盖不生成 PNG/JSON 的正常行为。
- 融合朝向测试由搜索 Python 源码字符串，改为同一标定/点云在“已验收/未验收”状态下的实际成功/拒绝行为。保留真实建系，仅隔离无关的提车和体素融合。
- 将单个 HTTP 输入校验用例并入 `test_service.py`；移除历史补丁名文件。将“人工审核”旧测试名改为它实际验证的 `inferredGeometry`，没有新增审核规则。
- 发布配置模板不再注入 Worker 已删除的恢复队列、完整场证明等旧键，改用当前 `AnalysisTaskDirectory`；永久点云保护目录明确使用 Linux 路径。未改 Worker 源码或真实配置。

验证结果：`python -m pytest -q tests` **1130 passed**，0 失败、0 跳过，1 条既有 FastAPI/Starlette 测试客户端弃用提示。
默认命令收集确认 **1115** 项，完整运行另外包含兼容 **10** 项、发布 **5** 项。
Ruff（主包/测试/工具/脚本）和 `git diff --check` 通过；已完成一次 simplify 复查。

修改前后对照：Analyzer 算法包 116 个 Python 源文件 SHA-256 均未改变；已有 Worker/WebApi 等工作树文件内容保持。
仅改测试组织、文档与上述打包模板。没有生成交付包、连接真实数据库/雷达、部署、重启或提交推送。
打包测试为本地模板与文件检查，不等于已在麒麟现场构建或部署。
