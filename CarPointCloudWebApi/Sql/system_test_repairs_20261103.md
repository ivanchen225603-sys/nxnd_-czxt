# 2026-11-03 系统测试修复的数据库说明

本次修复不需要新增迁移。此前 DM8 回归失败是跨车道测试更换 PassageKey 后漏清发布关联，修复位于测试清理代码。业务外键、历史数据和生产表结构保持原状。

本轮取消体积质量门槛、尺寸按已算字段直接保存，也不需要 ALTER 或 UPDATE 脚本。空洞比例、范围宽度和残帧标记仍使用原字段保存；旧 `low_confidence` 测量可由自动发布器按当前规则处理，不批量改写历史载荷。尺寸缺项继续使用现有可空字段，不新增人工审核。`LoadRadar:VolumeBoundSpreadMax` 保留配置兼容性，但不再影响启动或发布。

先使用同目录 `preflight_system_test_repairs_20261103.sql` 核查当前连接的 Schema。该脚本只有 SELECT，不会切换 Schema 或写数据。本次只在自动创建的本机隔离测试 Schema 验证，没有连接 101 或修改现有 CarPointCloud。

若以后发布前核查发现缺少既有功能所需结构，按实际缺项选用以下已有脚本，不要整套重复执行：

| 缺项 | 已有增量脚本 | 执行条件 |
|---|---|---|
| EnterId、两个测量时间、ActualRibCount、MaximumWidthCm、LengthCm | `partner_contract_v4_20261012.sql` | 逐项核查，只执行缺失字段的 ALTER；原脚本不是重复执行安全的 |
| PlateAcquisitionJob.CandidateJson | `async_plate_checkpoint_20261022.sql` | 已含存在性判断 |
| LoadRadar 基础表 | `load_radar_20260917.sql` | 仅旧环境缺少该模块时使用 |
| 来车登记表及关联 | `load_radar_passage_start_20260918.sql` | 基础表已存在且此增量尚未执行 |
| 正式发布关联表 | `load_radar_publication_20260918.sql` | 前面结构已存在且此增量尚未执行 |
| LoadRadarVolumeRevision 修订表 | `load_radar_volume_revision_20261102.sql` | 测量表、发布关联及 CoalVolumeRecord 已存在，修订表尚不存在 |

增量执行前按现场流程备份并确认目标 Schema。`Database/dm8_fresh_init.sql` 是新建空库脚本，不能用于升级已有业务库。部分历史脚本含 `SET SCHEMA CarPointCloud`，在测试库执行时必须替换为本次隔离 Schema；本轮测试建库工具只对自己的 `CPT_...` Schema 做此替换。
