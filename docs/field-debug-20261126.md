# 11 月 27 日内网联调：CH4 空车尺寸与车牌归属

先走诊断采集，再验收对外尺寸。CH4 停稳立即启动双雷达，乙方识牌独立按到磅后约 10 秒首查，每次未确认再等 5 秒，含首查总共最多 3 次。原始 PCD 可用于核对，即使本车不能确认车牌或静止也会按现场保留策略留存。

## 现场升级

1. 记录四个服务的运行路径、版本、原有配置、外参、数据目录与 DM8 备份结果。暂停自动派发，等当前车次结束。以程序包的 SHA256SUMS 验证交付文件。
2. 在既有业务库使用有权执行 DDL 的账户依次运行包内 `database/partner_plate_observations_20261126.sql` 和 `database/static_capture_evidence_20261126.sql`。两者为增量、可重复执行；保留现场数据。新库初始化脚本仅适用于空库。部署前核实 `CarPointCloud` 为实际业务模式。
3. 确认 `PartnerPlateObservation`、`StaticCaptureEvidence`、`IX_StaticCapture_Due` 及三个启用的触发器 `TR_StaticCaptureClaim`、`TR_StaticDimensionWrite`、`TR_StaticProfileWrite` 均存在。新 CH4 自动派发受静态确认约束；历史数据和独立人工采集沿用原流程。回退应用时若保留数据库保护，旧版自动采集只能留作诊断，不能写入未经确认的尺寸或覆盖档案。
4. 合并配置后按 WebApi、Analyzer、Worker、LoadRadar 顺序启动。先核对服务运行的是新二进制与新 Python 源码，再接收 CH4 停稳事件。沿用现场共享 Python 环境、DM8 驱动、标定、待办目录和既有站点凭据。

| 组件 | 需要核实的有效配置 |
|---|---|
| WebApi | `LoadRadar:Enabled=true`、`DimensionScanChannelId=CH4`、`DimensionScanWindowSeconds=10`。乙方 `Integration:PartnerBaseUrl=http://partner-host:8520`，路径 `/api/v1/licensePlateRecords`，现场联调时启用外呼、关闭模拟识牌。 |
| Worker | `LadarCapture:EnableApiRequests=true`，`WebApiUrl` 使用同机回环地址和正确端口，`AppSecret` 与现场 WebApi 一致。默认 `MaxConcurrentAnalyses=1`、`AnalysisRetryCount=2`；`AnalysisTaskDirectory` 空值沿用 `CaptureDirectory/analysis-tasks`。已显式配置且有效的现场目录继续使用。保留现场 SDK 回调保护值。 |
| LoadRadar | CH4 启用真实采集和 WebApi 节点认证。核对 `channels.CH4.standstill_observation_age_bound_ms`，单位毫秒。 |
| Analyzer | 核对实际外参、DM8 和 `CARPOINT_EMPTY_PROFILE_ROOT`。用服务账户核实目录可写、重启后仍挂载同一位置。 |

## 先确定时间依据

每台双雷达都要分别记录并验证 `VerifiedMaxScanDurationMs`（一份完整场最长采集时长）、`VerifiedMaxCallbackDelayMs`（场结束到 Worker 回调的最大延迟）、`VerifiedClockUncertaintyMs`（Worker 与 LoadRadar 时间换算的误差上界）。它们位于 `LadarCapture:Devices` 的各设备项。LoadRadar 的 CH4 上界涵盖位置观测到接收整帧的最大年龄。用厂商帧时标、现场同步参考或经核实的设备界限佐证这些值；保留验证时间、设备和依据。回调周期的中位数不能充当最长扫描时长。

101 现场 Worker 的回调保护为 15000 毫秒，停稳采集窗口为 10 秒。r2 中，WebApi 为每个自动停稳派发签名，绑定车次、停稳段和截止时间。Worker 只对经签名验证的同机自动派发允许提前保存候选场，仍按现场配置丢弃第一个边界场。人工采集和旧指令沿用原回调保护。

无实测依据的参数保留 `null`。此模式可调通来车、识牌、诊断点云落盘和证据传输。诊断文件记录此次收到的回调，不能据此确认点云属于本车的完整静止期。两路完整扫描的保守时间区间均落在同一段已观测的连续静止期间，才会送入 Analyzer。内部时间保留亚秒精度，对外业务时间沿用 `yyyy-MM-dd HH:mm:ss`。

## 按一辆车核对

用 `nodeId / channelId / passageId` 找到 `workflowId`，再按 `segmentId`、`pairId` 核对 A/B 点云、静止观测和尺寸。LoadRadar 向 `POST /api/v1/load-radar-agent/passages/standstill-state` 发送观测；Worker 在同机签名调用 `POST /api/v1/vehicleWorkflows/{workflowId}/staticEvidence`。两条都是内部路由。

```sql
SELECT WorkflowId, PairId, Decision, EvidenceJson
FROM StaticCaptureEvidence WHERE WorkflowId = ?;

SELECT SessionId, CameraIdentifier, ArrivalTime, PlateStatus, LicensePlate
FROM VehicleArrival WHERE SessionId = ?;

SELECT Id, WorkflowId, PairId, LicensePlate, GetTime
FROM VehicleDimensionRecord WHERE WorkflowId = ?;
```

- `pending` 表示正在等两路证据。到采集截止后再过 30 秒仍未取得充分证据，会收口为 `insufficient`。缺少设备上界可直接结束。确认运动跨界为 `rejected`。`confirmed` 才可送 Analyzer，尺寸对外可见还需要有效车牌和正式入库结果。
- 识牌日志按通道和车次对照我方 `arrivalAt`、乙方 `captureAt`、我方 `firstSeenAt`、最终 `decisionAt` 与 `enterId`。首次看到时间只代表本系统收到过该抓拍。冷启动期间邻车未识别、归属不清时可能连续返回未知，核对 `prior_passage_capture` 等排除原因及通道历史样本。
- 乙方联调读 `GET http://webapi-host:5031/api/v1/vehicleDimensions?limit=10`。同一轮也可读 `GET http://webapi-host:5031/api/v1/coalVolumes?limit=10`。按合同在请求头放 `Timestamp` 和 `Signature`。

## 验收记录

### 按通道确认车牌归属

同通道按乙方实际抓拍时间从早到晚核对过车顺序，允许我方漏登。已经确认的来车和抓拍配对可以分隔前后的车次。若某条抓拍在已确认车辆之后，它就不能归属该车辆之前的失败车次。同一秒、顺序矛盾、一个车次仍有多个可能的抓拍，或一个抓拍仍可属于多辆车，均保留待确认。数组位置、车牌重复与 enterID 数字大小均不提供先后证据。

日志字段 `timeReference` 表明本次时间依据。`partner_history` 使用实际成功绑定的历史，`verified_passages` 使用现场独立核验的历史，`verified_bounds` 使用该点位已核实的完整偏移范围，`cold` 表示尚无可信时间参考。没有已确认顺序或时间依据时，单条近时抓拍仍可能是前车迟报，或是雷达漏登的车辆。

有独立设备或乙方测量依据时，可设置 `PlateAcquisition:VerifiedCaptureOffsetBounds:<CameraIdentifier>:MinimumSeconds` 和 `MaximumSeconds`。取值为实际抓拍时间减去我方实际到磅时间，须覆盖允许的时钟差、触发延迟、迟抓拍及秒级精度。该范围优先用于同点位每辆车的抓拍归属，车辆间隔不限。未知范围保持空配置。

### 识牌冷启动的人工校准

WebApi 配置 `PlateAcquisition:VerifiedCalibrationSamples` 默认为空数组。需要校准时，每条样本填写一次经现场独立核实的真实过磅和抓拍配对。将人工核对凭据留在现场联调记录中，避免把系统待确认的匹配再当成核对依据。

| 字段 | 填写内容 |
|---|---|
| `CameraIdentifier`、`WorkflowId` | 我方该次过磅的点位标识、车次标识 |
| `EnterId` | 乙方原始抓拍编号，按字符串录入并保留前导零 |
| `ArrivalTime` | 我方记录的实际到磅时间 |
| `CaptureTime` | 乙方对应该编号的原始抓拍时间 |
| `VerifiedAt` | 现场完成该配对核实的我方本地时间 |

三个时间字段均用 `yyyy-MM-dd HH:mm:ss`。一个点位取本车到磅前已核验的最近五个有效车次，沿用自动校准的规则，至少三个不同车次和三个不同抓拍编号形成唯一的一致组才生效。相同样本重复登记只计一次，同一车次或抓拍的相互矛盾记录剔除。人工样本只在自动历史不足以校准时单独使用，避免同一车次被两份记录重复计数。现场重新校时或改变抓拍触发时机后，应清理并重新核实该点位的样本。

已核实为其他车的抓拍编号保留给原车次。确认某前车已经识别时，还须匹配其实际到磅秒数；人工记录中车次或抓拍编号相互冲突时，该前车仍按归属未明处理。

### 现场验收场景

| 现场或受控回放 | 应看到的结果 |
|---|---|
| 第 8 秒完整收齐 A/B，第 10 秒开始移动，车牌稍后返回 | 两份 PCD 属于本车。静态确认、尺寸入库与车牌回填可分别核对，乙方最终只能读到这一车的有效记录。 |
| 第 7 秒已移动，第 8 秒收到跨界 B 帧，延迟传送失稳消息 | 保留 PCD 和原因。即使乙方已正确识牌，也没有该次成功尺寸或有效空车档案。 |
| 前车未识别，两车 0/20 秒到磅，后车首查只见第 16 或第 21 秒的前车抓拍 | 后车不能占用这条抓拍，校准样本不增加。另测同号新 enterID 和间隔一小时后可唯一归属的单条记录，应能绑定。 |
| 成功空车车牌迟到，待办写入后重启 Analyzer，再完成识牌 | 保留一条尺寸；按原 workflow 补登一份有效空车档案。前一车档案持续可用直到新一份真正提交。 |
