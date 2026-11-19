-- 合同 v4 增量字段。部署新 WebApi / Analyzer 前由 DBA 在业务 Schema 手工执行。
-- 不回填历史数据，不修改 HighestToGround 的原最高点高度，不自动执行。
-- 重复执行前检查 USER_TAB_COLUMNS；这些 ALTER 只执行一次。
ALTER TABLE LicensePlateRecord ADD EnterId CLOB;
/
ALTER TABLE CoalVolumeRecord ADD MeasurementStartedAt DATETIME(3);
/
ALTER TABLE CoalVolumeRecord ADD MeasurementEndedAt DATETIME(3);
/
ALTER TABLE VehicleDimensionRecord ADD ActualRibCount INT;
/
ALTER TABLE VehicleDimensionRecord ADD MaximumWidthCm DECIMAL(10,1);
/
ALTER TABLE VehicleDimensionHighLowPlate ADD LengthCm DECIMAL(10,1);
/
