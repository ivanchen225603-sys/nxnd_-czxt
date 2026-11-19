-- ============================================================
-- 2026-04-01: V2 API 文档适配 — 移除 VehicleDimensionRecord 中废弃的高低板标量列
--
-- 背景：
--   V2 文档将「高低板离地高度」从两个独立的标量字段 (最高/最低)
--   调整为 highLowPlates[] 数组（每条对应一块高低板的上沿离地、下沿离地），
--   数据已迁移到 CarPointCloud.VehicleDimensionHighLowPlate 子表。
--
-- 影响：
--   丢弃 CarPointCloud.VehicleDimensionRecord 的
--     HighLowPlateHighestToGround、HighLowPlateLowestToGround
--   两列（不再写入、不再读取、DTO 中已无对应字段）。
-- ============================================================

SET SCHEMA CarPointCloud;

ALTER TABLE VehicleDimensionRecord DROP COLUMN HighLowPlateHighestToGround;
ALTER TABLE VehicleDimensionRecord DROP COLUMN HighLowPlateLowestToGround;

COMMIT;
