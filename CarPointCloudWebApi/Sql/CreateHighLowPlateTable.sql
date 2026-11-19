-- 车辆尺寸高低板明细表
-- 用于存储车辆高低板信息（达梦 DM8 版本）
-- 对应 API 文档 v2.0 变更：highLowPlateHighestToGround / highLowPlateLowestToGround 标量字段改为 highLowPlates 数组结构

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEDIMENSIONHIGHLOWPLATE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleDimensionHighLowPlate (
        Id BIGINT IDENTITY(1,1) PRIMARY KEY,
        DimensionRecordId BIGINT NOT NULL,
        PlateId INT NOT NULL,
        HighestToGround DECIMAL(10,1),
        LowestToGround DECIMAL(10,1)
    )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VDHLP_DIMENSIONRECORDID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VDHLP_DimensionRecordId ON VehicleDimensionHighLowPlate(DimensionRecordId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VDHLP_DIMENSIONRECORDID_PLATEID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX IX_VDHLP_DimensionRecordId_PlateId ON VehicleDimensionHighLowPlate(DimensionRecordId, PlateId)';
  END IF;
END;
/

COMMIT;
