-- ============================================================
-- 达梦 DM8 数据库初始化脚本
-- 由 SQL Server 版 Production_InitDatabase.sql 转换而来
-- 假定在目标 schema 中执行（无 USE 语句、无 DB_ID()）
-- ============================================================

-- ============================================================
-- 表：DatabaseMigration（迁移记录）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'DATABASEMIGRATION') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE DatabaseMigration (
        Id INT IDENTITY(1,1) NOT NULL CONSTRAINT PK_DatabaseMigration PRIMARY KEY,
        MigrationId VARCHAR(64) NOT NULL CONSTRAINT UQ_DatabaseMigration_MigrationId UNIQUE,
        MigrationName VARCHAR(256) NOT NULL,
        AppliedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        AppliedBy VARCHAR(128) NULL,
        Version INT NOT NULL,
        ScriptPath VARCHAR(512) NULL
    )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_DATABASEMIGRATION_VERSION') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_DatabaseMigration_Version ON DatabaseMigration(Version)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_DATABASEMIGRATION_APPLIEDAT') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_DatabaseMigration_AppliedAt ON DatabaseMigration(AppliedAt)';
  END IF;
END;
/

-- ============================================================
-- 表：LicensePlateRecord（车牌识别记录）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LICENSEPLATERECORD') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LicensePlateRecord (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_LicensePlateRecord PRIMARY KEY,
        WorkflowId VARCHAR(128) NULL,
        InteractionId VARCHAR(64) NULL,
        CameraIdentifier VARCHAR(64) NOT NULL,
        CameraId VARCHAR(64) NOT NULL DEFAULT '''',
        LicensePlate VARCHAR(32) NOT NULL DEFAULT '''',
        CaptureTime DATETIME(3) NULL,
        EnterId CLOB NULL,
        RequestTime DATETIME(3) NOT NULL,
        ResponseCode INT NOT NULL,
        ResponseMessage VARCHAR(512) NOT NULL DEFAULT '''',
        IsSuccess BIT NOT NULL DEFAULT 0,
        ErrorMessage VARCHAR(1024) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE
    )';
  END IF;
END;
/

-- ============================================================
-- 表：VehicleDimensionRecord（车辆尺寸记录）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEDIMENSIONRECORD') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleDimensionRecord (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionRecord PRIMARY KEY,
        IdempotencyKey VARCHAR(64) NOT NULL DEFAULT '''',
        WorkflowId VARCHAR(128) NULL,
        PairId VARCHAR(64) NULL,
        PayloadHash VARCHAR(64) NULL,
        LicensePlate VARCHAR(32) NOT NULL,
        CameraIdentifier VARCHAR(64) NOT NULL,
        GetTime DATETIME(3) NOT NULL,
        DetectStatus VARCHAR(32) NOT NULL DEFAULT ''success'',
        ActualRibCount INT NULL,
        MaximumWidthCm DECIMAL(10,1) NULL,
        CabToTrailerFrontDist DECIMAL(10,1) NULL,
        TrailerLength DECIMAL(10,1) NULL,
        TrailerWidth DECIMAL(10,1) NULL,
        TrailerBodyHeight DECIMAL(10,1) NULL,
        TrailerBottomToGroundHeight DECIMAL(10,1) NULL,
        HighLowPlateHeight DECIMAL(10,1) NULL,
        SourceFile VARCHAR(512) NULL,
        ResultJsonPath VARCHAR(512) NULL,
        ResultImagePath VARCHAR(512) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        CONSTRAINT CK_VehicleDimensionRecord_DetectStatus CHECK (DetectStatus IN (''success'', ''failed'', ''low_confidence'', ''unknown''))
    )';
  END IF;
END;
/

-- ============================================================
-- 表：VehicleDimensionRib（车辆尺寸-横梁）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEDIMENSIONRIB') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleDimensionRib (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionRib PRIMARY KEY,
        DimensionRecordId BIGINT NOT NULL,
        RibId INT NOT NULL,
        DistToPrev DECIMAL(10,1) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        CONSTRAINT FK_VehicleDimensionRib_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
    )';
  END IF;
END;
/

-- ============================================================
-- 表：VehicleDimensionWire（车辆尺寸-拉线）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEDIMENSIONWIRE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleDimensionWire (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionWire PRIMARY KEY,
        DimensionRecordId BIGINT NOT NULL,
        WireId INT NOT NULL,
        DistToPrev DECIMAL(10,1) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        CONSTRAINT FK_VehicleDimensionWire_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
    )';
  END IF;
END;
/

-- ============================================================
-- 表：CoalVolumeRecord（煤量体积记录）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'COALVOLUMERECORD') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE CoalVolumeRecord (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_CoalVolumeRecord PRIMARY KEY,
        IdempotencyKey VARCHAR(64) NOT NULL DEFAULT '''',
        BusinessKey VARCHAR(64) NOT NULL,
        WorkflowId VARCHAR(128) NULL,
        PayloadHash VARCHAR(64) NULL,
        LicensePlate VARCHAR(32) NOT NULL,
        CameraIdentifier VARCHAR(64) NOT NULL,
        Volume DECIMAL(10,1) NOT NULL,
        Unit VARCHAR(16) NOT NULL DEFAULT ''m³'',
        MeasureTime DATETIME(3) NOT NULL,
        MeasurementStartedAt DATETIME(3) NULL,
        MeasurementEndedAt DATETIME(3) NULL,
        LicensePlateRecordId BIGINT NULL,
        Status VARCHAR(32) NOT NULL DEFAULT ''success'',
        Confidence DECIMAL(5,2) NULL,
        SourceFile VARCHAR(512) NULL,
        BaselineFile VARCHAR(512) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        CONSTRAINT CK_CoalVolumeRecord_Status CHECK (Status IN (''success'', ''failed'', ''low_confidence'', ''unknown'')),
        CONSTRAINT CK_CoalVolumeRecord_Confidence CHECK (Confidence IS NULL OR (Confidence >= 0 AND Confidence <= 1)),
        CONSTRAINT CK_CoalVolumeRecord_Volume CHECK (Volume >= 0 AND Volume <= 1000)
    )';
  END IF;
END;
/

-- ============================================================
-- 表：ApiAccessLog（API访问日志）
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'APIACCESSLOG') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE ApiAccessLog (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_ApiAccessLog PRIMARY KEY,
        RecordKind VARCHAR(16) NOT NULL DEFAULT ''Access'',
        LogLevel VARCHAR(16) NOT NULL DEFAULT ''info'',
        LifecycleStage VARCHAR(64) NULL,
        LogMessage VARCHAR(1024) NULL,
        InteractionId VARCHAR(64) NULL,
        IntegrationType VARCHAR(32) NULL,
        Direction VARCHAR(16) NULL,
        CallerSystem VARCHAR(64) NULL,
        TargetSystem VARCHAR(64) NULL,
        WorkflowId VARCHAR(128) NULL,
        EventId VARCHAR(128) NULL,
        LicensePlate VARCHAR(32) NULL,
        CameraIdentifier VARCHAR(64) NULL,
        AttemptNo INT NULL,
        HttpMethod VARCHAR(16) NOT NULL DEFAULT '''',
        RequestPath VARCHAR(256) NOT NULL,
        QueryString VARCHAR(1024) NULL,
        Scheme VARCHAR(16) NULL,
        Host VARCHAR(256) NULL,
        TraceIdentifier VARCHAR(128) NULL,
        ClientIp VARCHAR(64) NULL,
        UserAgent VARCHAR(512) NULL,
        RequestContentType VARCHAR(256) NULL,
        TimestampHeader VARCHAR(32) NULL,
        SignatureHeader VARCHAR(64) NULL,
        IdempotencyKeyHeader VARCHAR(128) NULL,
        RequestHeadersJson TEXT NULL,
        RequestBody TEXT NULL,
        ResponseContentType VARCHAR(256) NULL,
        ResponseHeadersJson TEXT NULL,
        ResponseBody TEXT NULL,
        RequestContentLength BIGINT NULL,
        ResponseContentLength BIGINT NULL,
        HttpStatus INT NOT NULL,
        BusinessCode INT NOT NULL,
        ElapsedMs INT NOT NULL,
        ErrorMessage VARCHAR(1024) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE
    )';
  END IF;
END;
/

-- ============================================================
-- 索引：LicensePlateRecord
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LICENSEPLATERECORD_CAMERA_TIME') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LicensePlateRecord_Camera_Time ON LicensePlateRecord(CameraIdentifier, CaptureTime DESC)';
  END IF;
END;
/

-- ============================================================
-- 索引：VehicleDimensionRecord
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONRECORD_GETTIME') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionRecord_GetTime ON VehicleDimensionRecord(GetTime DESC)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSION_CURSOR') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimension_Cursor ON VehicleDimensionRecord(GetTime DESC, Id DESC)';
  END IF;
END;
/

-- 幂等键唯一索引（原 SQL Server 版带 WHERE IdempotencyKey <> '' 过滤条件，
-- 达梦不支持过滤索引，改为常规唯一索引，空值由应用层保证唯一性）
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEDIMENSIONRECORD_IDEMPOTENCYKEY') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleDimensionRecord_IdempotencyKey ON VehicleDimensionRecord(IdempotencyKey)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONRECORD_LICENSEPLATE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionRecord_LicensePlate ON VehicleDimensionRecord(LicensePlate)';
  END IF;
END;
/

-- ============================================================
-- 索引：VehicleDimensionRib / VehicleDimensionWire
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONRIB_RECORD_RIB') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionRib_Record_Rib ON VehicleDimensionRib(DimensionRecordId, RibId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONWIRE_RECORD_WIRE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionWire_Record_Wire ON VehicleDimensionWire(DimensionRecordId, WireId)';
  END IF;
END;
/

-- ============================================================
-- 索引：CoalVolumeRecord
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_COALVOLUMERECORD_MEASURETIME') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_CoalVolumeRecord_MeasureTime ON CoalVolumeRecord(MeasureTime DESC)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_COALVOLUME_CURSOR') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_CoalVolume_Cursor ON CoalVolumeRecord(MeasureTime DESC, Id DESC)';
  END IF;
END;
/

-- 幂等键唯一索引（同上，移除 WHERE 过滤条件）
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_COALVOLUMERECORD_IDEMPOTENCYKEY') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_CoalVolumeRecord_IdempotencyKey ON CoalVolumeRecord(IdempotencyKey)';
  END IF;
END;
/

-- ============================================================
-- 索引：ApiAccessLog
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_CREATEDAT') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_CreatedAt ON ApiAccessLog(CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_INTERACTIONID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_InteractionId ON ApiAccessLog(InteractionId, CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_WorkflowId ON ApiAccessLog(WorkflowId, CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_INTEGRATIONTYPE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_IntegrationType ON ApiAccessLog(IntegrationType, CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_LOGLEVEL') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_LogLevel ON ApiAccessLog(LogLevel, CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_KIND_STAGE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_Kind_Stage ON ApiAccessLog(RecordKind, LifecycleStage, CreatedAt DESC)';
  END IF;
END;
/

-- ============================================================
-- 迁移记录插入
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260521_production_baseline') THEN
    INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES ('20260521_production_baseline', 'Production baseline schema', 1, SYSDATE, USER, 'Database/Production_InitDatabase.sql');
  END IF;
END;
/

-- ============================================================
-- 表：VehicleDimensionHighLowPlate（车辆尺寸-高低板明细）
-- API 文档 v2.0 变更：标量字段改为数组结构
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEDIMENSIONHIGHLOWPLATE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleDimensionHighLowPlate (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionHighLowPlate PRIMARY KEY,
        DimensionRecordId BIGINT NOT NULL,
        PlateId INT NOT NULL,
        HighestToGround DECIMAL(10,1),
        LengthCm DECIMAL(10,1),
        LowestToGround DECIMAL(10,1),
        CONSTRAINT FK_VehicleDimensionHighLowPlate_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
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

-- 迁移记录：v2.0 highLowPlates 数组结构
BEGIN
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260724_highlowplate_array') THEN
    INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES ('20260724_highlowplate_array', 'v2.0: highLowPlates array structure', 2, SYSDATE, USER, 'Sql/CreateHighLowPlateTable.sql');
  END IF;
END;
/

COMMIT;

-- ============================================================
-- v10: 持久化车牌任务与内部幂等约束（第三方契约不变）
-- ============================================================
-- Production_InitDatabase.sql 也用于全新库，因此先保证被任务外键引用的流程表存在。
-- 后面的 v6 区块仍负责兼容旧库补列和登记 v6 迁移。
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEARRIVAL') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleArrival (
      Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleArrival PRIMARY KEY,
      EventId VARCHAR(128) NOT NULL,
      DeviceId VARCHAR(64) NOT NULL,
      CameraIdentifier VARCHAR(64) NOT NULL,
      EventType VARCHAR(32) NOT NULL,
      SessionId VARCHAR(128) NOT NULL,
      LicensePlate VARCHAR(32) NULL,
      PlateStatus VARCHAR(32) NOT NULL DEFAULT ''PENDING'',
      PlateSource VARCHAR(32) NULL,
      PlateCapturedAt DATETIME(3) NULL,
      ArrivalTime DATETIME(3) NOT NULL,
      RadarStatus VARCHAR(32) NOT NULL DEFAULT ''NOT_REQUIRED'',
      RadarError VARCHAR(1024) NULL,
      CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
      UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
      CONSTRAINT UX_VehicleArrival_SessionId UNIQUE (SessionId),
      CONSTRAINT CK_VehicleArrival_RadarStatus CHECK (RadarStatus IN (''NOT_REQUIRED'', ''PENDING'', ''SUCCEEDED'', ''FAILED'')),
      CONSTRAINT CK_VehicleArrival_PlateStatus CHECK (PlateStatus IN (''PENDING'', ''SUCCEEDED'', ''FAILED''))
    )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'LICENSEPLATERECORD' AND COLUMN_NAME = 'WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE LicensePlateRecord ADD WorkflowId VARCHAR(128) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'LICENSEPLATERECORD' AND COLUMN_NAME = 'INTERACTIONID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE LicensePlateRecord ADD InteractionId VARCHAR(64) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'COALVOLUMERECORD' AND COLUMN_NAME = 'BUSINESSKEY') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE CoalVolumeRecord ADD BusinessKey VARCHAR(64) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'COALVOLUMERECORD' AND COLUMN_NAME = 'WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE CoalVolumeRecord ADD WorkflowId VARCHAR(128) NULL';
  END IF;
END;
/

UPDATE CoalVolumeRecord SET BusinessKey = 'legacy-' || TO_CHAR(Id) WHERE BusinessKey IS NULL;
/

BEGIN
  IF EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
       WHERE TABLE_NAME = 'COALVOLUMERECORD'
         AND COLUMN_NAME = 'BUSINESSKEY'
         AND NULLABLE = 'Y') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE CoalVolumeRecord MODIFY BusinessKey VARCHAR(64) NOT NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'PLATEACQUISITIONJOB') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE PlateAcquisitionJob (
      Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_PlateAcquisitionJob PRIMARY KEY,
      WorkflowId VARCHAR(128) NOT NULL,
      InteractionId VARCHAR(64) NOT NULL,
      CameraIdentifier VARCHAR(64) NOT NULL,
      Status VARCHAR(16) NOT NULL DEFAULT ''PENDING'',
      AttemptCount INT NOT NULL DEFAULT 0,
      NextAttemptAt DATETIME(3) NOT NULL,
      LastError VARCHAR(1024) NULL,
      CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
      UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
      CONSTRAINT FK_PlateAcquisitionJob_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId),
      CONSTRAINT CK_PlateAcquisitionJob_Status CHECK (Status IN (''PENDING'', ''PROCESSING'', ''RETRY'', ''SUCCEEDED'', ''FAILED''))
    )';
  END IF;
END;
/

INSERT INTO PlateAcquisitionJob
  (WorkflowId, InteractionId, CameraIdentifier, Status, AttemptCount,
   NextAttemptAt, LastError, CreatedAt, UpdatedAt)
SELECT v.SessionId, 'legacy-' || TO_CHAR(v.Id), v.CameraIdentifier,
       'PENDING', 0, SYSDATE, NULL, SYSDATE, SYSDATE
  FROM VehicleArrival v
 WHERE v.PlateStatus = 'PENDING'
   AND NOT EXISTS (SELECT 1 FROM PlateAcquisitionJob j WHERE j.WorkflowId = v.SessionId);
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_LICENSEPLATERECORD_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_LicensePlateRecord_WorkflowId ON LicensePlateRecord(WorkflowId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_PLATEACQUISITIONJOB_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_PlateAcquisitionJob_WorkflowId ON PlateAcquisitionJob(WorkflowId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_PLATEACQUISITIONJOB_DUE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_PlateAcquisitionJob_Due ON PlateAcquisitionJob(Status, NextAttemptAt)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_COALVOLUMERECORD_BUSINESSKEY') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_CoalVolumeRecord_BusinessKey ON CoalVolumeRecord(BusinessKey)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEDIMENSIONRECORD_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleDimensionRecord_WorkflowId ON VehicleDimensionRecord(WorkflowId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEDIMENSIONRECORD_PAIRID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleDimensionRecord_PairId ON VehicleDimensionRecord(PairId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_COALVOLUMERECORD_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_CoalVolumeRecord_WorkflowId ON CoalVolumeRecord(WorkflowId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEDIMENSIONRIB_RECORD_RIB') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleDimensionRib_Record_Rib ON VehicleDimensionRib(DimensionRecordId, RibId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEDIMENSIONWIRE_RECORD_WIRE') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleDimensionWire_Record_Wire ON VehicleDimensionWire(DimensionRecordId, WireId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_CONSTRAINTS WHERE CONSTRAINT_NAME = 'FK_LICENSEPLATERECORD_WORKFLOW') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE LicensePlateRecord ADD CONSTRAINT FK_LicensePlateRecord_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_CONSTRAINTS WHERE CONSTRAINT_NAME = 'FK_COALVOLUMERECORD_WORKFLOW') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE CoalVolumeRecord ADD CONSTRAINT FK_CoalVolumeRecord_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260814_workflow_idempotency_v10') THEN
    INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES ('20260814_workflow_idempotency_v10', 'Durable plate workflow and business idempotency', 10, SYSDATE, USER, 'Database/Production_InitDatabase.sql');
  END IF;
END;
/

COMMIT;

-- v10 的增量升级与约束定义见：
-- Database/dm8_upgrade_v9_to_v10_workflow_idempotency.sql

-- ============================================================
-- v6: 来车 workflow、唯一车牌及分析结果关联
-- ============================================================
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEARRIVAL') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleArrival (
        Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleArrival PRIMARY KEY,
        EventId VARCHAR(128) NOT NULL,
        DeviceId VARCHAR(64) NOT NULL,
        CameraIdentifier VARCHAR(64) NOT NULL,
        EventType VARCHAR(32) NOT NULL,
        SessionId VARCHAR(128) NOT NULL,
        LicensePlate VARCHAR(32) NULL,
        PlateStatus VARCHAR(32) NOT NULL DEFAULT ''PENDING'',
        PlateSource VARCHAR(32) NULL,
        PlateCapturedAt DATETIME(3) NULL,
        ArrivalTime DATETIME(3) NOT NULL,
        RadarStatus VARCHAR(32) NOT NULL DEFAULT ''NOT_REQUIRED'',
        RadarError VARCHAR(1024) NULL,
        CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
        CONSTRAINT CK_VehicleArrival_RadarStatus CHECK (RadarStatus IN (''NOT_REQUIRED'', ''PENDING'', ''SUCCEEDED'', ''FAILED'')),
        CONSTRAINT CK_VehicleArrival_PlateStatus CHECK (PlateStatus IN (''PENDING'', ''SUCCEEDED'', ''FAILED''))
    )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'LICENSEPLATE') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD LicensePlate VARCHAR(32) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'PLATESTATUS') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD PlateStatus VARCHAR(32) DEFAULT ''PENDING'' NOT NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'PLATESOURCE') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD PlateSource VARCHAR(32) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'PLATECAPTUREDAT') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD PlateCapturedAt DATETIME(3) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEDIMENSIONRECORD' AND COLUMN_NAME = 'WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleDimensionRecord ADD WorkflowId VARCHAR(128) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'VEHICLEDIMENSIONRECORD' AND COLUMN_NAME = 'PAIRID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleDimensionRecord ADD PairId VARCHAR(64) NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEARRIVAL_EVENTID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleArrival_EventId ON VehicleArrival(EventId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEARRIVAL_SESSIONID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleArrival_SessionId ON VehicleArrival(SessionId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONRECORD_WORKFLOWID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionRecord_WorkflowId ON VehicleDimensionRecord(WorkflowId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSIONRECORD_PAIRID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimensionRecord_PairId ON VehicleDimensionRecord(PairId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_CONSTRAINTS WHERE CONSTRAINT_NAME = 'FK_VEHICLEDIMENSIONRECORD_WORKFLOW') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleDimensionRecord ADD CONSTRAINT FK_VehicleDimensionRecord_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260812_workflow_correlation') THEN
    INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES ('20260812_workflow_correlation', 'End-to-end vehicle workflow correlation', 6, SYSDATE, USER, 'Database/Production_InitDatabase.sql');
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS
                 WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'DIMENSIONUNAVAILABLEAT') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD DimensionUnavailableAt DATETIME(3)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS
                 WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'DIMENSIONUNAVAILABLECAMERA') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD DimensionUnavailableCamera VARCHAR(32)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_ARRIVAL_DIMUNAVAILABLEAT') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_Arrival_DimUnavailableAt ON VehicleArrival(DimensionUnavailableAt DESC)';
  END IF;
END;
/

COMMIT;
