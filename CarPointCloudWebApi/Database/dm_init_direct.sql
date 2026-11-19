-- 达梦 DM8 直接建表脚本（无条件检查，适用于全新数据库）
SET SCHEMA CarPointCloud;

CREATE TABLE DatabaseMigration (
    Id INT IDENTITY(1,1) NOT NULL CONSTRAINT PK_DatabaseMigration PRIMARY KEY,
    MigrationId VARCHAR(64) NOT NULL CONSTRAINT UQ_DatabaseMigration_MigrationId UNIQUE,
    MigrationName VARCHAR(256) NOT NULL,
    AppliedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    AppliedBy VARCHAR(128) NULL,
    Version INT NOT NULL,
    ScriptPath VARCHAR(512) NULL
);
/

CREATE INDEX IX_DatabaseMigration_Version ON DatabaseMigration(Version);
/
CREATE INDEX IX_DatabaseMigration_AppliedAt ON DatabaseMigration(AppliedAt);
/

CREATE TABLE LicensePlateRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_LicensePlateRecord PRIMARY KEY,
    WorkflowId VARCHAR(128) NULL,
    InteractionId VARCHAR(64) NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    CameraId VARCHAR(64) NOT NULL DEFAULT '',
    LicensePlate VARCHAR(32) NOT NULL DEFAULT '',
    CaptureTime DATETIME(3) NULL,
    EnterId CLOB NULL,
    RequestTime DATETIME(3) NOT NULL,
    ResponseCode INT NOT NULL,
    ResponseMessage VARCHAR(512) NOT NULL DEFAULT '',
    IsSuccess BIT NOT NULL DEFAULT 0,
    ErrorMessage VARCHAR(1024) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE
);
/

CREATE TABLE VehicleDimensionRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionRecord PRIMARY KEY,
    IdempotencyKey VARCHAR(64) NOT NULL DEFAULT '',
    WorkflowId VARCHAR(128) NULL,
    PairId VARCHAR(64) NULL,
    PayloadHash VARCHAR(64) NULL,
    LicensePlate VARCHAR(32) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    GetTime DATETIME(3) NOT NULL,
    DetectStatus VARCHAR(32) NOT NULL DEFAULT 'success',
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
    CONSTRAINT CK_VehicleDimensionRecord_DetectStatus CHECK (DetectStatus IN ('success', 'failed', 'low_confidence', 'unknown'))
);
/

CREATE TABLE VehicleDimensionRib (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionRib PRIMARY KEY,
    DimensionRecordId BIGINT NOT NULL,
    RibId INT NOT NULL,
    DistToPrev DECIMAL(10,1) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT FK_VehicleDimensionRib_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE VehicleDimensionWire (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionWire PRIMARY KEY,
    DimensionRecordId BIGINT NOT NULL,
    WireId INT NOT NULL,
    DistToPrev DECIMAL(10,1) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT FK_VehicleDimensionWire_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE VehicleDimensionHighLowPlate (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleDimensionHighLowPlate PRIMARY KEY,
    DimensionRecordId BIGINT NOT NULL,
    PlateId INT NOT NULL,
    HighestToGround DECIMAL(10,1),
    LengthCm DECIMAL(10,1),
    LowestToGround DECIMAL(10,1),
    CONSTRAINT FK_VehicleDimensionHighLowPlate_Record FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE CoalVolumeRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_CoalVolumeRecord PRIMARY KEY,
    IdempotencyKey VARCHAR(64) NOT NULL DEFAULT '',
    BusinessKey VARCHAR(64) NOT NULL,
    WorkflowId VARCHAR(128) NULL,
    PayloadHash VARCHAR(64) NULL,
    LicensePlate VARCHAR(32) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    Volume DECIMAL(10,1) NOT NULL,
    Unit VARCHAR(16) NOT NULL DEFAULT 'm³',
    MeasureTime DATETIME(3) NOT NULL,
    MeasurementStartedAt DATETIME(3) NULL,
    MeasurementEndedAt DATETIME(3) NULL,
    LicensePlateRecordId BIGINT NULL,
    Status VARCHAR(32) NOT NULL DEFAULT 'success',
    Confidence DECIMAL(5,2) NULL,
    SourceFile VARCHAR(512) NULL,
    BaselineFile VARCHAR(512) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT CK_CoalVolumeRecord_Status CHECK (Status IN ('success', 'failed', 'low_confidence', 'unknown')),
    CONSTRAINT CK_CoalVolumeRecord_Confidence CHECK (Confidence IS NULL OR (Confidence >= 0 AND Confidence <= 1)),
    CONSTRAINT CK_CoalVolumeRecord_Volume CHECK (Volume >= 0 AND Volume <= 1000)
);
/

CREATE TABLE ApiAccessLog (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_ApiAccessLog PRIMARY KEY,
    RecordKind VARCHAR(16) NOT NULL DEFAULT 'Access',
    LogLevel VARCHAR(16) NOT NULL DEFAULT 'info',
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
    HttpMethod VARCHAR(16) NOT NULL DEFAULT '',
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
);
/

CREATE TABLE VehicleArrival (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_VehicleArrival PRIMARY KEY,
    EventId VARCHAR(128) NOT NULL,
    DeviceId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    EventType VARCHAR(32) NOT NULL,
    SessionId VARCHAR(128) NOT NULL,
    LicensePlate VARCHAR(32) NULL,
    PlateStatus VARCHAR(32) NOT NULL DEFAULT 'PENDING',
    PlateSource VARCHAR(32) NULL,
    PlateCapturedAt DATETIME(3) NULL,
    ArrivalTime DATETIME(3) NOT NULL,
    RadarStatus VARCHAR(32) NOT NULL DEFAULT 'NOT_REQUIRED',
    RadarError VARCHAR(1024) NULL,
    DimensionUnavailableAt DATETIME(3) NULL,
    DimensionUnavailableCamera VARCHAR(32) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT CK_VehicleArrival_RadarStatus CHECK (RadarStatus IN ('NOT_REQUIRED', 'PENDING', 'SUCCEEDED', 'FAILED')),
    CONSTRAINT CK_VehicleArrival_PlateStatus CHECK (PlateStatus IN ('PENDING', 'SUCCEEDED', 'FAILED'))
);
/

ALTER TABLE VehicleDimensionRecord ADD CONSTRAINT FK_VehicleDimensionRecord_Workflow
    FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId);
/
ALTER TABLE LicensePlateRecord ADD CONSTRAINT FK_LicensePlateRecord_Workflow
    FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId);
/
ALTER TABLE CoalVolumeRecord ADD CONSTRAINT FK_CoalVolumeRecord_Workflow
    FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId);
/

CREATE TABLE PlateAcquisitionJob (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_PlateAcquisitionJob PRIMARY KEY,
    WorkflowId VARCHAR(128) NOT NULL,
    InteractionId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    Status VARCHAR(16) NOT NULL DEFAULT 'PENDING',
    AttemptCount INT NOT NULL DEFAULT 0,
    NextAttemptAt DATETIME(3) NOT NULL,
    LastError VARCHAR(1024) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT FK_PlateAcquisitionJob_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId),
    CONSTRAINT CK_PlateAcquisitionJob_Status CHECK (Status IN ('PENDING', 'PROCESSING', 'RETRY', 'SUCCEEDED', 'FAILED')),
    CONSTRAINT CK_PlateAcquisitionJob_Attempts CHECK (AttemptCount >= 0)
);
/

-- 索引
CREATE INDEX IX_LicensePlateRecord_Camera_Time ON LicensePlateRecord(CameraIdentifier, CaptureTime DESC);
/
CREATE INDEX IX_VehicleDimensionRecord_GetTime ON VehicleDimensionRecord(GetTime DESC);
/
CREATE INDEX IX_VehicleDimension_Cursor ON VehicleDimensionRecord(GetTime DESC, Id DESC);
/
CREATE UNIQUE INDEX UX_VehicleDimensionRecord_IdempotencyKey ON VehicleDimensionRecord(IdempotencyKey);
/
CREATE INDEX IX_VehicleDimensionRecord_LicensePlate ON VehicleDimensionRecord(LicensePlate);
/
CREATE INDEX IX_VehicleDimensionRecord_WorkflowId ON VehicleDimensionRecord(WorkflowId);
/
CREATE UNIQUE INDEX UX_VehicleDimensionRecord_WorkflowId ON VehicleDimensionRecord(WorkflowId);
/
CREATE INDEX IX_VehicleDimensionRecord_PairId ON VehicleDimensionRecord(PairId);
/
CREATE UNIQUE INDEX UX_VehicleDimensionRecord_PairId ON VehicleDimensionRecord(PairId);
/
CREATE INDEX IX_VehicleDimensionRib_Record_Rib ON VehicleDimensionRib(DimensionRecordId, RibId);
/
CREATE UNIQUE INDEX UX_VehicleDimensionRib_Record_Rib ON VehicleDimensionRib(DimensionRecordId, RibId);
/
CREATE INDEX IX_VehicleDimensionWire_Record_Wire ON VehicleDimensionWire(DimensionRecordId, WireId);
/
CREATE UNIQUE INDEX UX_VehicleDimensionWire_Record_Wire ON VehicleDimensionWire(DimensionRecordId, WireId);
/
CREATE INDEX IX_VDHLP_DimensionRecordId ON VehicleDimensionHighLowPlate(DimensionRecordId);
/
CREATE UNIQUE INDEX IX_VDHLP_DimensionRecordId_PlateId ON VehicleDimensionHighLowPlate(DimensionRecordId, PlateId);
/
CREATE INDEX IX_CoalVolumeRecord_MeasureTime ON CoalVolumeRecord(MeasureTime DESC);
/
CREATE INDEX IX_CoalVolume_Cursor ON CoalVolumeRecord(MeasureTime DESC, Id DESC);
/
CREATE UNIQUE INDEX UX_CoalVolumeRecord_IdempotencyKey ON CoalVolumeRecord(IdempotencyKey);
/
CREATE UNIQUE INDEX UX_CoalVolumeRecord_BusinessKey ON CoalVolumeRecord(BusinessKey);
/
CREATE INDEX IX_CoalVolumeRecord_WorkflowId ON CoalVolumeRecord(WorkflowId);
/
CREATE UNIQUE INDEX UX_LicensePlateRecord_WorkflowId ON LicensePlateRecord(WorkflowId);
/
CREATE UNIQUE INDEX UX_PlateAcquisitionJob_WorkflowId ON PlateAcquisitionJob(WorkflowId);
/
CREATE INDEX IX_PlateAcquisitionJob_Due ON PlateAcquisitionJob(Status, NextAttemptAt);
/
CREATE INDEX IX_ApiAccessLog_CreatedAt ON ApiAccessLog(CreatedAt DESC);
/
CREATE INDEX IX_ApiAccessLog_InteractionId ON ApiAccessLog(InteractionId, CreatedAt DESC);
/
CREATE INDEX IX_ApiAccessLog_WorkflowId ON ApiAccessLog(WorkflowId, CreatedAt DESC);
/
CREATE INDEX IX_ApiAccessLog_IntegrationType ON ApiAccessLog(IntegrationType, CreatedAt DESC);
/
CREATE INDEX IX_ApiAccessLog_LogLevel ON ApiAccessLog(LogLevel, CreatedAt DESC);
/
CREATE INDEX IX_ApiAccessLog_Kind_Stage ON ApiAccessLog(RecordKind, LifecycleStage, CreatedAt DESC);
/
CREATE UNIQUE INDEX UX_VehicleArrival_EventId ON VehicleArrival(EventId);
/
CREATE UNIQUE INDEX UX_VehicleArrival_SessionId ON VehicleArrival(SessionId);
/
CREATE INDEX IX_VehicleArrival_DeviceId_CreatedAt ON VehicleArrival(DeviceId, CreatedAt DESC);
/
CREATE INDEX IX_VehicleArrival_CameraIdentifier ON VehicleArrival(CameraIdentifier);
/
CREATE INDEX IX_Arrival_DimUnavailableAt ON VehicleArrival(DimensionUnavailableAt DESC);
/

-- 迁移记录
INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES ('20260521_production_baseline', 'Production baseline schema', 1, SYSDATE, USER, 'Database/dm_init_direct.sql');
INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES ('20260724_highlowplate_array', 'v2.0: highLowPlates array structure', 2, SYSDATE, USER, 'Sql/CreateHighLowPlateTable.sql');
INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES ('20260812_workflow_correlation', 'End-to-end vehicle workflow correlation', 6, SYSDATE, USER, 'Database/dm_init_direct.sql');
COMMIT;
