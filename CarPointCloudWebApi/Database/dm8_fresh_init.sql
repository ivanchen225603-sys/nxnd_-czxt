-- CarPointCloudWebApi DM8 fresh-schema initializer.
-- Run only against a newly created, empty CarPointCloud schema.
-- This file intentionally contains plain DDL/DML only; it does not use
-- SQL Server-style named DEFAULT constraints or conditional DMSQL blocks.

SET SCHEMA CarPointCloud;

CREATE TABLE DatabaseMigration (
    Id INT IDENTITY(1,1) NOT NULL,
    MigrationId VARCHAR(64) NOT NULL,
    MigrationName VARCHAR(256) NOT NULL,
    AppliedAt DATETIME DEFAULT SYSDATE NOT NULL,
    AppliedBy VARCHAR(128),
    Version INT NOT NULL,
    ScriptPath VARCHAR(512),
    PRIMARY KEY (Id),
    UNIQUE (MigrationId)
);
/

CREATE TABLE LicensePlateRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    WorkflowId VARCHAR(128),
    InteractionId VARCHAR(64),
    CameraIdentifier VARCHAR(64) NOT NULL,
    CameraId VARCHAR(64) DEFAULT '' NOT NULL,
    LicensePlate VARCHAR(32) DEFAULT '' NOT NULL,
    CaptureTime DATETIME,
    EnterId CLOB,
    RequestTime DATETIME NOT NULL,
    ResponseCode INT NOT NULL,
    ResponseMessage VARCHAR(512) DEFAULT '' NOT NULL,
    IsSuccess BIT DEFAULT 0 NOT NULL,
    ErrorMessage VARCHAR(1024),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id)
);
/

CREATE TABLE VehicleDimensionRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    IdempotencyKey VARCHAR(64) DEFAULT '' NOT NULL,
    WorkflowId VARCHAR(128),
    PairId VARCHAR(64),
    PayloadHash VARCHAR(64),
    LicensePlate VARCHAR(32) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    GetTime DATETIME NOT NULL,
    DetectStatus VARCHAR(32) DEFAULT 'success' NOT NULL,
    ActualRibCount INT,
    MaximumWidthCm DECIMAL(10,1) NULL,
    CabToTrailerFrontDist DECIMAL(10,1),
    TrailerLength DECIMAL(10,1),
    TrailerWidth DECIMAL(10,1),
    TrailerBodyHeight DECIMAL(10,1),
    TrailerBottomToGroundHeight DECIMAL(10,1),
    HighLowPlateHeight DECIMAL(10,1),
    SourceFile VARCHAR(512),
    ResultJsonPath VARCHAR(512),
    ResultImagePath VARCHAR(512),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    CHECK (DetectStatus IN ('success', 'failed', 'low_confidence', 'unknown'))
);
/

CREATE TABLE VehicleDimensionRib (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    DimensionRecordId BIGINT NOT NULL,
    RibId INT NOT NULL,
    DistToPrev DECIMAL(10,1),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE VehicleDimensionWire (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    DimensionRecordId BIGINT NOT NULL,
    WireId INT NOT NULL,
    DistToPrev DECIMAL(10,1),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE VehicleDimensionHighLowPlate (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    DimensionRecordId BIGINT NOT NULL,
    PlateId INT NOT NULL,
    HighestToGround DECIMAL(10,1),
    LengthCm DECIMAL(10,1),
    LowestToGround DECIMAL(10,1),
    PRIMARY KEY (Id),
    FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id)
);
/

CREATE TABLE EmptyVehicleProfile (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    LicensePlate VARCHAR(32) NOT NULL,
    DimensionRecordId BIGINT NOT NULL,
    WorkflowId VARCHAR(128),
    PairId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    PointCloudPath VARCHAR(512) NOT NULL,
    PointCloudSha256 VARCHAR(64) NOT NULL,
    PointCloudSize BIGINT NOT NULL,
    PointCount BIGINT NOT NULL,
    ScanTime DATETIME NOT NULL,
    CabToTrailerFrontDist DECIMAL(10,1),
    TrailerLength DECIMAL(10,1),
    TrailerWidth DECIMAL(10,1),
    TrailerBodyHeight DECIMAL(10,1),
    TrailerBottomToGroundHeight DECIMAL(10,1),
    HighLowPlateHeight DECIMAL(10,1),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id),
    CHECK (PointCloudSize > 0),
    CHECK (PointCount > 0)
);
/

CREATE TABLE CoalVolumeRecord (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    IdempotencyKey VARCHAR(64) DEFAULT '' NOT NULL,
    BusinessKey VARCHAR(64) NOT NULL,
    WorkflowId VARCHAR(128),
    PayloadHash VARCHAR(64),
    LicensePlate VARCHAR(32) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    Volume DECIMAL(10,1) NOT NULL,
    Unit VARCHAR(16) DEFAULT 'm³' NOT NULL,
    MeasureTime DATETIME NOT NULL,
    MeasurementStartedAt DATETIME(3),
    MeasurementEndedAt DATETIME(3),
    LicensePlateRecordId BIGINT,
    Status VARCHAR(32) DEFAULT 'success' NOT NULL,
    Confidence DECIMAL(5,2),
    SourceFile VARCHAR(512),
    BaselineFile VARCHAR(512),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    CHECK (Status IN ('success', 'failed', 'low_confidence', 'unknown')),
    CHECK (Confidence IS NULL OR (Confidence >= 0 AND Confidence <= 1)),
    CHECK (Volume >= 0 AND Volume <= 1000)
);
/

CREATE TABLE ApiAccessLog (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    RecordKind VARCHAR(16) DEFAULT 'Access' NOT NULL,
    LogLevel VARCHAR(16) DEFAULT 'info' NOT NULL,
    LifecycleStage VARCHAR(64),
    LogMessage VARCHAR(1024),
    InteractionId VARCHAR(64),
    IntegrationType VARCHAR(32),
    Direction VARCHAR(16),
    CallerSystem VARCHAR(64),
    TargetSystem VARCHAR(64),
    WorkflowId VARCHAR(128),
    EventId VARCHAR(128),
    LicensePlate VARCHAR(32),
    CameraIdentifier VARCHAR(64),
    AttemptNo INT,
    HttpMethod VARCHAR(16) DEFAULT '' NOT NULL,
    RequestPath VARCHAR(256) NOT NULL,
    QueryString VARCHAR(1024),
    Scheme VARCHAR(16),
    Host VARCHAR(256),
    TraceIdentifier VARCHAR(128),
    ClientIp VARCHAR(64),
    UserAgent VARCHAR(512),
    RequestContentType VARCHAR(256),
    TimestampHeader VARCHAR(32),
    SignatureHeader VARCHAR(64),
    IdempotencyKeyHeader VARCHAR(128),
    RequestHeadersJson TEXT,
    RequestBody TEXT,
    ResponseContentType VARCHAR(256),
    ResponseHeadersJson TEXT,
    ResponseBody TEXT,
    RequestContentLength BIGINT,
    ResponseContentLength BIGINT,
    HttpStatus INT NOT NULL,
    BusinessCode INT NOT NULL,
    ElapsedMs INT NOT NULL,
    ErrorMessage VARCHAR(1024),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id)
);
/

CREATE TABLE VehicleArrival (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    EventId VARCHAR(128) NOT NULL,
    DeviceId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    EventType VARCHAR(32) NOT NULL,
    SessionId VARCHAR(128) NOT NULL,
    LicensePlate VARCHAR(32),
    PlateStatus VARCHAR(32) DEFAULT 'PENDING' NOT NULL,
    PlateSource VARCHAR(32),
    PlateCapturedAt DATETIME,
    ArrivalTime DATETIME NOT NULL,
    RadarStatus VARCHAR(32) DEFAULT 'NOT_REQUIRED' NOT NULL,
    RadarError VARCHAR(1024),
    DimensionUnavailableAt DATETIME(3),
    DimensionUnavailableCamera VARCHAR(32),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    CHECK (RadarStatus IN ('NOT_REQUIRED', 'PENDING', 'SUCCEEDED', 'FAILED')),
    CHECK (PlateStatus IN ('PENDING', 'SUCCEEDED', 'FAILED'))
);
/

-- DM requires the referenced key to be unique before any workflow foreign key.
CREATE UNIQUE INDEX UX_VehicleArrival_SessionId ON VehicleArrival(SessionId);
/

CREATE TABLE PlateAcquisitionJob (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    WorkflowId VARCHAR(128) NOT NULL,
    CandidateJson CLOB,
    InteractionId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    Status VARCHAR(16) DEFAULT 'PENDING' NOT NULL,
    AttemptCount INT DEFAULT 0 NOT NULL,
    NextAttemptAt DATETIME NOT NULL,
    LastError VARCHAR(1024),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId),
    CHECK (Status IN ('PENDING', 'PROCESSING', 'RETRY', 'SUCCEEDED', 'FAILED')),
    CHECK (AttemptCount >= 0)
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

CREATE INDEX IX_DatabaseMigration_Version ON DatabaseMigration(Version);
/
CREATE INDEX IX_DatabaseMigration_AppliedAt ON DatabaseMigration(AppliedAt);
/
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
CREATE UNIQUE INDEX UX_VDHLP_DimensionRecordId_PlateId ON VehicleDimensionHighLowPlate(DimensionRecordId, PlateId);
/
CREATE UNIQUE INDEX UX_EmptyVehicleProfile_LicensePlate ON EmptyVehicleProfile(LicensePlate);
/
CREATE INDEX IX_EmptyVehicleProfile_ScanTime ON EmptyVehicleProfile(ScanTime DESC);
/
CREATE INDEX IX_EmptyVehicleProfile_DimensionId ON EmptyVehicleProfile(DimensionRecordId);
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
CREATE UNIQUE INDEX UX_LicensePlateRecord_WorkflowId ON LicensePlateRecord(WorkflowId);
/
CREATE UNIQUE INDEX UX_PlateAcquisitionJob_WorkflowId ON PlateAcquisitionJob(WorkflowId);
/
CREATE INDEX IX_PlateAcquisitionJob_Due ON PlateAcquisitionJob(Status, NextAttemptAt);
/
CREATE INDEX IX_VehicleArrival_DeviceId_CreatedAt ON VehicleArrival(DeviceId, CreatedAt DESC);
/
CREATE INDEX IX_VehicleArrival_CameraIdentifier ON VehicleArrival(CameraIdentifier);
/
CREATE INDEX IX_Arrival_DimUnavailableAt ON VehicleArrival(DimensionUnavailableAt DESC);
/

INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260521_production_baseline', 'Production baseline schema', 1, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260724_highlowplate_array', 'v2.0: highLowPlates array structure', 2, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260601_vehicle_arrival', 'Vehicle arrival table', 3, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260801_vehicle_arrival_reliability', 'Vehicle arrival idempotency and radar result state', 4, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260801_payload_hash_cursor', 'Payload fingerprints and cursor indexes', 5, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260812_workflow_correlation', 'End-to-end vehicle workflow correlation', 6, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260813_empty_vehicle_profile', 'Latest fused empty-vehicle profile', 7, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260814_integration_audit', 'Correlated third-party interaction audit', 8, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260814_interface_lifecycle_logs', 'Four-level interface lifecycle logs', 9, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/
INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260814_workflow_idempotency_v10', 'Durable plate workflow and business idempotency', 10, SYSDATE, USER, 'database/dm8_fresh_init.sql');
/

COMMIT;
