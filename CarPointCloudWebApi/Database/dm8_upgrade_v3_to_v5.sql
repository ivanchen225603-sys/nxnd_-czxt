-- CarPointCloudWebApi DM8 upgrade from the v1/v2 baseline to v5.
-- Preconditions:
--   * The CarPointCloud schema already contains the v1/v2 business tables.
--   * VehicleArrival does not exist yet.
-- Run once only. Do not use this file on a brand-new empty schema.

SET SCHEMA CarPointCloud;

CREATE TABLE VehicleArrival (
    Id BIGINT IDENTITY(1,1) NOT NULL,
    EventId VARCHAR(128) NOT NULL,
    DeviceId VARCHAR(64) NOT NULL,
    CameraIdentifier VARCHAR(64) NOT NULL,
    EventType VARCHAR(32) NOT NULL,
    SessionId VARCHAR(128) NOT NULL,
    ArrivalTime DATETIME NOT NULL,
    RadarStatus VARCHAR(32) DEFAULT 'NOT_REQUIRED' NOT NULL,
    RadarError VARCHAR(1024),
    CreatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL,
    PRIMARY KEY (Id),
    CHECK (RadarStatus IN ('NOT_REQUIRED', 'PENDING', 'SUCCEEDED', 'FAILED'))
);
/

CREATE UNIQUE INDEX UX_VehicleArrival_EventId ON VehicleArrival(EventId);
/
CREATE UNIQUE INDEX UX_VehicleArrival_SessionId ON VehicleArrival(SessionId);
/
CREATE INDEX IX_VehicleArrival_DeviceId_CreatedAt ON VehicleArrival(DeviceId, CreatedAt DESC);
/
CREATE INDEX IX_VehicleArrival_CameraIdentifier ON VehicleArrival(CameraIdentifier);
/

INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260601_vehicle_arrival', 'Vehicle arrival table', 3, SYSDATE, USER,
     'database/dm8_upgrade_v3_to_v5.sql');
/

INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260801_vehicle_arrival_reliability', 'Vehicle arrival idempotency and radar result state', 4,
     SYSDATE, USER, 'database/dm8_upgrade_v3_to_v5.sql');
/

INSERT INTO DatabaseMigration
    (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES
    ('20260801_payload_hash_cursor', 'Payload fingerprints and cursor indexes', 5,
     SYSDATE, USER, 'database/dm8_upgrade_v3_to_v5.sql');
/

COMMIT;
