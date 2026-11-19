-- Vehicle-arrival table and idempotent reliability migration for DM8.
-- This script is safe to execute for both a new schema and an existing installation.

SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'VEHICLEARRIVAL') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE VehicleArrival (
        Id BIGINT IDENTITY(1,1) PRIMARY KEY,
        EventId VARCHAR(128) NOT NULL,
        DeviceId VARCHAR(64) NOT NULL,
        CameraIdentifier VARCHAR(64) NOT NULL,
        EventType VARCHAR(32) NOT NULL,
        SessionId VARCHAR(128) NOT NULL,
        ArrivalTime DATETIME NOT NULL,
        RadarStatus VARCHAR(32) NOT NULL DEFAULT ''NOT_REQUIRED'',
        RadarError VARCHAR(1024) NULL,
        CreatedAt DATETIME NOT NULL DEFAULT SYSDATE,
        UpdatedAt DATETIME NOT NULL DEFAULT SYSDATE,
        CONSTRAINT CK_VehicleArrival_RadarStatus
          CHECK (RadarStatus IN (''NOT_REQUIRED'', ''PENDING'', ''SUCCEEDED'', ''FAILED''))
    )';
  END IF;
END;
/

-- Upgrade an existing seven-column VehicleArrival table without deleting data.
BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'EVENTID') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD EventId VARCHAR(128) NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'RADARSTATUS') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD RadarStatus VARCHAR(32) DEFAULT ''NOT_REQUIRED'' NOT NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'RADARERROR') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD RadarError VARCHAR(1024) NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'VEHICLEARRIVAL' AND COLUMN_NAME = 'UPDATEDAT') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD UpdatedAt DATETIME DEFAULT SYSDATE NOT NULL';
  END IF;
END;
/

-- Give legacy rows stable internal event IDs before creating the unique index.
UPDATE VehicleArrival
SET EventId = 'legacy_' || CAST(Id AS VARCHAR(40))
WHERE EventId IS NULL OR EventId = '';

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_CONSTRAINTS
      WHERE CONSTRAINT_NAME = 'CK_VEHICLEARRIVAL_RADARSTATUS') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleArrival ADD CONSTRAINT CK_VehicleArrival_RadarStatus
      CHECK (RadarStatus IN (''NOT_REQUIRED'', ''PENDING'', ''SUCCEEDED'', ''FAILED''))';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEARRIVAL_EVENTID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleArrival_EventId ON VehicleArrival(EventId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_VEHICLEARRIVAL_SESSIONID') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_VehicleArrival_SessionId ON VehicleArrival(SessionId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEARRIVAL_DEVICEID_CREATEDAT') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleArrival_DeviceId_CreatedAt ON VehicleArrival(DeviceId, CreatedAt DESC)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEARRIVAL_CAMERAIDENTIFIER') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleArrival_CameraIdentifier ON VehicleArrival(CameraIdentifier)';
  END IF;
END;
/

BEGIN
  IF EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'DATABASEMIGRATION') AND
     NOT EXISTS (
       SELECT 1 FROM DatabaseMigration
       WHERE MigrationId = '20260601_vehicle_arrival') THEN
    INSERT INTO DatabaseMigration
      (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES
      ('20260601_vehicle_arrival', 'Vehicle arrival table', 3, SYSDATE, USER,
       'Sql/CreateVehicleArrivalTable.sql');
  END IF;
END;
/

BEGIN
  IF EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'DATABASEMIGRATION') AND
     NOT EXISTS (
       SELECT 1 FROM DatabaseMigration
       WHERE MigrationId = '20260801_vehicle_arrival_reliability') THEN
    INSERT INTO DatabaseMigration
      (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES
      ('20260801_vehicle_arrival_reliability',
       'Vehicle arrival idempotency and radar result state',
       4,
       SYSDATE,
       USER,
       'Sql/CreateVehicleArrivalTable.sql');
  END IF;
END;
/

COMMIT;
