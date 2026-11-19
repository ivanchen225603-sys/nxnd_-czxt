-- CarPointCloudWebApi DM8 v6 -> v7: latest fused empty-vehicle profile.
-- VehicleDimensionRecord remains the immutable per-scan history table.
-- EmptyVehicleProfile has exactly one latest row per license plate.
-- Safe to run repeatedly against the CarPointCloud schema.

SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'EMPTYVEHICLEPROFILE') THEN
    EXECUTE IMMEDIATE '
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
        ScanTime DATETIME(3) NOT NULL,
        CabToTrailerFrontDist DECIMAL(10,1),
        TrailerLength DECIMAL(10,1),
        TrailerWidth DECIMAL(10,1),
        TrailerBodyHeight DECIMAL(10,1),
        TrailerBottomToGroundHeight DECIMAL(10,1),
        HighLowPlateHeight DECIMAL(10,1),
        CreatedAt DATETIME(3) DEFAULT SYSDATE NOT NULL,
        UpdatedAt DATETIME(3) DEFAULT SYSDATE NOT NULL,
        PRIMARY KEY (Id),
        FOREIGN KEY (DimensionRecordId) REFERENCES VehicleDimensionRecord(Id),
        CHECK (PointCloudSize > 0),
        CHECK (PointCount > 0)
      )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'UX_EMPTYVEHICLEPROFILE_LICENSEPLATE') THEN
    EXECUTE IMMEDIATE 'CREATE UNIQUE INDEX UX_EmptyVehicleProfile_LicensePlate ON EmptyVehicleProfile(LicensePlate)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_EMPTYVEHICLEPROFILE_SCANTIME') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_EmptyVehicleProfile_ScanTime ON EmptyVehicleProfile(ScanTime DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_EMPTYVEHICLEPROFILE_DIMENSIONID') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_EmptyVehicleProfile_DimensionId ON EmptyVehicleProfile(DimensionRecordId)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260813_empty_vehicle_profile') THEN
    INSERT INTO DatabaseMigration
      (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES
      ('20260813_empty_vehicle_profile', 'Latest fused empty-vehicle profile', 7,
       SYSDATE, USER, 'Database/dm8_upgrade_v6_to_v7_empty_vehicle_profile.sql');
  END IF;
END;
/

COMMIT;
