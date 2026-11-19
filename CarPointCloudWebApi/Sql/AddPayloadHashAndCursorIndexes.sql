-- Adds internal payload fingerprints and stable keyset pagination indexes.
-- SHA-256 here is only a database idempotency fingerprint; the third-party MD5 protocol is unchanged.
SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'VEHICLEDIMENSIONRECORD' AND COLUMN_NAME = 'PAYLOADHASH') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE VehicleDimensionRecord ADD PayloadHash VARCHAR(64) NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (
      SELECT 1 FROM USER_TAB_COLUMNS
      WHERE TABLE_NAME = 'COALVOLUMERECORD' AND COLUMN_NAME = 'PAYLOADHASH') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE CoalVolumeRecord ADD PayloadHash VARCHAR(64) NULL';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_VEHICLEDIMENSION_CURSOR') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_VehicleDimension_Cursor ON VehicleDimensionRecord(GetTime DESC, Id DESC)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_COALVOLUME_CURSOR') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_CoalVolume_Cursor ON CoalVolumeRecord(MeasureTime DESC, Id DESC)';
  END IF;
END;
/

BEGIN
  IF EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'DATABASEMIGRATION') AND
     NOT EXISTS (
       SELECT 1 FROM DatabaseMigration
       WHERE MigrationId = '20260801_payload_hash_cursor') THEN
    INSERT INTO DatabaseMigration
      (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES
      ('20260801_payload_hash_cursor',
       'Payload fingerprints and cursor indexes',
       5,
       SYSDATE,
       USER,
       'Sql/AddPayloadHashAndCursorIndexes.sql');
  END IF;
END;
/

COMMIT;
