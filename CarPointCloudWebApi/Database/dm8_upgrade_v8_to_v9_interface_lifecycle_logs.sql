-- CarPointCloud DM8 v8 -> v9: four-level interface lifecycle logs.
-- Safe to rerun. Existing access rows are backfilled and remain queryable.

SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'APIACCESSLOG' AND COLUMN_NAME = 'RECORDKIND') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE ApiAccessLog ADD RecordKind VARCHAR(16) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'APIACCESSLOG' AND COLUMN_NAME = 'LOGLEVEL') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE ApiAccessLog ADD LogLevel VARCHAR(16) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'APIACCESSLOG' AND COLUMN_NAME = 'LIFECYCLESTAGE') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE ApiAccessLog ADD LifecycleStage VARCHAR(64) NULL';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TAB_COLUMNS WHERE TABLE_NAME = 'APIACCESSLOG' AND COLUMN_NAME = 'LOGMESSAGE') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE ApiAccessLog ADD LogMessage VARCHAR(1024) NULL';
  END IF;
END;
/

UPDATE ApiAccessLog
   SET RecordKind = 'Access'
 WHERE RecordKind IS NULL;

UPDATE ApiAccessLog
   SET LogLevel = CASE
       WHEN HttpStatus >= 500 THEN 'error'
       WHEN HttpStatus >= 400 THEN 'warn'
       ELSE 'info'
   END
 WHERE LogLevel IS NULL;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_LOGLEVEL') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_LogLevel ON ApiAccessLog(LogLevel, CreatedAt DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_APIACCESSLOG_KIND_STAGE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_ApiAccessLog_Kind_Stage ON ApiAccessLog(RecordKind, LifecycleStage, CreatedAt DESC)';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM DatabaseMigration WHERE MigrationId = '20260814_interface_lifecycle_logs') THEN
    INSERT INTO DatabaseMigration
      (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
    VALUES
      ('20260814_interface_lifecycle_logs', 'Four-level interface lifecycle logs', 9,
       SYSDATE, USER, 'Database/dm8_upgrade_v8_to_v9_interface_lifecycle_logs.sql');
  END IF;
END;
/

COMMIT;
