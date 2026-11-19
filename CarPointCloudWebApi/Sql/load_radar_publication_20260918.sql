-- 前置：load_radar_20260917.sql。仅交付代码，不由应用启动执行。
-- 管理员经备份及隔离 DM8 演练后执行。不存在历史回填/自动补发或旧数据删除。
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARPUBLISHJOB') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarPublishJob (
      MeasurementId VARCHAR(128) NOT NULL PRIMARY KEY,
      Status VARCHAR(16) NOT NULL,
      AttemptCount INT NOT NULL,
      NextAttemptAt VARCHAR(40) NOT NULL,
      ReasonCode VARCHAR(64) NOT NULL,
      CONSTRAINT CK_LoadPublishJob_Status CHECK (Status IN (''pending'', ''published'', ''rejected'')),
      CONSTRAINT CK_LoadPublishJob_Attempt CHECK (AttemptCount >= 0),
      CONSTRAINT FK_LoadPublishJob_Measurement FOREIGN KEY (MeasurementId) REFERENCES LoadRadarMeasurement(MeasurementId)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LOADPUBLISHJOB_DUE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LoadPublishJob_Due ON LoadRadarPublishJob(Status, NextAttemptAt, MeasurementId)';
  END IF;
END;
/
COMMIT;
-- 回退关闭模块、回退应用；保留任务与发布记录，不 DROP、不重放旧测量。
