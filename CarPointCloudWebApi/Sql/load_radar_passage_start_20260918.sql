-- 前置 load_radar_20260917.sql；仅管理员授权后在隔离库演练/正式执行，启动不自动执行。
-- 不补造旧车次，不迁移旧基线身份，不删除业务记录。
BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARPASSAGESTART') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarPassageStart (
      PassageKey VARCHAR(64) NOT NULL PRIMARY KEY,
      PayloadJson CLOB NOT NULL,
      CONSTRAINT FK_LoadPassageStart_Passage FOREIGN KEY (PassageKey) REFERENCES LoadRadarPassage(PassageKey)
    )';
  END IF;
END;
/
COMMIT;
