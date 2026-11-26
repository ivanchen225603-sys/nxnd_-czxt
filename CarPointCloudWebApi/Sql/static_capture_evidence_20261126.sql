-- CH4 自动尺寸采集静态证据。只新增对象，不改历史数据；新派发由触发器登记。
SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME='STATICCAPTUREEVIDENCE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE StaticCaptureEvidence (
      WorkflowId VARCHAR(128) PRIMARY KEY,
      Decision VARCHAR(16) NOT NULL,
      PairId VARCHAR(64),
      DeadlineMs BIGINT,
      EvidenceJson VARCHAR(4000)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME='IX_STATICCAPTURE_DUE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_StaticCapture_Due ON StaticCaptureEvidence(Decision,DeadlineMs)';
  END IF;
END;
/

CREATE OR REPLACE TRIGGER TR_StaticCaptureClaim
AFTER UPDATE OF RadarStatus ON VehicleArrival
FOR EACH ROW
BEGIN
  IF :NEW.DeviceId='LOAD_RADAR' AND :OLD.RadarStatus='NOT_REQUIRED' AND :NEW.RadarStatus='PENDING' THEN
    INSERT INTO StaticCaptureEvidence(WorkflowId,Decision) VALUES (:NEW.SessionId,'insufficient');
  END IF;
END;
/

CREATE OR REPLACE TRIGGER TR_StaticDimensionWrite
BEFORE INSERT OR UPDATE ON VehicleDimensionRecord
FOR EACH ROW
DECLARE
  blocked INT;
BEGIN
  SELECT COUNT(*) INTO blocked FROM StaticCaptureEvidence s WHERE s.WorkflowId=:NEW.WorkflowId
    AND (s.Decision <> 'confirmed' OR s.PairId IS NULL OR :NEW.PairId IS NULL OR s.PairId <> :NEW.PairId);
  IF blocked > 0 THEN
    RAISE_APPLICATION_ERROR(-20061,'Static capture evidence is not confirmed for this pair');
  END IF;
END;
/

CREATE OR REPLACE TRIGGER TR_StaticProfileWrite
BEFORE INSERT OR UPDATE ON EmptyVehicleProfile
FOR EACH ROW
DECLARE
  blocked INT;
BEGIN
  SELECT COUNT(*) INTO blocked FROM StaticCaptureEvidence s WHERE s.WorkflowId=:NEW.WorkflowId
    AND (s.Decision <> 'confirmed' OR s.PairId IS NULL OR :NEW.PairId IS NULL OR s.PairId <> :NEW.PairId);
  IF blocked > 0 THEN
    RAISE_APPLICATION_ERROR(-20061,'Static capture evidence is not confirmed for this pair');
  END IF;
END;
/

COMMIT;
