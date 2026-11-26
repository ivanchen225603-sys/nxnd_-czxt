-- 仅新增乙方抓拍观察表；首次可见为我方业务时刻，不回推旧数据。可重复执行。
SET SCHEMA CarPointCloud;

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'PARTNERPLATEOBSERVATION') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE PartnerPlateObservation (
    ObservationKey VARCHAR(64) NOT NULL PRIMARY KEY,
    CameraIdentifier VARCHAR(64) NOT NULL,
    EnterId CLOB NOT NULL,
    FirstSeenAt DATETIME(3) NOT NULL,
    CaptureTimeText VARCHAR(19) NOT NULL,
    LicensePlate VARCHAR(32) NOT NULL,
    IdentityConflict INT DEFAULT 0 NOT NULL
)';
  END IF;
END;
/
