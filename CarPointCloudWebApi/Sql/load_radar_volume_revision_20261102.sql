-- 人工发布前备份后执行；应用不自动运行。原始测量、CoalVolumeRecord幂等键保持不变。
CREATE TABLE LoadRadarVolumeRevision (
    Id VARCHAR(64) NOT NULL PRIMARY KEY,
    MeasurementId VARCHAR(128) NOT NULL,
    ActivationId BIGINT NOT NULL,
    BaselineVersionId VARCHAR(128) NOT NULL,
    CoalVolumeId BIGINT NOT NULL,
    ResultHash VARCHAR(64) NOT NULL,
    RevisionJson CLOB NOT NULL,
    CONSTRAINT UX_LoadRevision_Activation UNIQUE (MeasurementId, ActivationId),
    CONSTRAINT FK_LoadRevision_Measurement FOREIGN KEY (MeasurementId) REFERENCES LoadRadarMeasurement(MeasurementId),
    CONSTRAINT FK_LoadRevision_Coal FOREIGN KEY (CoalVolumeId) REFERENCES CoalVolumeRecord(Id)
);
CREATE INDEX IX_LoadRevision_Measurement ON LoadRadarVolumeRevision(MeasurementId, ActivationId);
