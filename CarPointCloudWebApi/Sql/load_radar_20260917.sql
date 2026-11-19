-- JT128 载货雷达增量结构。仅由管理员在备份、确认 schema 后手动执行。
-- 前置：既有 VehicleArrival、CoalVolumeRecord 及 v10 workflow 幂等迁移已完成。
-- 不修改尺寸表，不回填或删除历史数据，不由应用启动自动执行。
-- 在隔离 DM8 库演练后，再对目标业务 schema 应用；DDL 不保证整体可回滚。

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARPASSAGE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarPassage (
      PassageKey VARCHAR(64) NOT NULL PRIMARY KEY,
      NodeId VARCHAR(64) NOT NULL,
      ChannelId VARCHAR(64) NOT NULL,
      PassageId VARCHAR(128) NOT NULL,
      WorkflowId VARCHAR(128),
      LicensePlate VARCHAR(32),
      BindingJson CLOB,
      CONSTRAINT UX_LoadPassage_Workflow UNIQUE (WorkflowId),
      CONSTRAINT UX_LoadPassage_Identity UNIQUE (NodeId, ChannelId, PassageId),
      CONSTRAINT UX_LoadPassage_Binding UNIQUE (PassageKey, WorkflowId),
      CONSTRAINT FK_LoadPassage_Workflow FOREIGN KEY (WorkflowId) REFERENCES VehicleArrival(SessionId)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARMEASUREMENT') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarMeasurement (
      SequenceId BIGINT IDENTITY(1,1) NOT NULL,
      MeasurementId VARCHAR(128) NOT NULL PRIMARY KEY,
      PassageKey VARCHAR(64) NOT NULL,
      NodeId VARCHAR(64) NOT NULL,
      ChannelId VARCHAR(64) NOT NULL,
      PayloadHash VARCHAR(64) NOT NULL,
      PayloadJson CLOB NOT NULL,
      ReceivedAt VARCHAR(40) NOT NULL,
      CONSTRAINT UX_LoadMeasurement_Order UNIQUE (SequenceId),
      CONSTRAINT UX_LoadMeasurement_Passage UNIQUE (MeasurementId, PassageKey),
      CONSTRAINT FK_LoadMeasurement_Passage FOREIGN KEY (PassageKey) REFERENCES LoadRadarPassage(PassageKey)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARBASELINEHEAD') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarBaselineHead (
      ScopeKey VARCHAR(64) NOT NULL PRIMARY KEY,
      ActiveVersionId VARCHAR(128)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARBASELINE') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarBaseline (
      SequenceId BIGINT IDENTITY(1,1) NOT NULL,
      VersionId VARCHAR(128) NOT NULL PRIMARY KEY,
      ScopeKey VARCHAR(64) NOT NULL,
      MeasurementId VARCHAR(128) NOT NULL,
      VersionJson CLOB NOT NULL,
      ActivatedAt VARCHAR(40),
      CONSTRAINT FK_LoadBaseline_Scope FOREIGN KEY (ScopeKey) REFERENCES LoadRadarBaselineHead(ScopeKey),
      CONSTRAINT FK_LoadBaseline_Measurement FOREIGN KEY (MeasurementId) REFERENCES LoadRadarMeasurement(MeasurementId)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_CONSTRAINTS WHERE CONSTRAINT_NAME = 'FK_LOADBASELINEHEAD_VERSION') THEN
    EXECUTE IMMEDIATE 'ALTER TABLE LoadRadarBaselineHead ADD CONSTRAINT FK_LoadBaselineHead_Version
      FOREIGN KEY (ActiveVersionId) REFERENCES LoadRadarBaseline(VersionId)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARPUBLICATION') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarPublication (
      PassageKey VARCHAR(64) NOT NULL PRIMARY KEY,
      WorkflowId VARCHAR(128) NOT NULL,
      MeasurementId VARCHAR(128) NOT NULL,
      CoalVolumeId BIGINT NOT NULL,
      PublicationJson CLOB NOT NULL,
      CONSTRAINT UX_LoadPublication_Workflow UNIQUE (WorkflowId),
      CONSTRAINT UX_LoadPublication_Measurement UNIQUE (MeasurementId),
      CONSTRAINT UX_LoadPublication_Coal UNIQUE (CoalVolumeId),
      CONSTRAINT FK_LoadPublication_Binding FOREIGN KEY (PassageKey, WorkflowId)
        REFERENCES LoadRadarPassage(PassageKey, WorkflowId),
      CONSTRAINT FK_LoadPublication_Measurement FOREIGN KEY (MeasurementId, PassageKey)
        REFERENCES LoadRadarMeasurement(MeasurementId, PassageKey),
      CONSTRAINT FK_LoadPublication_Coal FOREIGN KEY (CoalVolumeId) REFERENCES CoalVolumeRecord(Id)
    )';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_TABLES WHERE TABLE_NAME = 'LOADRADARAUDIT') THEN
    EXECUTE IMMEDIATE 'CREATE TABLE LoadRadarAudit (
      Id BIGINT IDENTITY(1,1) NOT NULL PRIMARY KEY,
      ActionName VARCHAR(32) NOT NULL,
      EntityId VARCHAR(128) NOT NULL,
      Actor VARCHAR(128) NOT NULL,
      Reason VARCHAR(1024) NOT NULL,
      RecordedAt VARCHAR(40) NOT NULL,
      PayloadJson CLOB NOT NULL
    )';
  END IF;
END;
/

BEGIN
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LOADMEASUREMENT_LANE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LoadMeasurement_Lane ON LoadRadarMeasurement(NodeId, ChannelId, SequenceId DESC)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LOADMEASUREMENT_PASSAGE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LoadMeasurement_Passage ON LoadRadarMeasurement(PassageKey)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LOADPASSAGE_PLATE') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LoadPassage_Plate ON LoadRadarPassage(LicensePlate)';
  END IF;
  IF NOT EXISTS (SELECT 1 FROM USER_INDEXES WHERE INDEX_NAME = 'IX_LOADAUDIT_ENTITY') THEN
    EXECUTE IMMEDIATE 'CREATE INDEX IX_LoadAudit_Entity ON LoadRadarAudit(EntityId, Id DESC)';
  END IF;
END;
/

COMMIT;
-- 回退仅关闭模块、回退应用；保留以上表与所有测量、基线及审计，不执行 DROP。
