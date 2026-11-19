-- 达梦 DM8 用户认证与点云文件管理建表脚本
-- 说明：用户表与业务数据（VehicleDimensionRecord 等）放在同一数据库（CarPointCloud）中，
--       本脚本仅新增认证/权限/审计/点云文件相关表，不影响既有业务表。
-- 使用方式：以 SYSDBA 或具备建表权限的用户执行本脚本（先执行 SET SCHEMA 切换到目标模式）。
SET SCHEMA CarPointCloud;

-- ============================================================
-- 1. 系统用户表
-- ============================================================
CREATE TABLE SysUser (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysUser PRIMARY KEY,
    Username VARCHAR(64) NOT NULL,
    PasswordHash VARCHAR(128) NOT NULL,
    PasswordSalt VARCHAR(64) NOT NULL,
    DisplayName VARCHAR(128) NOT NULL DEFAULT '',
    Status INT NOT NULL DEFAULT 1,
    LastLoginAt DATETIME(3) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    UpdatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysUser_Username UNIQUE (Username),
    CONSTRAINT CK_SysUser_Status CHECK (Status IN (0, 1))
);
/
COMMENT ON TABLE SysUser IS '系统用户表';
COMMENT ON COLUMN SysUser.Username IS '登录用户名（唯一）';
COMMENT ON COLUMN SysUser.PasswordHash IS '密码哈希值（PBKDF2 结果，Base64）';
COMMENT ON COLUMN SysUser.PasswordSalt IS '密码盐值（随机 16 字节，Base64）';
COMMENT ON COLUMN SysUser.DisplayName IS '显示名称';
COMMENT ON COLUMN SysUser.Status IS '状态：1=启用 0=禁用';
COMMENT ON COLUMN SysUser.LastLoginAt IS '最近登录时间';
/

-- ============================================================
-- 2. 角色表
-- ============================================================
CREATE TABLE SysRole (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysRole PRIMARY KEY,
    RoleCode VARCHAR(64) NOT NULL,
    RoleName VARCHAR(128) NOT NULL,
    Description VARCHAR(512) NOT NULL DEFAULT '',
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysRole_RoleCode UNIQUE (RoleCode)
);
/
COMMENT ON TABLE SysRole IS '角色表';
COMMENT ON COLUMN SysRole.RoleCode IS '角色编码（admin/viewer）';
COMMENT ON COLUMN SysRole.RoleName IS '角色名称';
/

-- ============================================================
-- 3. 用户-角色关联表（多对多，RBAC）
-- ============================================================
CREATE TABLE SysUserRole (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysUserRole PRIMARY KEY,
    UserId BIGINT NOT NULL,
    RoleId BIGINT NOT NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE,
    CONSTRAINT UQ_SysUserRole_User_Role UNIQUE (UserId, RoleId),
    CONSTRAINT FK_SysUserRole_User FOREIGN KEY (UserId) REFERENCES SysUser(Id),
    CONSTRAINT FK_SysUserRole_Role FOREIGN KEY (RoleId) REFERENCES SysRole(Id)
);
/
COMMENT ON TABLE SysUserRole IS '用户角色关联表';
/

-- ============================================================
-- 4. 登录日志表（审计）
-- ============================================================
CREATE TABLE SysLoginLog (
    Id BIGINT IDENTITY(1,1) NOT NULL CONSTRAINT PK_SysLoginLog PRIMARY KEY,
    Username VARCHAR(64) NOT NULL,
    ClientIp VARCHAR(64) NULL,
    UserAgent VARCHAR(512) NULL,
    Result VARCHAR(16) NOT NULL,
    FailReason VARCHAR(256) NULL,
    CreatedAt DATETIME(3) NOT NULL DEFAULT SYSDATE
);
/
COMMENT ON TABLE SysLoginLog IS '登录日志表';
COMMENT ON COLUMN SysLoginLog.Result IS '结果：success / failed';
COMMENT ON COLUMN SysLoginLog.FailReason IS '失败原因';
/

-- 索引
CREATE INDEX IX_SysUser_Status ON SysUser(Status);
/
CREATE INDEX IX_SysUserRole_RoleId ON SysUserRole(RoleId);
/
CREATE INDEX IX_SysLoginLog_CreatedAt ON SysLoginLog(CreatedAt DESC);
/
-- ============================================================
-- 初始数据
-- ============================================================
-- 角色：admin（系统管理员）、viewer（查看者）
INSERT INTO SysRole (RoleCode, RoleName, Description, CreatedAt)
VALUES ('admin', '系统管理员', '拥有用户管理和全部功能权限', SYSDATE);
/
INSERT INTO SysRole (RoleCode, RoleName, Description, CreatedAt)
VALUES ('viewer', '查看者', '仅拥有查看日志与点云数据的权限', SYSDATE);
/

-- 说明：PasswordHash/PasswordSalt 由应用层 PasswordHasher 生成（PBKDF2-SHA256, 100000 迭代）。
-- 占位值：为避免直接执行 SQL 造成弱口令，初始账号由后端启动初始化（SeedDefaultAdmin）创建，
--         本脚本不写入明文口令。若需手动创建，请用后端提供的用户创建接口。

-- 迁移记录
INSERT INTO DatabaseMigration (MigrationId, MigrationName, Version, AppliedAt, AppliedBy, ScriptPath)
VALUES ('20260802_auth_pointcloud_tables', 'v3.0: auth, roles, login log and point cloud file tables', 3, SYSDATE, USER, 'Database/CreateAuthTables.sql');
/
COMMIT;
