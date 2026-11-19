# 三系统麒麟 x86_64 交付说明（2026-09-10）

包含当前工作树代码与今日实测端壁/净宽/底板修复，不是旧Git提交的重新打包。
只交付三个tar.gz、SHA256SUMS和本说明；build/packages/dependency-check是本机构建目录，勿拷贝。

## 环境与交付边界

- x86_64，glibc >= 2.31。先执行 `uname -m`、`getconf GNU_LIBC_VERSION`。
- Worker/WebApi为.NET8自包含Linux发行，不要求目标机另装dotnet。
- Analyzer含Linux CPython3.11、Open3D0.19、NumPy1.26.3、SciPy1.16.3及完整wheelhouse；
  包含Linux dmPython2.5.38及其wheel内的DPI库。安装依赖不访问公网。
- 系统还需libGL.so.1、libgomp.so.1、libstdc++、ICU、OpenSSL等操作系统运行库。
  缺失时从对应麒麟版本离线软件源安装，不能混拷Windows DLL或其他发行版glibc。
- Worker包含厂家Galaxy V1.1.4 Linux SDK及当前桥接源码；桥接库必须在麒麟执行
  `prepare-native.sh`用g++编译（目标机需g++开发环境）。本机构建机是Windows，
  没有将旧桥接二进制冒充当前版本。
- 已做.NET跨平台编译、ELF架构、离线依赖解析、归档校验；未在真实麒麟执行或连接雷达/DM8。

## 解压和准备（不会自动启动、覆盖旧服务或迁移数据库）

1. 先保存旧服务程序/配置的独立备份，记录旧systemd文件；不要删采集目录、队列或数据库。
2. 在最终安装位置解压三个归档。每个归档是独立版本目录：
   `tar -xzf 对应文件.tar.gz`。进入各目录执行 `bash preflight.sh`。
   SHA清单校验必须在修改配置和准备环境之前进行。
3. Analyzer目录执行 `bash prepare-python.sh`；Worker目录执行 `bash prepare-native.sh`。
   Python venv生成后不要移动目录。任一导入失败/ldd缺库均需先解决，不能直接启动业务。
4. 在三个包的 `app/appsettings.json` 中填写现场值。只加载这个文件；环境变量、
   appsettings.Production.json和旧/etc/*.env不能再覆盖业务配置。

## 必填配置

- Analyzer：填写DM8主机/端口/业务账号/密码/Schema，并设置正式B_to_A外参路径。
  allowed root默认为`/data/carpointcloud`，外参、captures、baselines、永久档案均须在其内。
- WebApi：填写`ConnectionStrings:CarPointCloudDb`、`Auth:JwtSecret`（至少32字符）和
  `Integration:AppSecret`。密钥应在现场安全生成，不要使用示例密码。
  `Database:InitializeOnStartup=false`，不自动建表/创建账号；保留已存在账号。
  默认仅监听`127.0.0.1:5031`；现场确认后将`Urls`改为需开放的受控业务网地址。
- Worker：填写CAM_004/CAM_005的实际IP，检查雷达帧参数、路径和采集证据。
  包内是7秒完整场安全示例，并非已经验收的现场参数；两项完整场证据字段为空，
  因而真实采集可能返回`acquisition_unverified`。不能凭此包伪造证据或绕过门禁。
- 固定内部链：Worker不上传/不向合作方取车牌，Analyzer直写DM8；WebApi无合作方。
  车牌由页面先写VehicleArrival传递。Worker 8000、Analyzer 8010仅回环监听。
- 三者永久档案路径一致：`/data/carpointcloud/empty-vehicle-profiles`；不可放captures内。
  空密码/IP是有意留空，未填前启动失败是配置保护，不是缺程序文件。

## 权限、运行和验证

建议复用/创建专用非root服务账号和共享组`carpointcloud-shared`。包内systemd示例使用
`carpointcloud`账号，现有账号不同则相应调整，不直接覆盖旧systemd单元。

由管理员为实际账号建立并授权目录（示例账号不存在时先按现场规则创建）：

```bash
sudo install -d -m 2770 -o carpointcloud -g carpointcloud-shared \
  /data/carpointcloud /data/carpointcloud/captures /data/carpointcloud/baselines \
  /data/carpointcloud/extrinsics /data/carpointcloud/empty-vehicle-profiles \
  /data/carpointcloud/worker-state /data/carpointcloud/worker-state/capture-pairs \
  /data/carpointcloud/worker-state/analysis-queue /data/carpointcloud/worker-state/upload-queue \
  /data/carpointcloud/worker-state/health
```

外参使用现场已验收文件，不用示例矩阵。各包app/appsettings.json应由服务账号可读且
对其他用户不可读（如0640），父目录可遍历。WebApi的`app/logs`需由服务账号可写；
不要以root长期运行服务。三个版本目录代码可保持管理员只写、服务账号只读。

先在WebApi目录执行 `./run.sh --check-startup`（只读配置校验，不连库）。
随后在授权的停机窗口停止旧进程，分别以服务账号执行三个`./run.sh`；启动顺序建议
Analyzer → WebApi → Worker。不要同时运行新旧Worker连接同一对雷达。
需要常驻时，审阅并修改各`*.service.example`里的绝对版本路径/账号后再安装systemd。
包不会自动stop/start/enable任何现有服务。

```bash
curl --fail http://127.0.0.1:8010/health
curl --fail http://127.0.0.1:5031/healthz
curl --fail http://127.0.0.1:8000/api/health
```

最终现场验收还需：DM8可达及账号权限、两台雷达正常出帧/预热、页面显示、点击保存
PCD后实际记录入库。离线编译和HTTP回放不代替该验收。不要自动重放旧dead队列。

回退：停新服务，恢复原服务路径/已备份配置并启动原版本；不删除数据、队列或历史档案。
