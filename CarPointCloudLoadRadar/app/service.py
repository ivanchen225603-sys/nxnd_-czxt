"""JT128 组合入口：配置、共享接收器、各车道、发布与维护的生命周期。"""
from __future__ import annotations
import time
import threading
from typing import Dict, List, Optional, Tuple
from .settings import load_settings, load_calib, Settings, ChannelCfg
from .radar_source import LiveHub
from .demo_director import DemoDirector
from .result_finalizer import ResultFinalizer
from .application.channel_worker import ChannelWorker as ChannelWorker
from .application.channel_worker import ENTER_THIN_MAX_STEP_M as ENTER_THIN_MAX_STEP_M
from .application.channel_worker import _downsample_xyz as _downsample_xyz
from .application.channel_state import ChannelState as ChannelState, trigger_digest as trigger_digest
from .application.events import EventBus as EventBus
from .adapters.console_logger import ConsoleLogger as ConsoleLogger, attach_console_logger as attach_console_logger
from .detect import StabilityState as StabilityState
from .result_finalizer import MeasurementRejected as MeasurementRejected

class LidarService:
    """
    服务主体：加载配置 → 起每条启用的通道 → 汇总状态给看板。

    · 只跑一次 start()，用 stop() 收尾。
    · snapshot() 是看板的数据源，纯读、无副作用。
    """

    def __init__(self, settings: Optional[Settings] = None,
                 channels_path: str = "", save_override: Optional[bool] = None,
                 plate_mode: str = "", demo: bool = False,
                 only: Optional[List[str]] = None,
                 auto_demo: bool = False, webapi_config: str = "", volume_provider_config: str = ""):
        # 先冻结/校验显式参数，再创建采集源、数据库或后台线程；不热读文件。
        from .volume_provider import VolumeCatalog
        self.volume_catalog = VolumeCatalog.load(volume_provider_config) if volume_provider_config else None
        if self.volume_catalog is not None and not webapi_config:
            raise ValueError("体积参数接入必须显式启用WebApi配置")
        self.st = settings or load_settings(channels_path)
        self.maintenance = None
        if self.st.service.daily_cleanup_at:
            from .storage_maintenance import StorageMaintenance
            self.maintenance = StorageMaintenance(self, self.st.service.daily_cleanup_at)
        self.bus = EventBus()
        self.save_override = save_override
        self.plate_mode = plate_mode
        self.demo = demo
        chans = [c for c in self.st.enabled() if not (only and c.id not in only)]
        self.integration = None
        if webapi_config:
            from .load_radar_transport import BridgeConfig
            from .load_radar_bridge import LoadRadarBridge
            bridge_config = BridgeConfig.load(webapi_config)
            if self.volume_catalog is not None and not bridge_config.enabled:
                raise ValueError("体积参数接入要求WebApi配置已启用")
            if bridge_config.enabled:
                if set(bridge_config.channels) != {c.id for c in chans}:
                    raise ValueError("WebApi 接口通道必须与本次启动通道完全一致")
                self.integration = LoadRadarBridge(bridge_config)
        # ★ 多雷达：只要有 live 通道，就建**一个共享 LiveHub**（一个 socket 收全部，
        #   再按源 IP 分流）。绝不能让每条通道各开一个 socket 抢同一个端口 ——
        #   那样只有一条通道收得到数据，且不报错。详见 radar_source.LiveHub。
        self.hub: Optional[LiveHub] = self._maybe_build_hub(chans)
        self.workers: List[ChannelWorker] = []
        self.started = 0.0
        for i, ch in enumerate(chans):
            # plate_rotate=i：四路仿真车牌错开起点，避免看板上四张卡片同时出同一张牌
            self.workers.append(ChannelWorker(
                ch, self.bus, self.st.plate, save_override, plate_mode, demo,
                hub=self.hub, plate_rotate=i,
                state_push_hz=self.st.service.state_push_hz,
                raw_frames=self.st.service.raw_frames,
                enter_frames=self.st.service.enter_frames,
                enter_stride=self.st.service.enter_stride,
                enter_min_points=self.st.service.enter_min_points,
                enter_step_m=self.st.service.enter_step_m,
                enter_crop_roi=self.st.service.enter_crop_roi,
                enter_volume=self.st.service.enter_volume,
                exit_frames=self.st.service.exit_frames,
                exit_step_m=self.st.service.exit_step_m,
                exit_crop_roi=self.st.service.exit_crop_roi,
                exit_volume=self.st.service.exit_volume,
                integration=self.integration, volume_catalog=self.volume_catalog))
        self.finalizer = ResultFinalizer({w.ch.id: (w.ch.abs_output_dir, w.save_enabled)
                                          for w in self.workers}, self.integration)
        for worker in self.workers:
            worker._finalizer = self.finalizer
            worker._owns_finalizer = False
        baseline_source = next((ch for ch in self.st.channels if ch.id == "CH4"), None)
        if baseline_source is not None:
            for worker in self.workers:
                worker._baseline_dir = baseline_source.abs_baseline_dir
        self.deferred = None
        if self.integration is not None:
            from .deferred_measurements import DeferredMeasurements
            try:
                self.deferred = DeferredMeasurements(self.integration, self.finalizer,
                    [worker.ch.id for worker in self.workers])
            except Exception:
                # 新任务库初始化失败也须释放先前取得的源、队列和进程租约。
                closers = [worker.source.close for worker in self.workers]
                if self.hub is not None:
                    closers.append(self.hub.close)
                closers.extend((self.finalizer.close, self.integration.close))
                for close in closers:
                    try:
                        close()
                    except Exception:
                        pass
                raise
            for worker in self.workers:
                worker._deferred = self.deferred
        # ★ 自动演示编排器：只有显式要求时才起（demo=True 本身不起）。
        #   测试里构造 LidarService(demo=True) 会拿到一个**静止**的场景，
        #   结果可预期；若默认就自转，用例之间会互相踩，测试变成抽奖。
        self.director: Optional[DemoDirector] = (
            DemoDirector(self.workers, bus=self.bus) if auto_demo else None)

    def _maybe_build_hub(self, chans: List[ChannelCfg]) -> Optional[LiveHub]:
        """按本次要跑的通道决定是否建共享 LiveHub（没有 live 通道就不建）。"""
        if self.demo:
            return None                       # 演示全部走仿真源，不需要收包
        live = [c for c in chans
                if (c.mode or "").lower() in ("live", "udp", "real")]
        if not live:
            return None

        # 4 台雷达发往**同一个**端口，这是"单套接字 + 源 IP 分流"的前提。
        ports = sorted({c.lidar.udp_port for c in live})
        if len(ports) > 1:
            raise ValueError(
                f"多雷达必须共用同一个 UDP 端口，但配置里出现了 {ports}。"
                f"本方案靠『同端口 + 源 IP 分流』区分雷达；"
                f"请把所有 live 通道的 lidar.udp_port 统一。")

        first = live[0]
        calib = load_calib(first.abs_config_path)
        bind_ip = first.lidar.bind_ip or calib.network.bind_ip or "0.0.0.0"
        hub = LiveHub(bind_ip=bind_ip, udp_port=ports[0], strict=True)

        # 先替所有 live 通道登记源 IP：这样某台雷达没上线时，看板上能明确
        # 看到"这个 IP 一个包都没收到"，而不是整条通道凭空消失、无从排查。
        seen: Dict[str, str] = {}
        for c in live:
            ip = (c.lidar.source_ip or c.lidar.ip or "").strip()
            if not ip:
                raise ValueError(
                    f"{c.id} 是 live 通道，但没有 lidar.source_ip / lidar.ip —— "
                    f"多雷达必须一机一 IP 才能分流。")
            if ip in seen:
                raise ValueError(
                    f"{c.id} 与 {seen[ip]} 的 source_ip 都是 {ip}；"
                    f"多雷达必须一机一 IP。")
            seen[ip] = c.id
            hub.register(ip, c.id)
        return hub

    # ── 生命周期 ────────────────────────────────────────────
    def start(self) -> None:
        self.started = time.time()
        self.finalizer.start()
        if self.integration is not None:
            self.integration.start()
        if self.deferred is not None:
            self.deferred.start()
        if self.hub is not None:
            self.hub.start()               # 先起接收线程，worker 才有数据可取
        self.bus.publish("service_start", {
            "channels": [w.ch.id for w in self.workers],
            "hub": (f"{self.hub.bind_ip}:{self.hub.udp_port}"
                    if self.hub is not None else ""),
            "sources": self.hub.source_ips() if self.hub is not None else [],
            "channels_path": self.st.channels_path})
        for w in self.workers:
            w.start()
        # 编排器必须在 worker 起来之后再动数据源（否则第一帧就可能被写花）
        if self.director is not None:
            self.director.start()
        if self.maintenance is not None:
            self.maintenance.start()

    def stop(self, timeout: float = 3.0) -> bool:
        """停服务。**任何一步失败都不能阻止后面的清理** —— 尤其是 hub.close()：
        它持有那个唯一的 UDP socket，不关掉的话下一次启动会 EADDRINUSE。"""
        pending_stop = getattr(self, "_pending_integration_stop", None)
        if pending_stop is not None and pending_stop.is_alive():
            return False
        maintenance = getattr(self, "maintenance", None)
        maintenance_stopped = maintenance is None or maintenance.close(timeout)
        errs: List[str] = []
        if self.director is not None:      # 先停编排器，别再往数据源写位置
            try:
                self.director.stop()
            except Exception as e:                          # noqa: BLE001
                errs.append(f"director stop: {type(e).__name__}: {e}")
        for w in self.workers:
            try:
                w.stop()
            except Exception as e:                          # noqa: BLE001
                errs.append(f"{w.ch.id} stop: {type(e).__name__}: {e}")
        for w in self.workers:
            try:
                w.join(timeout=timeout)
            except Exception as e:                          # noqa: BLE001
                errs.append(f"{w.ch.id} join: {type(e).__name__}: {e}")
        if self.hub is not None:           # worker 全停后再关 hub（队列不再有人读）
            try:
                self.hub.close()
            except Exception as e:                          # noqa: BLE001
                errs.append(f"hub close: {type(e).__name__}: {e}")
        finalizer = getattr(self, "finalizer", None)
        deferred = getattr(self, "deferred", None)
        if (not maintenance_stopped or any(w.is_alive() for w in self.workers)
                or deferred is not None and not deferred.close(timeout)
                or finalizer is not None and not finalizer.close(timeout)):
            # 最后一次测量可能仍在计算或写盘，不能先关闭队列并释放单进程锁。
            self.bus.publish("error", {"channel": "", "error": "检测仍在收尾，队列和进程锁保持占用"})
            self._pending_integration_stop = threading.Thread(
                target=self._finish_integration_stop, name="load-radar-drain", daemon=False)
            self._pending_integration_stop.start()
            return False
        if self.integration is not None:
            try:
                self.integration.close()
            except Exception as e:
                errs.append("接口停止失败: " + type(e).__name__)
        for worker in self.workers:
            if hasattr(worker, "_drain_finalizer"):
                worker._drain_finalizer()
        if errs:
            self.bus.publish("error", {"channel": "", "error": "停止时出错: " + "; ".join(errs)})
        self.bus.publish("service_stop", {})
        return True

    def _finish_integration_stop(self) -> None:
        """非守护收尾线程保证正常退出等待最后一次检测，不提前释放持久队列。"""
        for worker in self.workers:
            worker.join()
        try:
            maintenance = getattr(self, "maintenance", None)
            if maintenance is not None:
                maintenance.close(None)
            deferred = getattr(self, "deferred", None)
            if deferred is not None:
                deferred.close(None)
            finalizer = getattr(self, "finalizer", None)
            if finalizer is not None:
                finalizer.close(None)
            for worker in self.workers:
                if hasattr(worker, "_drain_finalizer"):
                    worker._drain_finalizer()
            if self.integration is not None:
                self.integration.close()
            self.bus.publish("service_stop", {})
        except Exception as exc:
            self.bus.publish("error", {"channel": "", "error": "接口收尾失败: " + type(exc).__name__})

    # ── 查询 ────────────────────────────────────────────────
    def snapshot(self, include_history: bool = True, include_frame_integrity: bool = False) -> Dict:
        chans = [w.snapshot(include_history, include_frame_integrity=include_frame_integrity)
                 for w in self.workers]
        total_saved = sum(c["saved_count"] for c in chans)
        # 把编排器当前的"剧本进度"贴到每路通道上，看板据此显示
        # 「车辆驶入 / 停稳测量…」这类现场感的信息。
        if self.director is not None:
            for c in chans:
                row = self.director.snapshot_row(c["id"])
                if row:
                    c["demo"] = row
        # 判定门槛：看板要拿它跟实测值并排显示（低于门槛标红），
        # 所以跟着快照一起发出去，前端不用再请求一次。
        gate = {}
        if self.workers:
            g = self.workers[0].dev.gate
            gate = {"distance_m": g.distance_m, "normal_z_min": g.normal_z_min,
                    "z_above_min_max_m": g.z_above_min_max_m,
                    "min_inliers": g.min_inliers, "min_inlier_ratio": g.min_inlier_ratio,
                    "min_span_x_m": g.min_span_x_m, "min_span_y_m": g.min_span_y_m}
        return {
            "service": {
                "uptime": round(time.time() - self.started, 1) if self.started else 0.0,
                # ★ 服务端的"现在"。看板用它算"这条结论是多久以前的"——
                #   不能用浏览器本地时间：看板常常开在另一台机器上，
                #   两边时钟差几分钟的话，算出来的"几秒前"会是负数或离谱的值。
                "now": round(time.time(), 3),
                # 原厂 devices.json 的 calibrate.result_hold_sec（默认 8s）：
                # 结论超过这个岁数就不再算"刚出的"，看板会标成历史结论。
                "result_hold_sec": self.st.stability.result_hold_sec,
                "channels": len(chans),
                "online": sum(1 for c in chans if c["online"]),
                "saved_total": total_saved,
                "subscribers": self.bus.subscribers,
                "host": self.st.service.host,
                "port": self.st.service.port,
                "channels_path": self.st.channels_path,
                "demo": self.demo,
                # 仿真路数 / 真雷达路数：看板顶部据此给出"这是演示数据"的明确提示，
                # 免得领导看到满屏数据以为已经接了硬件。
                "sim_channels": sum(1 for c in chans if c["source_kind"] == "sim-radar"),
                "live_channels": sum(1 for c in chans if c["source_kind"] == "live"),
            },
            "gate": gate,
            "channels": chans,
            # 多雷达共享接收点的统计：总包率、各源 IP 的包率/队列积压/丢包。
            # 现场一眼就能看出"哪台雷达没上线"或"哪条通道解码跟不上"。
            "hub": self.hub.to_dict() if self.hub is not None else None,
            # 自动演示编排器的状态（没开时为 None）
            "director": self.director.status() if self.director is not None else None,
            "integration": self.integration.snapshot() if self.integration is not None else {"enabled": False},
        }

    def by_id(self, cid: str) -> Optional[ChannelWorker]:
        for w in self.workers:
            if w.ch.id == cid:
                return w
        return None

    def capture_baseline(self, cid: str, plate: str = "") -> Tuple[bool, str]:
        """在 cid 这条道上标定一辆空车的基线。基线**按车牌**存进共用目录，
        之后这辆车装货走任意一条道都能查到（见 ChannelWorker 构造里的说明）。"""
        w = self.by_id(cid)
        if w is None:
            return False, f"没有通道 {cid}"
        return w.capture_baseline(plate)

    def set_loaded_demo(self, cid: str, loaded: bool) -> bool:
        """
        看板「演示：空车/载货」按钮的后端。

        会**暂停自动编排**（若在跑）：手动点一下就该持续有效，
        否则编排器下一秒就把场景覆盖回去，用户会以为按钮坏了。
        """
        w = self.by_id(cid)
        if w is not None and self.director is not None:
            self.director.pause("看板手动切换了演示场景")
        ok = bool(w and w.set_vehicle_demo(loaded=bool(loaded)))
        return ok

    def force_new_session(self, cid: str = "") -> int:
        """强制开新会话（重测一次）。cid 为空则对所有通道生效。

        同样会暂停自动编排 —— 手工重测后应当由人接着操作。
        """
        n = 0
        targets = [w for w in self.workers if not cid or w.ch.id == cid]
        if targets and self.director is not None:
            self.director.pause("看板手动触发了重测")
        for w in targets:
            if w.force_new_session():
                n += 1
        return n

    def set_auto_demo(self, on: bool) -> bool:
        """开关自动演示编排。没有编排器（非演示启动）时返回 False。"""
        if self.director is None:
            return False
        self.director.set_enabled(bool(on))
        return True
