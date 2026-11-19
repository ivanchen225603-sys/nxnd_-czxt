# -*- coding: utf-8 -*-
"""真实报文、整圈切帧与 Worker 停稳许可之间的完整性回归。"""
import copy
import tempfile
import unittest

from tests import _helpers as H
from tests._observations import ObservationClock
from app.radar_packet import PacketDecoder
from app.radar_sim import RadarSimulator, box_ground, box_truck
from app.radar_source import LiveHub, LiveSource
from app.service import ChannelWorker, EventBus


class TestLiveFrameIntegrity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, ch, calib, _, angle = H.ctx()
        simulator = RadarSimulator(calib.packet, calib.processing, angle,
            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
            az_step_deg=.4, range_noise_m=0, dropout_prob=0, seed=3)
        cls.good_packets = simulator.scan([
            box_ground(z_floor=calib.detection.ground_z),
            box_truck(dx=2.4, dy=12, dz=2.6, z_floor=calib.detection.ground_z)])
        cls.half_bad_packets = [b[:calib.packet.size // 2] if i % 2 == 0 else b
                                for i, b in enumerate(cls.good_packets)]
        cls.all_bad_packets = [b[:calib.packet.size // 2] for b in cls.good_packets]

    def setUp(self):
        self.clock = ObservationClock(self)
        settings, original, calib, _, angle = H.ctx()
        ch = copy.deepcopy(original)
        ch.mode = "synthetic"
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        ch.abs_baseline_dir = self.temporary.name
        self.worker = ChannelWorker(ch, EventBus(), settings.plate,
                                    save_override=False, plate_mode="null")
        self.worker.source.close()
        decoder = PacketDecoder(calib.packet, calib.processing, angle,
            range_mode=ch.lidar.range_mode, z_down=ch.lidar.z_down,
            max_range_m=ch.lidar.max_range_m)
        self.source = LiveSource(decoder,
            hub=LiveHub(bind_ip="127.0.0.1", udp_port=0), source_ip="127.0.0.2",
            roi=calib.roi, frame_mode="rotation")
        self.source.keep_raw = True
        self.worker.source = self.source
        self.addCleanup(self.source.close)
        self.addCleanup(self.worker.stop)
        self.measurements = []

        def capture(frame, stability, *args):
            # 仅省略重型测量；输入、身份、完整性、停稳、沉降均为真实实现。
            self.measurements.append((self.clock.mono, frame, stability))
            self.worker._detected_this_session += 1
        self.worker._calculate_detection = capture

    def tick(self, packets, dt=.1):
        self.clock.advance(dt)
        if packets is not None:
            # 两圈给真实回绕切帧器一个闭合边界；它只使用最新的完整一圈。
            # 队列交接、解码、ROI 与 Worker.read 均不替换。
            for _ in range(2):
                for packet in packets:
                    self.source.queue.push((packet, self.clock.wall))
        return self.worker._tick()

    def enter_settling(self):
        for _ in range(100):
            self.tick(self.good_packets)
            if self.worker.st.stability.get("settling"):
                self.assertEqual(self.measurements, [])
                return
        self.fail("完整静止新帧未能进入沉降窗口")

    def recover_and_measure(self):
        started = self.clock.mono
        for _ in range(100):
            self.tick(self.good_packets)
            if self.measurements:
                break
        self.assertEqual(len(self.measurements), 1)
        when, frame, stability = self.measurements[0]
        self.assertGreaterEqual(when - started + 1e-9,
                                self.worker.stability.stationary_min_sec + self.worker.settle_sec)
        self.assertLessEqual(when - started, 10.0)
        self.assertEqual(frame.packets, len(self.good_packets))
        self.assertEqual(frame.dropped_packets, 0)
        self.assertTrue(stability.stable)
        self.assertTrue(stability.fresh)
        self.assertFalse(stability.degraded)

    def assert_revoked(self):
        self.assertFalse(self.worker.st.stability["stable"])
        self.assertFalse(self.worker.st.stability.get("settling", False))
        self.assertEqual(len(self.worker._settle_frames), 0)
        self.assertEqual(len(self.worker._recent_frames), 0)
        self.assertEqual(self.measurements, [])

    def test_cold_start_decode_loss_cannot_establish_completeness_reference(self):
        for _ in range(70):
            self.assertTrue(self.tick(self.half_bad_packets))
            self.assert_revoked()
            self.assertEqual(self.worker.st.stability["reason"], "帧不完整")
        frame = self.worker.last_frame
        self.assertGreater(self.worker.stability.vehicle_points(frame),
                           self.worker.stability.vehicle_threshold,
                           "坏帧须仍有足够车辆点，防止以无车碰巧通过用例")
        self.assertEqual((frame.packets, frame.dropped_packets), (450, 450))
        self.assertEqual(len(self.worker._pkt_hist), 0)
        self.assertGreater(self.source.stats.dropped, 0)
        self.recover_and_measure()

    def test_diagnostics_distinguish_learning_decode_loss_and_low_packet_count(self):
        self.tick(self.good_packets)
        row = self.worker._frame_integrity_history[-1]
        self.assertEqual(row['reason'], 'learning')
        self.assertIsNone(row['reference_before'])
        self.assertFalse(row['comparison_applied'])
        for _ in range(5):
            self.tick(self.good_packets)
        row = self.worker._frame_integrity_history[-1]
        self.assertEqual(row['reason'], 'packet_count_sufficient')
        self.assertEqual(row['reference_before'], len(self.good_packets))
        self.assertEqual(row['required_packets'], len(self.good_packets) * .99)
        self.tick(self.half_bad_packets)
        row = self.worker._frame_integrity_history[-1]
        self.assertEqual(row['reason'], 'decode_dropped')
        self.assertEqual(row['received_packets'], len(self.good_packets))
        self.assertEqual(row['decode_dropped'], len(self.good_packets) // 2)
        self.assertFalse(row['comparison_applied'])
        self.assertFalse(row['complete'])
        self.worker._pkt_hist.clear()
        self.worker._pkt_hist.extend([1000] * 31)
        self.tick(self.good_packets)
        row = self.worker._frame_integrity_history[-1]
        self.assertEqual(row['reason'], 'packet_count_below_reference')
        self.assertEqual((row['reference_before'], row['reference_after']), (1000, 1000))
        self.assertEqual(row['required_packets'], 990)
        self.assertTrue(row['comparison_applied'])
        self.assertFalse(row['complete'])
        self.assertEqual(row['seq'], self.worker.last_frame.seq)
        self.assertEqual(row['frame_ts'], self.worker.last_frame.ts)

    def test_frame_diagnostics_are_opt_in_independent_snapshot_data(self):
        self.tick(self.good_packets)
        self.assertNotIn('frame_integrity', self.worker.snapshot(include_history=False))
        snap = self.worker.snapshot(include_history=False, include_frame_integrity=True)
        diag = snap['frame_integrity']
        self.assertEqual(diag['capture_instance_id'], self.worker._capture_instance_id)
        self.assertEqual(diag['reference_packets'], len(self.good_packets))
        self.assertEqual(len(diag['frames']), 1)
        diag['frames'][0]['complete'] = False
        diag['reference_history'].clear()
        self.assertTrue(self.worker._frame_integrity_history[-1]['complete'])
        self.assertEqual(len(self.worker._pkt_hist), 1)

    def test_decode_loss_after_warmup_revokes_window_and_recovers(self):
        self.enter_settling()
        reference = list(self.worker._pkt_hist)
        for _ in range(70):
            self.tick(self.half_bad_packets)
            self.assert_revoked()
        self.assertEqual(list(self.worker._pkt_hist), reference)
        self.recover_and_measure()

    def test_all_bad_packets_revoke_permission_on_the_same_observation(self):
        self.enter_settling()
        self.assertTrue(self.tick(self.all_bad_packets))
        self.assert_revoked()
        frame = self.worker.last_frame
        self.assertEqual((frame.n, frame.packets, frame.dropped_packets), (0, 0, 900))
        self.assertTrue(self.worker.st.stability["degraded"])
        self.assertEqual(self.worker.st.stability["reason"], "帧不完整")
        self.recover_and_measure()

    def test_stream_gap_discards_settle_time_before_recovery(self):
        self.enter_settling()
        for _ in range(10):
            self.tick(self.good_packets)
        self.assertEqual(self.measurements, [])
        self.assertFalse(self.tick(None,
            dt=self.worker.stability.stationary_max_gap_sec + .05))
        self.assert_revoked()
        self.recover_and_measure()


if __name__ == "__main__":
    unittest.main()
