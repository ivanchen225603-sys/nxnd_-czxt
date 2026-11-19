"""独立标签和物理平面真值；不使用被测 ROI 产生答案。"""
import unittest

import numpy as np

from app.cloud_roi import RoiLimits, extract_observed_roi
from app.pointcloud import Frame


def labelled_scene(width=2.4, length=10., offset=.0, yaw=0., noise=0.,
                   cab=False, neighbor=False, loaded=False, missing_front=0., seed=5):
    rng = np.random.default_rng(seed)
    xs = np.linspace(-width / 2, width / 2, int(width / .10) + 1)
    ys = np.linspace(-length / 2, length / 2, int(length / .10) + 1)
    zs = np.linspace(1., 3., 21)
    parts = []

    def face(a, b, axis, fixed):
        u, v = np.meshgrid(a, b)
        p = np.empty((u.size, 3))
        axes = [x for x in range(3) if x != axis]
        p[:, axis] = fixed
        p[:, axes[0]] = u.ravel()
        p[:, axes[1]] = v.ravel()
        return p

    parts.append(face(xs, ys, 2, 1.))
    parts.extend(face(ys, zs, 0, x) for x in (-width / 2, width / 2))
    parts.extend(face(xs, zs, 1, y) for y in (-length / 2, length / 2))
    if loaded:
        parts.append(face(xs, ys, 2, 1.8))
    box = np.concatenate(parts)
    if missing_front:
        box = box[box[:, 1] < length / 2 - missing_front]
    truck, label = [box], [np.zeros(len(box), np.int64)]
    if cab:
        # 低于厢底的可见底盘连接，不能仅凭空间接近把独立物体并入车辆。
        chassis = face(np.linspace(-.4, .4, 9), np.linspace(length / 2, length / 2 + .6, 7), 2, .6)
        cy = np.linspace(length / 2 + .5, length / 2 + 2.5, 21)
        cx = np.linspace(-1.1, 1.1, 23)
        cz = np.linspace(.6, 2., 15)
        cabin = np.concatenate([chassis, face(cx, cy, 2, 2.),
                                face(cy, cz, 0, -1.1), face(cy, cz, 0, 1.1),
                                face(cx, cz, 1, cy[-1])])
        truck.append(cabin)
        label.append(np.ones(len(cabin), np.int64))
    vehicle = np.concatenate(truck)
    labels = np.concatenate(label)
    theta = np.deg2rad(yaw)
    transform = np.array([[np.cos(theta), -np.sin(theta), 0.],
                          [np.sin(theta), np.cos(theta), 0.], [0., 0., 1.]])
    vehicle = vehicle @ transform.T + [offset, 0., 0.]
    if noise:
        vehicle += rng.normal(0, noise, vehicle.shape)
    ground = face(np.linspace(-5, 5, 31), np.linspace(-12, 12, 61), 2, 0.)
    xyz = [vehicle, ground]
    labels = [labels, np.full(len(ground), 2, np.int64)]
    if neighbor:
        other = np.concatenate(parts) + [4., 0., 0.]
        xyz.append(other)
        labels.append(np.full(len(other), 3, np.int64))
    xyz, labels = np.concatenate(xyz), np.concatenate(labels)
    order = rng.permutation(len(xyz))
    xyz, labels = xyz[order], labels[order]
    frame = Frame(xyz, np.arange(len(xyz), dtype=np.uint16), seq=10, ts=1000., packets=450)
    centers = {"left": [-width / 2, 0., 2.], "right": [width / 2, 0., 2.],
               "rear": [0., -length / 2, 2.], "front": [0., length / 2, 2.]}
    truth = {name: np.array(value) @ transform.T + [offset, 0., 0.] for name, value in centers.items()}
    return frame, labels, truth, transform


class TestObservedRoi(unittest.TestCase):
    def extract(self, frame, **kwargs):
        return extract_observed_roi(frame, ((-2., -11., -1.), (2., 11., 5.)), 0., **kwargs)

    def assert_complete(self, frame, labels, truth, transform, tolerance=.06):
        result = self.extract(frame)
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertTrue(np.all(labels[result.vehicle_indices] <= 1))
        self.assertEqual(int((labels <= 1).sum()), len(result.vehicle_indices))
        self.assertTrue(np.all(labels[result.carriage_indices] == 0), "驾驶室/环境混入车厢")
        self.assertEqual(int((labels == 0).sum()), len(result.carriage_indices), "车厢实测点被静默切掉")
        matrix = np.array(result.report["analysis_transform"]["R"])
        np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-10)
        self.assertLess(np.max(np.abs(matrix @ transform - np.eye(3))), .01)
        for name, axis in (("left", 0), ("right", 0), ("rear", 1), ("front", 1)):
            boundary = result.report["boundaries"][name]
            self.assertEqual("supported" if axis == 0 else "surface_supported", boundary["status"])
            expected = (truth[name] @ matrix.T)[axis]
            candidate = boundary["value_m"] if axis == 0 else boundary["candidate_m"]
            if axis == 1:
                self.assertIsNone(boundary["value_m"], "几何候选不能冒充已确认物理端点")
            self.assertLessEqual(abs(candidate - expected), tolerance, name)
        return result

    def test_complete_empty_and_loaded_are_usable_positive_controls(self):
        for loaded in (False, True):
            with self.subTest(loaded=loaded):
                self.assert_complete(*labelled_scene(loaded=loaded))

    def test_width_length_and_offset_are_not_fixed_to_one_truck(self):
        for width, length, offset in ((2.2, 6., -.8), (2.45, 10., .8), (2.8, 12., .6)):
            with self.subTest(width=width, length=length, offset=offset):
                self.assert_complete(*labelled_scene(width, length, offset))

    def test_small_yaw_and_noise_boundary_error_against_independent_planes(self):
        for yaw, offset in ((-3., -.8), (3., .8), (1.2, .6)):
            with self.subTest(yaw=yaw, offset=offset):
                self.assert_complete(*labelled_scene(yaw=yaw, offset=offset, noise=.003))

    def test_cab_belongs_to_vehicle_but_not_carriage(self):
        self.assert_complete(*labelled_scene(cab=True))

    def test_adjacent_vehicle_is_not_selected_by_greater_point_count(self):
        self.assert_complete(*labelled_scene(neighbor=True))

    def test_equal_candidates_inside_seed_are_ambiguous(self):
        frame, labels, _, _ = labelled_scene()
        points = frame.xyz[labels == 0]
        twin = Frame(np.concatenate([points + [-2., 0., 0.], points + [2., 0., 0.]]))
        result = extract_observed_roi(twin, ((-3., -11., -1.), (3., 11., 5.)), 0.)
        self.assertEqual("ambiguous", result.report["vehicle_status"])
        self.assertEqual(0, len(result.carriage_indices))

    def test_missing_front_remains_unsupported_even_when_repeated(self):
        for loaded in (False, True):
            frame, labels, _, _ = labelled_scene(missing_front=1., loaded=loaded)
            for _ in range(3):
                result = self.extract(frame)
                self.assertEqual("partial", result.report["carriage_status"], result.report)
                self.assertEqual("unsupported", result.report["boundaries"]["front"]["status"])
                self.assertIsNone(result.report["boundaries"]["front"]["value_m"])
                self.assertTrue(np.all(labels[result.carriage_indices] == 0))

    def test_top_plane_and_two_narrow_strips_cannot_become_carriage(self):
        frame, labels, _, _ = labelled_scene()
        vehicle = frame.xyz[labels == 0]
        for cloud in (vehicle[vehicle[:, 2] == 1.], vehicle[np.abs(vehicle[:, 0]) > 1.19]):
            result = self.extract(Frame(cloud))
            self.assertNotEqual("available", result.report["carriage_status"])
            self.assertEqual(0, len(result.carriage_indices))

    def test_known_input_clipping_is_reported_not_physical_front(self):
        frame, _, _, _ = labelled_scene()
        clipped = frame._mask(frame.xyz[:, 1] <= 4.)
        result = self.extract(clipped, capture_bounds=((-5., -12., -1.), (5., 4., 5.)))
        self.assertEqual("capture_clipped", result.report["boundaries"]["front"]["status"])
        self.assertIsNone(result.report["boundaries"]["front"]["value_m"])

    def test_original_order_duplicate_xyz_and_distinct_rings_survive(self):
        frame, labels, truth, transform = labelled_scene()
        duplicate = np.flatnonzero(labels == 0)[:50]
        frame = Frame(np.concatenate([frame.xyz, frame.xyz[duplicate]]),
                      np.concatenate([frame.ring, frame.ring[duplicate] + 30000]), seq=10, ts=1000.)
        labels = np.concatenate([labels, labels[duplicate]])
        result = self.assert_complete(frame, labels, truth, transform)
        self.assertTrue(np.all(np.diff(result.carriage_indices) > 0))
        self.assertTrue(np.all(np.isin(np.arange(frame.n - 50, frame.n), result.carriage_indices)))
        np.testing.assert_array_equal(frame._mask(result.carriage_indices).ring,
                                      frame.ring[result.carriage_indices])

    def test_density_change_does_not_shrink_to_densest_center(self):
        frame, labels, truth, transform = labelled_scene()
        center = np.flatnonzero((labels == 0) & (np.abs(frame.xyz[:, 1]) < 1.))
        frame = Frame(np.concatenate([frame.xyz] + [frame.xyz[center]] * 5), seq=10, ts=1000.)
        labels = np.concatenate([labels] + [labels[center]] * 5)
        self.assert_complete(frame, labels, truth, transform)

    def test_no_vehicle_and_invalid_limits_do_not_invent_roi(self):
        result = self.extract(Frame(np.zeros((1000, 3))))
        self.assertEqual("unavailable", result.report["vehicle_status"])
        with self.assertRaises(ValueError):
            self.extract(Frame(np.zeros((1000, 3))), limits=RoiLimits(cell_m=0))


def random_panel(rng, axis, fixed, lo, hi, count):
    points = rng.uniform(lo, hi, (count, 3))
    points[:, axis] = fixed
    return points


def random_body(rng, lo=(-1.2, -5., 1.), hi=(1.2, 5., 3.), counts=(8000, 4000, 2000)):
    # 与规则网格生成器独立；真值由指定物理平面直接给出。
    parts = [random_panel(rng, 2, lo[2], lo, hi, counts[0])]
    parts += [random_panel(rng, 0, x, lo, hi, counts[1]) for x in (lo[0], hi[0])]
    parts += [random_panel(rng, 1, y, lo, hi, counts[2]) for y in (lo[1], hi[1])]
    return np.concatenate(parts)


class TestIndependentRoiCounterexamples(unittest.TestCase):
    def setUp(self):
        self.rng = np.random.default_rng(901703)
        self.body = random_body(self.rng)

    @staticmethod
    def extract(points):
        return extract_observed_roi(Frame(points, seq=1, ts=1000.),
                                    ((-2., -11., -1.), (2., 11., 5.)), 0.)

    def test_random_surfaces_are_positive_control(self):
        result = self.extract(self.body)
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual(len(self.body), len(result.carriage_indices))

    def test_wall_fifteen_cm_away_is_not_silently_a_truck_side(self):
        wall = random_panel(self.rng, 0, 1.35, [1.35, -6., .6], [1.35, 6., 3.3], 12000)
        result = self.extract(np.concatenate([self.body, wall]))
        self.assertEqual("ambiguous", result.report["vehicle_status"], result.report)
        self.assertEqual(0, len(result.carriage_indices))

    def test_neighbor_with_ten_cm_gap_is_not_one_vehicle(self):
        neighbor = random_body(self.rng, lo=(1.3, -5., 1.), hi=(3.7, 5., 3.))
        result = self.extract(np.concatenate([self.body, neighbor]))
        self.assertEqual("ambiguous", result.report["vehicle_status"], result.report)
        self.assertEqual(0, len(result.vehicle_indices))

    def test_same_width_same_height_cab_is_split_at_observed_internal_bulkhead(self):
        cab = random_body(self.rng, lo=(-1.2, 5.3, .6), hi=(1.2, 7., 3.), counts=(2500, 1200, 900))
        chassis = random_panel(self.rng, 2, .6, [-.3, 5., .6], [.3, 5.35, .6], 600)
        all_points = np.concatenate([self.body, cab, chassis])
        result = self.extract(all_points)
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual(len(self.body), len(result.carriage_indices))
        self.assertTrue(np.all(result.carriage_indices < len(self.body)), "混入同高驾驶室/底盘")
        self.assertAlmostEqual(5., result.report["boundaries"]["front"]["candidate_m"], delta=.05)
        self.assertIsNone(result.report["boundaries"]["front"]["value_m"])

    def test_low_cargo_vertical_face_cannot_prove_hidden_front_boundary(self):
        seen = self.body[self.body[:, 1] <= 4.]
        cargo = random_panel(self.rng, 1, 4., [-1.2, 4., 1.], [1.2, 4., 2.], 2000)
        result = self.extract(np.concatenate([seen, cargo]))
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual("unsupported", result.report["boundaries"]["front"]["status"])
        self.assertIsNone(result.report["boundaries"]["front"]["value_m"])

    def test_dense_center_cannot_hide_existing_walls(self):
        rng = np.random.default_rng(991703)
        body = random_body(rng)
        center = random_panel(rng, 2, 1., [-.45, -1., 1.], [.45, 1., 1.], 200000)
        result = self.extract(np.concatenate([body, center]))
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual(len(body) + len(center), len(result.carriage_indices))

    def test_coal_above_sidewalls_is_not_silently_discarded(self):
        coal = random_panel(self.rng, 2, 3.6, [-1., -4., 3.6], [1., 4., 3.6], 8000)
        result = self.extract(np.concatenate([self.body, coal]))
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual(len(self.body) + len(coal), len(result.carriage_indices))

    def test_internal_full_height_divider_cannot_shorten_carriage(self):
        divider = random_panel(self.rng, 1, 4.5, [-1.2, 4.5, 1.], [1.2, 4.5, 3.], 2500)
        result = self.extract(np.concatenate([self.body, divider]))
        self.assertEqual("ambiguous", result.report["carriage_status"], result.report)
        self.assertEqual(0, len(result.carriage_indices))

    def test_short_external_wall_is_not_part_of_vehicle(self):
        wall = random_panel(self.rng, 0, 1.35, [1.35, -1., .6], [1.35, 1.5, 3.3], 1000)
        result = self.extract(np.concatenate([self.body, wall]))
        self.assertEqual("ambiguous", result.report["vehicle_status"], result.report)
        self.assertEqual(0, len(result.vehicle_indices))

    def test_small_positive_yaw_search_limit_is_valid(self):
        result = extract_observed_roi(Frame(self.body), ((-2., -11., -1.), (2., 11., 5.)),
                                      0., limits=RoiLimits(max_yaw_deg=.05))
        self.assertEqual("partial", result.report["carriage_status"], result.report)

    def test_dense_sidewalls_cannot_prove_cargo_face_reaches_carriage_top(self):
        for side_count, cargo_top in ((50000, 2.4), (100000, 2.7)):
            with self.subTest(side_count=side_count):
                body = random_body(self.rng, counts=(8000, side_count, 2000))
                seen = body[body[:, 1] <= 4.]
                cargo = random_panel(self.rng, 1, 4., [-1.2, 4., 1.], [1.2, 4., cargo_top], 500)
                result = self.extract(np.concatenate([seen, cargo]))
                self.assertEqual("partial", result.report["carriage_status"], result.report)
                self.assertEqual("unsupported", result.report["boundaries"]["front"]["status"])

    def test_adversarial_array_order_does_not_remove_wall_samples(self):
        points = random_body(self.rng, counts=(100000, 20000, 10000))
        selected = np.linspace(0, len(points) - 1, 60000).astype(int)
        order = np.empty(len(points), int)
        order[selected] = np.arange(len(selected))
        rest = np.ones(len(points), bool)
        rest[selected] = False
        order[rest] = np.arange(len(selected), len(points))
        for p in (points[self.rng.permutation(len(points))], points[order]):
            result = self.extract(p)
            self.assertEqual("partial", result.report["carriage_status"], result.report)
            self.assertEqual(len(points), len(result.carriage_indices))

    def test_actual_yaw_must_respect_declared_limit(self):
        for angle, available in ((5.9, True), (6.1, False), (6.4, False)):
            rad = np.deg2rad(angle)
            matrix = np.array([[np.cos(rad), -np.sin(rad), 0.], [np.sin(rad), np.cos(rad), 0.], [0., 0., 1.]])
            result = self.extract(self.body @ matrix.T)
            self.assertEqual(available, result.report["carriage_status"] == "partial", result.report)

    def test_full_height_cargo_cut_is_not_proven_physical_front(self):
        seen = self.body[self.body[:, 1] <= 4.]
        cut = random_panel(self.rng, 1, 4., [-1.2, 4., 1.], [1.2, 4., 3.], 3000)
        result = self.extract(np.concatenate([seen, cut]))
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual("surface_supported", result.report["boundaries"]["front"]["status"])
        self.assertIsNone(result.report["boundaries"]["front"]["value_m"])
        self.assertAlmostEqual(4., result.report["boundaries"]["front"]["candidate_m"], delta=.05)
        self.assertEqual(len(seen) + len(cut), len(result.carriage_indices))

    def test_high_only_sidewalls_do_not_erase_visible_floor(self):
        floor = random_panel(self.rng, 2, 1., [-1.2, -5., 1.], [1.2, 5., 1.], 8000)
        sides = [random_panel(self.rng, 0, x, [x, -5., 2.], [x, 5., 3.], 50000) for x in (-1.2, 1.2)]
        ends = [random_panel(self.rng, 1, y, [-1.2, y, 1.], [1.2, y, 3.], 2000) for y in (-5., 5.)]
        points = np.concatenate([floor] + sides + ends)
        result = self.extract(points)
        self.assertEqual("partial", result.report["carriage_status"], result.report)
        self.assertEqual(len(points), len(result.carriage_indices))
        self.assertTrue(np.all(np.isin(np.arange(len(floor)), result.carriage_indices)))


if __name__ == "__main__":
    unittest.main()
