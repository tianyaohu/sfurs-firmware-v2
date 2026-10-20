#!/usr/bin/env python3
"""Exercise the real C IK on the host; no probe, CAN or firmware build."""
import ctypes as ct
import math
import pathlib
import random
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class Wheel(ct.Structure):
    _fields_ = [(name, ct.c_float) for name in
                ('x_m', 'y_m', 'dx', 'dy', 'radius_m', 'gear_ratio',
                 'motor_sign', 'max_motor_rpm')]


def solve3(matrix, rhs):
    """Independent Gaussian solve for the overdetermined forward model."""
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda i: abs(rows[i][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        assert abs(divisor) > 1e-12
        rows[col] = [x / divisor for x in rows[col]]
        for i in range(3):
            if i != col:
                factor = rows[i][col]
                rows[i] = [a - factor * b for a, b in zip(rows[i], rows[col])]
    return [row[3] for row in rows]


class IKTests(unittest.TestCase):
    def geometry(self):
        return (Wheel * 4).from_buffer_copy(self.defaults)

    def run_ik(self, velocity, geometry=None):
        g = self.geometry() if geometry is None else geometry
        out = (ct.c_float * 4)(999, 999, 999, 999)
        scale = ct.c_float(-1)
        ok = self.lib.rover_ik(g, *velocity, out, ct.byref(scale))
        return ok, list(out), scale.value

    def assert_close(self, actual, expected, tolerance=0.0002):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_axes_against_independent_coefficients(self):
        # Independently derived/reviewed coefficients for provisional geometry.
        expected = [(-145.791, 252.517, 25.106),
                    (167.244, 238.850, 24.783),
                    (167.244, -238.850, 24.783),
                    (-145.791, -252.517, 25.106)]
        for axis in range(3):
            velocity = [0, 0, 0]
            velocity[axis] = 0.1
            ok, rpm, scale = self.run_ik(velocity)
            self.assertEqual((ok, scale), (1, 1))
            self.assert_close(rpm, [row[axis] * 0.1 for row in expected])

    def test_zero_and_small_commands_have_no_floor(self):
        self.assertEqual(self.run_ik((0, 0, 0)), (1, [0, 0, 0, 0], 1))
        ok, rpm, _ = self.run_ik((0, 0.001, 0))
        self.assertEqual(ok, 1)
        self.assertTrue(all(0 < abs(r) < 1 for r in rpm))

    def test_mixed_linearity_and_reversal(self):
        a, b = (0.1, -0.12, 0.2), (-0.03, 0.04, -0.1)
        ra, rb = self.run_ik(a)[1], self.run_ik(b)[1]
        combined = self.run_ik(tuple(x + y for x, y in zip(a, b)))[1]
        self.assert_close(combined, [x + y for x, y in zip(ra, rb)])
        self.assert_close(self.run_ik(tuple(-x for x in a))[1], [-x for x in ra])

    def test_uniform_saturation_preserves_twist(self):
        small = self.run_ik((0.1, 0.2, 0.3))[1]
        ok, rpm, scale = self.run_ik((10, 20, 30))
        self.assertEqual(ok, 1)
        self.assertTrue(0 < scale < 1)
        self.assert_close(rpm, [r * 100 * scale for r in small])
        self.assertTrue(all(abs(r) <= w.max_motor_rpm + 0.0001
                            for r, w in zip(rpm, self.geometry())))
        self.assertTrue(any(abs(abs(r) - w.max_motor_rpm) < 0.0001
                            for r, w in zip(rpm, self.geometry())))

    def test_all_four_900_caps_and_tournament_speed_scaling(self):
        self.assertEqual([w.max_motor_rpm for w in self.geometry()], [900] * 4)
        unit = self.run_ik((0, 1, 0))[1]
        for speed in (1, 2, 3):
            ok, rpm, scale = self.run_ik((0, speed, 0))
            self.assertEqual((ok, scale), (1, 1))
            self.assert_close(rpm, [r * speed for r in unit])
        for velocity in ((0, 5, 0), (0, -5, 0), (5, 5, 1), (-5, -5, -1)):
            ok, rpm, scale = self.run_ik(velocity)
            self.assertEqual(ok, 1)
            self.assertTrue(0 < scale < 1)
            self.assertLessEqual(max(map(abs, rpm)), 900.0001)
            self.assertAlmostEqual(max(map(abs, rpm)), 900, delta=0.0001)

    def test_motor_sign_and_gearing_applied_once(self):
        base = self.run_ik((0.05, 0.1, 0.2))[1]
        g = self.geometry()
        g[2].motor_sign = 1
        g[2].gear_ratio = 2
        rpm = self.run_ik((0.05, 0.1, 0.2), g)[1]
        self.assert_close(rpm, [base[0], base[1], -2 * base[2], base[3]])

    def test_forward_reconstruction(self):
        g = self.geometry()
        # Build a forward model using contact rigid-body velocities projected
        # onto rolling vectors. Remove raw motor signs/gearing before solving.
        matrix = [[w.dx, w.dy, -w.y_m * w.dx + w.x_m * w.dy] for w in g]
        normal = [[sum(row[i] * row[j] for row in matrix)
                   for j in range(3)] for i in range(3)]
        rng = random.Random(2026)
        for _ in range(100):
            velocity = [rng.uniform(-2, 2) for _ in range(3)]
            ok, rpm, scale = self.run_ik(velocity)
            self.assertEqual(ok, 1)
            linear = [r * (2 * math.pi * w.radius_m) /
                      (60 * w.gear_ratio * w.motor_sign) for r, w in zip(rpm, g)]
            rhs = [sum(row[i] * v for row, v in zip(matrix, linear))
                   for i in range(3)]
            self.assert_close(solve3(normal, rhs), [v * scale for v in velocity])

    def test_invalid_geometry_and_nonfinite_inputs(self):
        bad_fields = {'radius_m': [0, -1, math.nan, 1e38], 'gear_ratio': [0, math.inf],
                      'motor_sign': [0, 2], 'max_motor_rpm': [0, math.inf],
                      'dx': [0, math.nan], 'x_m': [math.inf]}
        for field, values in bad_fields.items():
            for value in values:
                g = self.geometry()
                setattr(g[3], field, value)
                self.assertEqual(self.run_ik((0.1, 0.2, 0.3), g),
                                 (0, [0, 0, 0, 0], 0))
        for axis in range(3):
            for bad in (math.nan, math.inf, -math.inf, 3.4e38):
                v = [0.1, 0.2, 0.3]
                v[axis] = bad
                self.assertEqual(self.run_ik(v), (0, [0, 0, 0, 0], 0))
        out = (ct.c_float * 4)(999, 999, 999, 999)
        scale = ct.c_float(-1)
        self.assertEqual(self.lib.rover_ik(None, 0, 0, 0, out, ct.byref(scale)), 0)
        self.assertEqual((list(out), scale.value), ([0, 0, 0, 0], 0))
        self.assertEqual(self.lib.rover_ik(self.geometry(), 0, 0, 0, None, None), 0)


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='omni-ik-math-') as tmp:
        library = pathlib.Path(tmp) / 'ik.so'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-shared', '-fPIC', '-I', str(ROOT / 'src/shared/inc'),
                        str(ROOT / 'src/shared/src/rover_kinematics.c'), '-lm',
                        '-o', str(library)], check=True)
        IKTests.lib = ct.CDLL(str(library))
        IKTests.defaults = (Wheel * 4).in_dll(IKTests.lib, 'rover_ik_provisional')
        IKTests.lib.rover_ik.argtypes = [ct.POINTER(Wheel), ct.c_float,
                                       ct.c_float, ct.c_float,
                                       ct.POINTER(ct.c_float), ct.POINTER(ct.c_float)]
        IKTests.lib.rover_ik.restype = ct.c_int
        unittest.main()
