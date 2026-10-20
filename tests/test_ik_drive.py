#!/usr/bin/env python3
"""Compile the real streaming IK drive (ik_drive.c + IK) with a fake CAN bus."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "ik_drive.h"
#include "can_datatypes.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static unsigned count, fail;
static uint32_t ids[4096]; static uint8_t payload[4096][4];
static int send(uint32_t id, const uint8_t d[4]) {
  if (fail) return 0;
  assert(count < 4096); ids[count] = id; memcpy(payload[count++], d, 4); return 1;
}
static uint32_t now;
static void run(unsigned ms, int beat) {
  for (unsigned i = 0; i < ms; ++i) { if (beat && now % 50 == 0) ++ik_drive_heartbeat; ik_drive_tick(now++, send); }
}
static int rpm_of(unsigned k) { return (int16_t)(payload[k][0] | payload[k][1] << 8); }
/* Last SPEED_WRITE rpm sent to wheel i (selector 1<<i), or 99999. */
static int last_rpm(unsigned i) {
  for (unsigned k = count; k-- > 0;)
    if (ids[k] == (SPEED_WRITE | (1U << i))) return rpm_of(k);
  return 99999;
}
static unsigned count_id(uint32_t id) { unsigned n = 0; for (unsigned k = 0; k < count; ++k) n += ids[k] == id; return n; }
static const int fwd1[4] = {253, 239, -239, -253}; /* 1 m/s forward, raw motor RPM */

int main(int argc, char **argv) {
  assert(argc == 2);
  const int scenario = atoi(argv[1]);
  if (scenario == 0) { /* nothing without enable + heartbeat */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
    run(1000, 0); assert(count == 0 && ik_drive_state == IK_STOPPED); /* no heartbeat yet */
    ik_drive_enable = 0; run(1000, 1); assert(count == 0); /* heartbeat, not enabled */
  } else if (scenario == 1) { /* stream at 50 Hz, accelerate along the twist, reach target */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
    run(20, 1); assert(ik_drive_state == IK_DRIVING && count == 4);
    for (unsigned i = 0; i < 4; ++i) {
      assert(ids[i] == (SPEED_WRITE | (1U << i)) && payload[i][2] == 20 && payload[i][3] == 0);
      /* first step: largest wheel moves 900 RPM/s * 20 ms = 18 RPM, others in proportion */
      assert(abs(rpm_of(i)) <= 18 && abs(rpm_of(i)) >= 16);
    }
    run(1000, 1);
    unsigned frames = count_id(SPEED_WRITE | 1U); assert(frames >= 50 && frames <= 52);
    for (unsigned i = 0; i < 4; ++i) assert(last_rpm(i) == fwd1[i] && ik_drive_target_rpm[i] == fwd1[i]);
  } else if (scenario == 2) { /* release: decelerate, hold, then 3 STOP broadcasts */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
    ik_drive_enable = 0; unsigned mark = count;
    run(320, 1); assert(ik_drive_state == IK_DRIVING && last_rpm(0) == 0); /* 253 RPM at 900 RPM/s */
    for (unsigned k = mark; k < count; ++k) assert(ids[k] != (MOTOR_STOP | 0xFU));
    run(400, 1); assert(ik_drive_state == IK_STOPPED && count_id(MOTOR_STOP | 0xFU) == 3);
    mark = count; run(500, 1); assert(count == mark); /* quiet once stopped */
  } else if (scenario == 3) { /* host dies: heartbeat ages out -> same brake + STOP */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
    run(240, 0); assert(last_rpm(0) == fwd1[0]); /* last beat at t=950: fresh until 1250 */
    run(1000, 0); assert(ik_drive_state == IK_STOPPED && count_id(MOTOR_STOP | 0xFU) == 3);
  } else if (scenario == 4) { /* reverse direction without stopping in between */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(1000, 1);
    ik_drive_vy_mmps = -1000; run(1000, 1);
    for (unsigned i = 0; i < 4; ++i) assert(last_rpm(i) == -fwd1[i]);
    assert(count_id(MOTOR_STOP | 0xFU) == 0);
  } else if (scenario == 5) { /* enable with zero twist holds zero, no STOP */
    ik_drive_enable = 1; run(1000, 1);
    assert(ik_drive_state == IK_DRIVING && last_rpm(2) == 0 && count_id(MOTOR_STOP | 0xFU) == 0);
  } else if (scenario == 6) { /* out-of-range input drives zero and counts */
    ik_drive_vx_mmps = 5001; ik_drive_enable = 1; run(200, 1);
    assert(ik_drive_bad_input > 0 && last_rpm(0) == 0);
    ik_drive_vx_mmps = 0; ik_drive_vy_mmps = 5000; run(3000, 1);
    const int sat[4] = {900, 851, -851, -900};
    for (unsigned i = 0; i < 4; ++i) assert(last_rpm(i) == sat[i]);
  } else if (scenario == 7) { /* raw frames only while stopped */
    ik_drive_raw_id = HALL_ALIGN | 1U; ik_drive_raw_data = HALL_ALIGN_MAGIC; ik_drive_raw_seq = 1;
    run(1, 0); assert(count == 1 && ids[0] == (HALL_ALIGN | 1U) && ik_drive_raw_done == 1);
    assert(memcmp(payload[0], "HALN", 4) == 0);
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1; run(100, 1);
    ik_drive_raw_seq = 2; run(100, 1); assert(ik_drive_raw_done == 1 && count_id(HALL_ALIGN | 1U) == 1);
  } else if (scenario == 8) { /* TX failures are counted, never fatal */
    ik_drive_vy_mmps = 1000; ik_drive_enable = 1; fail = 1; run(100, 1);
    assert(ik_drive_tx_fail >= 16 && ik_drive_state == IK_DRIVING);
    fail = 0; run(1000, 1); assert(last_rpm(0) == fwd1[0]);
  } else if (scenario == 9) { /* custom acceleration limit */
    ik_drive_accel_rpmps = 2530; ik_drive_vy_mmps = 1000; ik_drive_enable = 1;
    run(120, 1); assert(last_rpm(0) == fwd1[0]); /* 253 RPM in 100 ms */
  }
  return 0;
}
'''


class IkDriveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        tmp = pathlib.Path(cls.tmp.name)
        (tmp / "test.c").write_text(HARNESS)
        cls.exe = tmp / "test"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-I", str(ROOT / "src/nucleo/CM7/Core/Inc"), "-I", str(ROOT / "src/shared/inc"),
                        "-I", str(ROOT / "src/shared/can"),
                        str(tmp / "test.c"), str(ROOT / "src/nucleo/CM7/Core/Src/ik_drive.c"),
                        str(ROOT / "src/shared/src/rover_kinematics.c"),
                        "-lm", "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def check(self, n):
        r = subprocess.run([str(self.exe), str(n)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_needs_enable_and_heartbeat(self): self.check(0)
    def test_streams_and_slews_to_target(self): self.check(1)
    def test_release_brakes_then_stops(self): self.check(2)
    def test_lost_heartbeat_brakes_then_stops(self): self.check(3)
    def test_reverses_without_stop(self): self.check(4)
    def test_zero_twist_holds(self): self.check(5)
    def test_bad_input_and_saturation(self): self.check(6)
    def test_raw_frames_only_when_stopped(self): self.check(7)
    def test_tx_failures_counted(self): self.check(8)
    def test_custom_acceleration(self): self.check(9)


if __name__ == "__main__":
    unittest.main()
