#!/usr/bin/env python3
"""Compile the actual telemetry module: no HAL, CAN transport or hardware."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "ik_telemetry.h"
#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(sizeof(struct ik_telemetry_wheel) == 13U * 4U, "row size");
_Static_assert(sizeof(ik_drive_telem) == 4U * 52U, "four rows");
_Static_assert(_Generic(&ik_drive_telem_now, volatile uint32_t *: 1,
                       default: 0), "volatile 32-bit clock");
#define FIELD(field, word, type) \
  _Static_assert(offsetof(struct ik_telemetry_wheel, field) == (word) * 4U, \
                 "offset: " #field); \
  _Static_assert(sizeof(ik_drive_telem[0].field) == 4, "width: " #field); \
  _Static_assert(_Generic(&ik_drive_telem[0].field, volatile type *: 1, \
                         default: 0), "type: " #field)
FIELD(generation, 0, uint32_t);
FIELD(rpm, 1, int32_t);
FIELD(state, 2, uint32_t);
FIELD(hall, 3, uint32_t);
FIELD(hall_changes, 4, uint32_t);
FIELD(speed_tick, 5, uint32_t);
FIELD(iq_counts, 6, int32_t);
FIELD(id_counts, 7, int32_t);
FIELD(flags, 8, uint32_t);
FIELD(peak_l1_counts, 9, uint32_t);
FIELD(current_tick, 10, uint32_t);
FIELD(speed_count, 11, uint32_t);
FIELD(current_count, 12, uint32_t);
#undef FIELD

static void snapshot(struct ik_telemetry_wheel rows[4]) {
  unsigned char *out = (unsigned char *)rows;
  const volatile unsigned char *in =
      (const volatile unsigned char *)ik_drive_telem;
  for (size_t i = 0; i < sizeof(ik_drive_telem); ++i) out[i] = in[i];
}
static void unchanged(const struct ik_telemetry_wheel before[4]) {
  struct ik_telemetry_wheel after[4];
  snapshot(after);
  assert(memcmp(before, after, sizeof(after)) == 0);
}
static void poll(uint32_t now, int allowed, uint32_t expected) {
  assert(ik_telemetry_poll_due(now, allowed) == expected);
  assert(ik_drive_telem_now == now);
}
static void put16(uint8_t *data, int32_t value) {
  uint32_t bits = (uint16_t)value;
  data[0] = (uint8_t)bits; data[1] = (uint8_t)(bits >> 8);
}
static void put32(uint8_t *data, uint32_t bits) {
  for (unsigned i = 0; i < 4; ++i) data[i] = (uint8_t)(bits >> (8U * i));
}
static void speed(unsigned index, int32_t rpm, uint8_t state, uint8_t hall,
                  uint32_t changes, uint32_t tick) {
  uint8_t data[8];
  put16(data, rpm); data[2] = state; data[3] = hall;
  put32(data + 4, changes);
  assert(ik_telemetry_rx(0x280U | (1U << index), data, tick) == 1);
}
static void current(unsigned index, int32_t iq, int32_t id, uint32_t flags,
                    uint32_t peak, uint32_t tick) {
  uint8_t data[8];
  put16(data, iq); put16(data + 2, id);
  put16(data + 4, (int32_t)flags); put16(data + 6, (int32_t)peak);
  assert(ik_telemetry_rx(0x380U | (1U << index), data, tick) == 1);
}
static void check_initial(void) {
  const struct ik_telemetry_wheel zero[4] = {{0}};
  assert(ik_drive_telem_now == 0);
  unchanged(zero); /* Zero counts, not zero-valued samples, mean unseen. */
}
static void check_decode(int current_first) {
  const int32_t values[] = {-32768, -12345, -1, 0, 1, 32767};
  for (unsigned wheel = 0; wheel < 4; ++wheel) {
    for (unsigned v = 0; v < 6; ++v) {
      struct ik_telemetry_wheel before[4], after[4];
      const uint32_t tick = 1000U + wheel * 100U + v;
      const uint32_t flags = v & 1U ? 0xFFFFU : 0xA581U;
      const uint32_t peak = v & 1U ? 65535U : 0x81ABU;
      const uint32_t changes = v & 1U ? UINT32_MAX : 0x89ABCDEFU;
      const uint8_t state = (uint8_t)(0xF0U + v), hall = (uint8_t)(0x81U + v);
      snapshot(before);
      if (current_first) {
        current(wheel, values[v], values[5U-v], flags, peak, tick);
      } else {
        speed(wheel, values[v], state, hall, changes, tick);
      }
      snapshot(after);
      struct ik_telemetry_wheel expected = before[wheel];
      expected.generation += 2U;
      if (current_first) {
        expected.iq_counts = values[v]; expected.id_counts = values[5U-v];
        expected.flags = flags; expected.peak_l1_counts = peak;
        expected.current_tick = tick; ++expected.current_count;
      } else {
        expected.rpm = values[v]; expected.state = state; expected.hall = hall;
        expected.hall_changes = changes; expected.speed_tick = tick;
        ++expected.speed_count;
      }
      assert(memcmp(&expected, &after[wheel], sizeof(expected)) == 0);
      assert((after[wheel].generation & 1U) == 0);
      for (unsigned other = 0; other < 4; ++other) {
        if (other != wheel)
          assert(memcmp(&before[other], &after[other], sizeof(expected)) == 0);
      }
    }
  }
}
int main(int argc, char **argv) {
  assert(argc == 2);
  check_initial();
  const unsigned scenario = (unsigned)atoi(argv[1]);
  if (scenario == 0) {
    /* First request is measured from boot, not first invocation. */
    for (uint32_t now = 0; now < 20U; ++now) poll(now, 1, 0);
    const uint32_t ids[] = {0x101, 0x301, 0x102, 0x302,
                            0x104, 0x304, 0x108, 0x308};
    for (unsigned i = 0; i < 24; ++i) {
      uint32_t now = (i + 1U) * 20U;
      poll(now, 1, ids[i % 8U]);
      poll(now, 1, 0);
      poll(now + 19U, 1, 0);
    }
  } else if (scenario == 1) {
    /* A long disallowed period must neither consume nor advance a poll. */
    poll(19, 0, 0); poll(20, 0, 0); poll(400, 0, 0);
    poll(400, 2, 0x101); poll(419, 1, 0);
    poll(420, 0, 0); poll(1000, 0, 0);
    poll(1000, -1, 0x301); poll(1000, 1, 0);
    poll(1019, 1, 0); poll(1020, 1, 0x102);
  } else if (scenario == 2) {
    /* Delayed calls issue one request, then wait a full new interval. */
    poll(5000, 1, 0x101);
    for (uint32_t now = 5000; now < 5020; ++now) poll(now, 1, 0);
    poll(5020, 1, 0x301); poll(99999, 1, 0x102);
    poll(99999, 1, 0); poll(100018, 1, 0); poll(100019, 1, 0x302);
  } else if (scenario == 3) {
    poll(UINT32_MAX - 9U, 1, 0x101);
    poll(UINT32_MAX, 1, 0); poll(0, 1, 0); poll(9, 1, 0);
    poll(10, 1, 0x301); poll(29, 1, 0); poll(30, 1, 0x102);
    poll(UINT32_MAX - 4U, 0, 0); poll(0, 0, 0);
  } else if (scenario == 4) {
    check_decode(0); check_decode(1);
  } else if (scenario == 5) {
    check_decode(1); check_decode(0);
  } else if (scenario == 6) {
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      speed(wheel, -1, 255, 254, UINT32_MAX, 123);
      current(wheel, -32768, 32767, 65535, 65535, 456);
    }
    struct ik_telemetry_wheel before[4];
    const uint8_t data[8] = {0};
    snapshot(before);
    /* Exhaust all standard IDs: only these eight full IDs may match. */
    for (uint32_t id = 0; id <= 0x7FFU; ++id) {
      if (id == 0x281 || id == 0x282 || id == 0x284 || id == 0x288 ||
          id == 0x381 || id == 0x382 || id == 0x384 || id == 0x388) continue;
      assert(ik_telemetry_rx(id, data, 999) == 0);
      unchanged(before);
    }
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      for (uint32_t base = 0x280; base <= 0x380; base += 0x100) {
        uint32_t id = base | (1U << wheel);
        assert(ik_telemetry_rx(id, NULL, 999) == 0);
        for (unsigned bit = 11; bit < 32; ++bit)
          assert(ik_telemetry_rx(id | (1U << bit), data, 999) == 0);
      }
    }
    assert(ik_telemetry_rx(UINT32_MAX, data, 999) == 0);
    unchanged(before);
  } else if (scenario == 7) {
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      speed(wheel, 0, 0, 0, 0, UINT32_MAX - 9U);
      assert(ik_drive_telem[wheel].speed_count == 1);
      assert(ik_drive_telem[wheel].current_count == 0);
      current(wheel, 0, 0, 0, 0, 5);
      assert(ik_drive_telem[wheel].current_count == 1);
      assert(ik_drive_telem[wheel].speed_tick == UINT32_MAX - 9U);
    }
    poll(10, 0, 0);
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      assert((uint32_t)(ik_drive_telem_now - ik_drive_telem[wheel].speed_tick) == 20);
      assert((uint32_t)(ik_drive_telem_now - ik_drive_telem[wheel].current_tick) == 5);
      speed(wheel, 0, 0, 0, 0, 11);
      assert(ik_drive_telem[wheel].current_tick == 5);
      assert(ik_drive_telem[wheel].current_count == 1);
      assert(ik_drive_telem[wheel].speed_count == 2);
      assert(ik_drive_telem[wheel].generation == 6);
    }
  } else if (scenario == 8) {
    for (unsigned wheel = 0; wheel < 4; ++wheel) {
      /* Natural unsigned generation/count rollover must not affect data. */
      ik_drive_telem[wheel].generation = UINT32_MAX - 1U;
      ik_drive_telem[wheel].speed_count = UINT32_MAX;
      ik_drive_telem[wheel].current_count = UINT32_MAX;
      speed(wheel, -32768, 255, 255, UINT32_MAX, UINT32_MAX);
      assert(ik_drive_telem[wheel].generation == 0);
      assert(ik_drive_telem[wheel].speed_count == 0);
      assert(ik_drive_telem[wheel].current_count == UINT32_MAX);
      current(wheel, 32767, -32768, 65535, 65535, 0);
      assert(ik_drive_telem[wheel].generation == 2);
      assert(ik_drive_telem[wheel].current_count == 0);
      assert(ik_drive_telem[wheel].speed_tick == UINT32_MAX);
      assert(ik_drive_telem[wheel].rpm == -32768);
      assert(ik_drive_telem[wheel].iq_counts == 32767);
      assert(ik_drive_telem[wheel].id_counts == -32768);
      assert(ik_drive_telem[wheel].peak_l1_counts == 65535);
    }
  } else {
    assert(0 && "unknown scenario");
  }
  return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='ik-telemetry-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(HARNESS)
    for optimization in ('-O0', '-O2'):
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-pedantic', optimization,
                        '-I', str(ROOT / 'CM7/Core/Inc'), str(tmp / 'test.c'),
                        str(ROOT / 'CM7/Core/Src/ik_telemetry.c'),
                        '-o', str(tmp / 'test')], check=True)
        for scenario in range(9):
            subprocess.run([str(tmp / 'test'), str(scenario)], check=True)
print('PASS: 9 IK telemetry scenarios at -O0 and -O2 (actual C, no HAL/hardware)')
