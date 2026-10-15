#!/usr/bin/env python3
"""Compile the actual IK mailbox adapter with a fake CAN transport."""
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "ik_drive.h"
#include "wheel_bench.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static unsigned count, aborts, fail_send;
static unsigned ids[128];
static unsigned char payload[128][4];
static int status = BENCH_TX_OK;
int wheel_bench_send(uint32_t id, const uint8_t data[4]) {
  assert(count < 128);
  ids[count] = id; memcpy(payload[count++], data, 4);
  return !fail_send;
}
int wheel_bench_poll(void) { return status; }
void wheel_bench_abort(void) { ++aborts; }
static void request(unsigned command, unsigned seq, unsigned now) {
  ik_drive_request = command; ik_drive_sequence = seq; ik_drive_tick(now);
}
static void pump(unsigned start, unsigned duration) {
  for (unsigned i=0; i<duration; ++i) ik_drive_tick(start+i);
}
static void prepare(void) {
  request(IK_PREPARE, 1, 1); pump(2, 10);
  assert(ik_drive_completed == 1 && ik_drive_state == IK_IDLE);
  assert(count == 4 && ik_drive_result == IK_SENT);
  for (unsigned i=0; i<4; ++i) {
    assert(ids[i] == (0x400U | (1U << i)));
    assert(memcmp(payload[i], "\x32\x4d\x50\x52", 4) == 0);
  }
}
static int rpm(unsigned i) {
  unsigned bits=payload[i][0] | ((unsigned)payload[i][1]<<8);
  return bits & 0x8000 ? (int)bits-65536 : (int)bits;
}
int main(int argc, char **argv) {
  assert(argc==2); int scenario=atoi(argv[1]);
  if (scenario==0) {
    pump(0, 1000); assert(count==0 && ik_drive_state==IK_IDLE);
  } else if (scenario==1) {
    prepare(); assert(ik_drive_ack_count==4 && ik_drive_tx_count==4);
  } else if (scenario>=2 && scenario<=4) {
    if (scenario==2) ik_drive_vy_mmps=200;
    if (scenario==3) ik_drive_vx_mmps=200;
    if (scenario==4) ik_drive_omega_mradps=500;
    request(IK_START, 2, 1); pump(2, 15);
    assert(count==8 && ik_drive_state==IK_RUNNING && ik_drive_completed==2);
    const int expected[3][4]={{51,48,-48,-51},{-29,33,33,-29},{13,12,12,13}};
    for (unsigned i=0; i<4; ++i) {
      assert(ids[i]==(0x400U | (1U<<i)));
      assert(ids[4+i]==(0x080U | (1U<<i)));
      assert(rpm(4+i)==expected[scenario-2][i]);
      assert(ik_drive_rpm[i]==expected[scenario-2][i]);
      assert(payload[4+i][2]==0xf4 && payload[4+i][3]==1);
    }
    request(IK_START, 3, 20); /* No unsupported running updates. */
    assert(count==8 && ik_drive_completed==3 && ik_drive_result==IK_REJECTED);
    request(IK_STOP, 4, 21); pump(22, 200);
    assert(ik_drive_state==IK_STOPPING && ik_drive_completed!=4);
    pump(222, 800);
    assert(count==11 && ik_drive_completed==4 && ik_drive_state==IK_IDLE);
    for (unsigned i=8; i<11; ++i) {
      assert(ids[i]==0x78f && memcmp(payload[i], "\0\0\0\0", 4)==0);
    }
  } else if (scenario==5) {
    request(IK_START, 1, 0); pump(1, 1000);
    assert(count==3 && ik_drive_state==IK_IDLE); /* zero is STOP, not start */
    assert(ik_drive_completed==1);
  } else if (scenario==6) {
    ik_drive_vx_mmps=301;
    request(IK_START, 1, 0);
    assert(count==0 && ik_drive_completed==1 && ik_drive_result==IK_BAD_TARGET);
  } else if (scenario==7) {
    status=BENCH_TX_PENDING;
    request(IK_PREPARE, 1, 0);
    ik_drive_tick(99); assert(ik_drive_fault==0 && count==1);
    ik_drive_tick(100); assert(ik_drive_fault==3 && aborts==1);
    status=BENCH_TX_OK; pump(101, 200);
    assert(ik_drive_state==IK_FAULT && count==4 && ik_drive_completed==1);
    request(IK_START, 2, 400);
    assert(count==4 && ik_drive_result==IK_REJECTED);
  } else if (scenario==8) {
    fail_send=1; request(IK_PREPARE, 1, 0); pump(1, 200);
    assert(count==4 && ik_drive_fault==1 && ik_drive_state==IK_FAULT);
    assert(ik_drive_completed==1);
  } else if (scenario==9) {
    status=BENCH_TX_PENDING; request(IK_PREPARE, 1, 0);
    request(IK_STOP, 2, 1); assert(aborts==1 && ids[1]==0x78f);
    status=BENCH_TX_OK; pump(2, 1000);
    assert(count==4 && ik_drive_completed==2 && ik_drive_state==IK_IDLE);
  } else if (scenario==10) {
    status=BENCH_TX_PENDING; request(IK_PREPARE, 1, 0xfffffff0U);
    ik_drive_tick(83); assert(ik_drive_fault==0);
    ik_drive_tick(84); assert(ik_drive_fault==3);
    status=BENCH_TX_OK; pump(85, 200); assert(ik_drive_state==IK_FAULT);
  } else if (scenario==11) {
    ik_drive_vy_mmps=200; request(IK_START, 1, 0);
    pump(1, 5); status=BENCH_TX_FAILED; ik_drive_tick(6);
    status=BENCH_TX_OK; pump(7, 200);
    assert(ik_drive_fault==2 && ik_drive_state==IK_FAULT);
    for (unsigned i=6; i<count; ++i) assert(ids[i]==0x78f);
  } else if (scenario==12) {
    request(999, 1, 0); assert(count==0 && ik_drive_result==IK_REJECTED);
    request(IK_PREPARE, 2, 1); pump(2,10);
    assert(count==4 && ik_drive_state==IK_IDLE); /* Reject leaves IDLE usable. */
    pump(12, 100); assert(count==4); /* Persistent request is not replayed. */
    request(IK_PREPARE, 2, 120); assert(count==4); /* Duplicate sequence ignored. */
  }
  return 0;
}
'''

with tempfile.TemporaryDirectory(prefix='ik-drive-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(HARNESS)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(ROOT / 'CM7/Core/Inc'), '-I', str(ROOT / 'Common/Inc'),
                    str(tmp / 'test.c'), str(ROOT / 'CM7/Core/Src/ik_drive.c'),
                    str(ROOT / 'Common/Src/rover_kinematics.c'),
                    '-lm', '-o', str(tmp / 'test')], check=True)
    for scenario in range(13):
        subprocess.run([str(tmp / 'test'), str(scenario)], check=True)
print('PASS: 13 IK adapter scenarios (fake CAN, no hardware)')
