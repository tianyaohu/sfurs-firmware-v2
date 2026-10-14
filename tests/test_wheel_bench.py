#!/usr/bin/env python3
"""Compile and exercise the actual bench state machine with a fake CAN bus."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
harness = r'''
#include "wheel_bench.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>
static unsigned sends, id, fail_send, aborts;
static int status = BENCH_TX_OK;
static uint8_t payload[4];
int wheel_bench_send(uint32_t identifier, const uint8_t data[4]) {
  ++sends; id = identifier; memcpy(payload, data, 4); return !fail_send;
}
int wheel_bench_poll(void) { return status; }
void wheel_bench_abort(void) { ++aborts; }
static void arm(unsigned now) {
  wheel_bench_request = BENCH_ARM; wheel_bench_tick(now);
}
static void run(unsigned now) {
  wheel_bench_request = BENCH_RUN; wheel_bench_tick(now);
}
static void complete_stop(unsigned now) {
  wheel_bench_tick(now+1); wheel_bench_tick(now+51);
  wheel_bench_tick(now+52); wheel_bench_tick(now+102);
  wheel_bench_tick(now+103);
}
int main(int argc, char **argv) {
  assert(argc == 2);
  int test = atoi(argv[1]);
  wheel_bench_wheel = 4; wheel_bench_direction = 1;
  if (test == 0) {
    for (unsigned t=0;t<10000;t++) wheel_bench_tick(t);
    assert(sends == 0 && wheel_bench_state == BENCH_IDLE);
    assert(!wheel_bench_tx_count && !wheel_bench_first_fault && !wheel_bench_request_count);
    run(10000); assert(!sends && wheel_bench_result == 4);
    wheel_bench_request = 0x52554E31; wheel_bench_tick(10001);
    assert(!sends && wheel_bench_result == 4); /* old RUN1 rejected */
  } else if (test == 1) {
    for (unsigned w=0;w<16;w++) {
      wheel_bench_wheel=w; wheel_bench_direction=0; arm(w);
      assert(wheel_bench_state == BENCH_IDLE);
      if (w!=1 && w!=2 && w!=4 && w!=8) {
        wheel_bench_direction=1; arm(w);
        assert(wheel_bench_state == BENCH_IDLE);
      }
    }
    assert(!sends);
  } else if (test >= 2 && test <= 9) {
    unsigned index=(unsigned)(test-2);
    wheel_bench_wheel=1U << (index/2);
    wheel_bench_direction=(index&1) ? -1 : 1;
    if (index==1) {
      /* NE reverse also transmits CAN ARM (RPM2) on arming. */
      arm(0); assert(sends==1 && id==0x401 && wheel_bench_state == BENCH_ARMED);
      assert(payload[0]==0x32 && payload[1]==0x4d && payload[2]==0x50 && payload[3]==0x52);
      run(1); assert(sends==2 && id==(0x80|wheel_bench_wheel));
    } else {
      arm(0); assert(!sends && wheel_bench_state == BENCH_ARMED);
      run(1); assert(sends==1 && id==(0x80|wheel_bench_wheel));
    }
    assert(payload[0]==((index&1) ? ((index==1) ? 0x9c : 0xe2) : 30));
    assert(payload[1]==((index&1) ? 0xff : 0));
    assert(payload[2]==0xf4 && payload[3]==1);
    assert(wheel_bench_run_count == 1);
    run(2); assert(sends==((index==1) ? 2 : 1)); /* no RUN while RUNNING */
    if (index==1) {
      wheel_bench_tick(14000); assert(sends==2);
      wheel_bench_tick(15001); assert(sends==3 && id==0x78f);
    } else {
      wheel_bench_tick(1000); assert(sends==1);
      wheel_bench_tick(1001); assert(sends==2 && id==0x78f);
    }
    assert(!payload[0] && !payload[1] && !payload[2] && !payload[3]);
    complete_stop((index==1) ? 15001 : 1001);
    if (index==1) {
      assert(sends==5 && wheel_bench_state==BENCH_IDLE && wheel_bench_result==3);
      assert(wheel_bench_tx_count==5 && wheel_bench_tx_ack_count==5);
    } else {
      assert(sends==4 && wheel_bench_state==BENCH_IDLE && wheel_bench_result==3);
      assert(wheel_bench_tx_count==4 && wheel_bench_tx_ack_count==4);
    }
    arm(4000); assert(wheel_bench_state==BENCH_ARMED && wheel_bench_result==1); /* repeat, no reset */
    run(4001); assert(sends==((index==1) ? 7 : 5) && wheel_bench_run_count==2);
    if (index==1) {
      wheel_bench_tick(19001); assert(sends==8 && id==0x78f);
      complete_stop(19001); assert(sends==10 && wheel_bench_state==BENCH_IDLE);
      arm(19603); assert(wheel_bench_result==4); /* cooldown */
      arm(21104); assert(wheel_bench_state==BENCH_ARMED);
    } else {
      wheel_bench_tick(5001); assert(sends==6 && id==0x78f);
      complete_stop(5001); assert(sends==8 && wheel_bench_state==BENCH_IDLE);
      arm(5603); assert(wheel_bench_result==4); /* cooldown */
      arm(7104); assert(wheel_bench_state==BENCH_ARMED);
    }
  } else if (test == 10) {
    arm(0); wheel_bench_tick(5000); run(5001);
    assert(!sends && wheel_bench_state==BENCH_IDLE);
    arm(6000); wheel_bench_wheel=8; run(6001);
    assert(!sends && wheel_bench_state==BENCH_IDLE);
  } else if (test == 11) {
    arm(0); status=BENCH_TX_PENDING; run(1);
    wheel_bench_request=BENCH_STOP; wheel_bench_tick(2);
    assert(aborts==1 && sends==2 && id==0x78f);
    status=BENCH_TX_OK; complete_stop(2);
    assert(wheel_bench_state==BENCH_IDLE);
  } else if (test == 12) {
    arm(0); fail_send=1; run(1);
    wheel_bench_tick(51); wheel_bench_tick(101);
    assert(sends==4 && wheel_bench_state==BENCH_FAULT);
    assert(wheel_bench_first_fault==1 && wheel_bench_first_fault_tick==1);
    fail_send=0; arm(10000); assert(wheel_bench_state==BENCH_FAULT);
  } else if (test == 13) {
    arm(0); status=BENCH_TX_PENDING; run(1);
    wheel_bench_tick(101); assert(aborts==1 && id==0x78f);
    status=BENCH_TX_OK; complete_stop(101);
    assert(wheel_bench_state==BENCH_FAULT && wheel_bench_result==5);
    assert(wheel_bench_first_fault==3 && wheel_bench_first_fault_tick==101);
  } else if (test == 14) {
    arm(0xfffffff0U); run(0xfffffff1U);
    wheel_bench_tick(984); assert(id==(0x80|4));
    wheel_bench_tick(985); assert(id==0x78f);
  } else if (test == 15) {
    wheel_bench_request=BENCH_STOP; wheel_bench_tick(0);
    complete_stop(0); assert(sends==3 && wheel_bench_state==BENCH_IDLE);
  } else if (test == 16) {
    /* TX pending cleared before TXBTO is sampled: absence of both terminal
     * flags must not be classified as failure. Unrelated slot flags ignored. */
    assert(wheel_bench_tx_status(0, 0, 1)==BENCH_TX_PENDING);
    assert(wheel_bench_tx_status(2, 2, 1)==BENCH_TX_PENDING);
    assert(wheel_bench_tx_status(1, 0, 1)==BENCH_TX_OK);
    assert(wheel_bench_tx_status(0, 1, 1)==BENCH_TX_FAILED);
    assert(wheel_bench_tx_status(1, 1, 1)==BENCH_TX_OK);
    status=wheel_bench_tx_status(0, 0, 1);
    wheel_bench_request=BENCH_STOP; wheel_bench_tick(0);
    wheel_bench_tick(1); assert(!wheel_bench_first_fault);
    status=wheel_bench_tx_status(1, 0, 1);
    complete_stop(1);
    assert(wheel_bench_state==BENCH_IDLE && wheel_bench_tx_ack_count==3);
  } else if (test == 17) {
    wheel_bench_button(100,1); wheel_bench_button(200,1); wheel_bench_tick(200);
    assert(!sends && !wheel_bench_request_count);
    arm(300); wheel_bench_button(400,0); wheel_bench_button(450,0);
    wheel_bench_button(500,1); wheel_bench_button(550,1); wheel_bench_tick(550);
    assert(!sends && wheel_bench_state==BENCH_ARMED); /* normal ARM no button */
  } else if (test == 18) {
    wheel_bench_button(0,1); wheel_bench_button(50,1);
    wheel_bench_request=BENCH_ARM_BUTTON; wheel_bench_tick(100);
    wheel_bench_button(200,1); wheel_bench_tick(200); assert(!sends);
    wheel_bench_button(300,0); wheel_bench_button(350,0);
    wheel_bench_button(400,1); wheel_bench_button(449,1); wheel_bench_tick(449);
    assert(!sends);
    wheel_bench_button(450,1); wheel_bench_tick(450);
    assert(sends==1 && id==0x84 && wheel_bench_state==BENCH_RUNNING);
    wheel_bench_button(500,1); wheel_bench_tick(500); assert(sends==1);
    wheel_bench_tick(1450); complete_stop(1450);
    wheel_bench_button(2000,0); wheel_bench_button(2050,0);
    wheel_bench_button(2100,1); wheel_bench_button(2150,1); wheel_bench_tick(2150);
    assert(sends==4 && wheel_bench_state==BENCH_IDLE); /* no repeat */
  } else if (test == 19) {
    wheel_bench_request=BENCH_ARM_BUTTON; wheel_bench_tick(0);
    wheel_bench_tick(59999); assert(wheel_bench_state==BENCH_ARMED);
    wheel_bench_tick(60000); assert(wheel_bench_state==BENCH_IDLE);
    wheel_bench_button(60001,0); wheel_bench_button(60100,1);
    wheel_bench_button(60150,1); wheel_bench_tick(60150); assert(!sends);
  } else if (test == 20) {
    wheel_bench_request=BENCH_ARM_BUTTON; wheel_bench_tick(0);
    wheel_bench_button(100,0);
    wheel_bench_button(110,1); wheel_bench_button(120,0); /* bounce */
    wheel_bench_button(130,1); wheel_bench_button(179,1);
    wheel_bench_tick(179); assert(!sends);
    wheel_bench_wheel=8; wheel_bench_button(180,1); wheel_bench_tick(180);
    assert(!sends && wheel_bench_state==BENCH_IDLE && wheel_bench_result==4);
  } else if (test == 21) {
    /* Startup-held button must not arm, regardless of duration. */
    wheel_bench_button(0,1); wheel_bench_button(5000,1); wheel_bench_tick(5000);
    assert(!sends && wheel_bench_state==BENCH_IDLE && !wheel_bench_request_count);
    wheel_bench_button(5100,0); wheel_bench_button(5150,0);
    wheel_bench_button(5200,1); wheel_bench_button(7199,1); wheel_bench_tick(7199);
    assert(wheel_bench_state==BENCH_IDLE);
    wheel_bench_button(7200,1); wheel_bench_tick(7200);
    assert(!sends && wheel_bench_state==BENCH_ARMED);
    assert(wheel_bench_wheel==1 && wheel_bench_direction==1);
    wheel_bench_button(8000,1); wheel_bench_tick(8000); assert(!sends);
    wheel_bench_button(8100,0); wheel_bench_button(8150,0);
    wheel_bench_button(8200,1); wheel_bench_button(8250,1); wheel_bench_tick(8250);
    assert(sends==1 && id==0x81 && payload[0]==30 && wheel_bench_run_count==1);
    wheel_bench_tick(9250); complete_stop(9250);
    wheel_bench_button(10000,0); wheel_bench_button(10050,0);
    wheel_bench_button(10100,1); wheel_bench_button(12100,1); wheel_bench_tick(12100);
    assert(sends==4 && wheel_bench_state==BENCH_ARMED); /* repeat needs full gesture */
    wheel_bench_button(12200,0); wheel_bench_button(12250,0);
    wheel_bench_button(12300,1); wheel_bench_button(12350,1); wheel_bench_tick(12350);
    assert(sends==5 && id==0x81 && wheel_bench_run_count==2);
    wheel_bench_tick(13350); complete_stop(13351);
    assert(sends==8 && wheel_bench_state==BENCH_IDLE);
  } else if (test == 22) {
    /* Three short presses are neither arming nor RUN. */
    wheel_bench_button(0,0); wheel_bench_button(50,0);
    for (unsigned t=100;t<1000;t+=300) {
      wheel_bench_button(t,1); wheel_bench_button(t+100,1); wheel_bench_tick(t+100);
      wheel_bench_button(t+150,0); wheel_bench_button(t+200,0); wheel_bench_tick(t+200);
    }
    assert(!sends && !wheel_bench_request_count && wheel_bench_state==BENCH_IDLE);
  } else if (test == 23) {
    /* Physical arming cannot bypass a transport fault. */
    fail_send=1; wheel_bench_request=BENCH_STOP; wheel_bench_tick(0);
    wheel_bench_tick(50); wheel_bench_tick(100);
    assert(wheel_bench_state==BENCH_FAULT);
    wheel_bench_button(200,0); wheel_bench_button(250,0);
    wheel_bench_button(300,1); wheel_bench_button(2300,1); wheel_bench_tick(2300);
    assert(wheel_bench_state==BENCH_FAULT && sends==3);
  } else if (test == 24) {
    /* SE forward follows the configured speed. */
    wheel_bench_speed_rpm = 100;
    wheel_bench_wheel = 2; wheel_bench_direction = 1;
    arm(0); run(1);
    assert(sends==1 && id==0x82);
    assert(payload[0]==100 && payload[1]==0 && payload[2]==0xf4 && payload[3]==1);
  } else if (test == 25) {
    /* SW reverse follows the configured speed (signed payload). */
    wheel_bench_speed_rpm = 100;
    wheel_bench_wheel = 4; wheel_bench_direction = -1;
    arm(0); run(1);
    assert(sends==1 && id==0x84);
    assert(payload[0]==0x9c && payload[1]==0xff && payload[2]==0xf4 && payload[3]==1);
  } else if (test == 26) {
    /* Below-range clamps to 10 RPM. */
    wheel_bench_speed_rpm = 5;
    wheel_bench_wheel = 2; wheel_bench_direction = 1;
    arm(0); run(1);
    assert(sends==1 && id==0x82 && payload[0]==10 && payload[1]==0);
  } else if (test == 27) {
    /* Above-range clamps to 120 RPM; NE reverse stays locked at -100. */
    wheel_bench_speed_rpm = 500;
    wheel_bench_wheel = 2; wheel_bench_direction = 1;
    arm(0); run(1);
    assert(sends==1 && id==0x82 && payload[0]==120 && payload[1]==0);
    wheel_bench_request = BENCH_STOP; wheel_bench_tick(2);
    complete_stop(2); wheel_bench_tick(154);
    assert(wheel_bench_state==BENCH_IDLE);
    wheel_bench_wheel = 1; wheel_bench_direction = -1;
    arm(2154); run(2155);
    assert(sends==6 && id==0x81 && wheel_bench_run_count==2);
    assert(payload[0]==0x9c && payload[1]==0xff);
  }
  return 0;
}
'''
# Check integration: the old guarded sender cannot bypass the bench gates.
main = (root / 'CM7/Core/Src/main.c').read_text()
assert '#define WHEEL_BENCH_MODE 1U' in main
assert 'if (WHEEL_BENCH_MODE == 0U && can_tx_test_status != 1U &&' in main
assert 'wheel_bench_tick(now);' in main
assert 'HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)' in main
with tempfile.TemporaryDirectory(prefix='wheel-bench-tests-', dir='/tmp/opencode') as tmp:
    tmp = pathlib.Path(tmp)
    (tmp / 'test.c').write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(root / 'CM7/Core/Inc'), str(tmp / 'test.c'),
                    str(root / 'CM7/Core/Src/wheel_bench.c'), '-o', str(tmp / 'test')], check=True)
    for case in range(28):
        subprocess.run([str(tmp / 'test'), str(case)], check=True)
print('PASS: 28 scenarios, signed payloads, STOP/fault gates, TX race, physical ARM/RUN gates, speed select')
