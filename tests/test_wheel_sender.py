#!/usr/bin/env python3
"""Host-test the actual Nucleo command-selection block, without hardware."""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[1]
source = (root / 'CM7/Core/Src/main.c').read_text()
selection = source.split('      uint32_t requested = wheel_test_request;', 1)[1]
selection = 'uint32_t requested = wheel_test_request;' + selection.split('      tx_header.IdType', 1)[0]
harness = r'''
#include <assert.h>
#include <stdint.h>
uint32_t wheel_test_request, wheel_test_last_request, wheel_test_result;
uint32_t wheel_test_run_attempted, esc_current_reply_count, esc_ne_current_tick;
uint32_t esc_ne_test_flags, esc_telemetry_reply_count, esc_ne_motor_state;
uint32_t esc_ne_last_reply_tick, now;
uint8_t tx_is_wheel_command, tx_data[4];
struct { uint32_t Identifier; } tx_header;
void select_command(void) {
'''
harness += selection + '\n}\n'
harness += r'''
void reset(void) {
  wheel_test_request = wheel_test_last_request = wheel_test_result = 0;
  wheel_test_run_attempted = 0; now = 1000;
  esc_current_reply_count = esc_telemetry_reply_count = 1;
  esc_ne_current_tick = esc_ne_last_reply_tick = now;
  esc_ne_test_flags = 0x8800; esc_ne_motor_state = 0;
  tx_header.Identifier = 0x101; tx_is_wheel_command = 0;
  for (int i = 0; i < 4; ++i) tx_data[i] = 0;
}
void run(void) { wheel_test_request = 0x52554E31; select_command(); }
int main(void) {
  reset(); select_command(); assert(!tx_is_wheel_command);
  reset(); wheel_test_request = 1; select_command(); assert(wheel_test_result == 2);
  reset(); run(); assert(!tx_is_wheel_command && wheel_test_result == 2);
  reset(); wheel_test_request = 0x41524D31; select_command();
  assert(tx_is_wheel_command && tx_header.Identifier == 0x401);
  assert(tx_data[0] == 0x54 && tx_data[1] == 0x53 && tx_data[2] == 0x45 && tx_data[3] == 0x54);
  reset(); esc_current_reply_count = 0; esc_ne_test_flags = 0xc800;
  run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_current_tick = 699; esc_ne_test_flags = 0xc800;
  run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc804; run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc000; run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0x4000; run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xe800; run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xd800; run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc800; esc_telemetry_reply_count = 0;
  run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc800; esc_ne_motor_state = 10;
  run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc800; esc_ne_last_reply_tick = 699;
  run(); assert(!tx_is_wheel_command);
  reset(); esc_ne_test_flags = 0xc800; run();
  assert(tx_is_wheel_command && tx_header.Identifier == 0x81 && wheel_test_run_attempted);
  assert(tx_data[0] == 60 && tx_data[1] == 0 && tx_data[2] == 0xe8 && tx_data[3] == 3);
  tx_is_wheel_command = 0; run(); assert(!tx_is_wheel_command);
  reset(); esc_current_reply_count = 0; esc_ne_test_flags = 0;
  wheel_test_request = 0x53544F50; select_command();
  assert(tx_is_wheel_command && tx_header.Identifier == 0x781);
  reset(); now = 50; esc_ne_current_tick = UINT32_MAX - 49;
  esc_ne_last_reply_tick = esc_ne_current_tick; esc_ne_test_flags = 0xc800;
  run(); assert(tx_is_wheel_command);
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='nucleo-sender-tests-', dir='/tmp/opencode') as folder:
    folder = pathlib.Path(folder)
    c_file = folder / 'test.c'
    c_file.write_text(harness)
    binary = folder / 'test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    str(c_file), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
print('PASS: no automatic drive, freshness/profile/fault gates, one-shot payload and stop')
