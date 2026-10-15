#!/usr/bin/env python3
"""Read-only HOTPLUG monitoring of the verified original front-right ESC.

Never writes memory, resets, clears faults, or sends a motor command.
"""
import argparse
import json
import pathlib
import re
import subprocess
import time
from esc_original_decode import decode_foc, parse_words, ram_pointer, scale_float

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=90)
parser.add_argument('--output', type=pathlib.Path, required=True)
parser.add_argument('--foc', action='store_true',
                    help='Also sample verified original FOC/current/speed and raw PB6..8 inputs')
args = parser.parse_args()
if args.output.exists():
    parser.error('Refusing to overwrite existing log')
cli = '/home/fh/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI'
base = [cli, '-c', 'port=SWD', 'mode=HOTPLUG', 'sn=066AFF525266555067075126']
command = base + ['-r32', '0x20000000', '0x1c']
if args.foc:
    # Resolve live original-image pointers before reading any derived address.
    result = subprocess.run(base + ['-r32', '0x20001c00', '0x4',
                                    '-r32', '0x20000000', '0x30'],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError('Cannot resolve original MCI handle')
    mem = parse_words(result.stdout)
    if mem[0x20001c00] != 0x20000000:
        raise ValueError('Unrecognized original-image MCI handle')
    stc = ram_pointer(mem[0x2000001c], 24)
    foc = ram_pointer(mem[0x20000020], 32)
    scale = ram_pointer(mem[0x20000010], 8)
    result = subprocess.run(base + ['-r32', hex(stc + 20), '0x4',
                                    '-r32', hex(scale), '0x8'],
                            capture_output=True, text=True, timeout=5)
    if result.returncode:
        raise RuntimeError('Cannot resolve original speed/current scale')
    mem = parse_words(result.stdout)
    speed = ram_pointer(mem[stc + 20], 16)
    current_scale = scale_float(mem[scale + 4])
    print(json.dumps({'foc_pointer': hex(foc), 'speed_pointer': hex(speed),
                      'firmware_current_scale_a_per_sdk': current_scale,
                      'warning': 'Not calibrated physical-current proof; samples are non-atomic'}), flush=True)
    command += ['-r32', hex(foc), '0x20', '-r32', hex(speed), '0x10',
                '-r32', '0x48000410', '0x4']
start = time.monotonic()
seen = None
previous = None
states = set()
samples = 0
run_samples = []
with args.output.open('x') as log:
    while time.monotonic() - start < args.seconds:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        text = re.sub(r'\x1b\[[0-9;]*m', '', result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError('ESC read failed; no writes attempted: ' + text)
        mem = parse_words(text)
        first, second, ref_word, duration_word = (mem[a] for a in
                                                (0x20000000, 0x20000004,
                                                 0x20000014, 0x20000018))
        ref = ref_word & 0xffff
        ref = ref - 65536 if ref & 0x8000 else ref
        sample = {'seconds': round(time.monotonic() - start, 4),
                  'direct_command': first & 0xff, 'state': (first >> 8) & 0xff,
                  'current_faults': first >> 16, 'latched_faults': second & 0xffff,
                  'speed_reference_sdk': ref, 'ramp_ms': duration_word & 0xffff,
                  'raw_words': [mem[a] for a in sorted(mem)]}
        if args.foc:
            sample.update(decode_foc(mem, foc, speed, current_scale))
            sample['current_feedback_assessment'] = 'unvalidated ADC offset calibration'
            if sample['state'] != 6 or sample['current_faults'] or sample['latched_faults']:
                sample['current_feedback_assessment'] = 'invalid outside fault-free RUN'
                for field in ('ia', 'ib', 'iq', 'id'):
                    sample[field + '_a_firmware_scaled'] = None
            if sample['state'] == 6:
                run_samples.append(sample)
        log.write(json.dumps(sample) + '\n')
        log.flush()
        samples += 1
        states.add(sample['state'])
        signature = tuple(sample[k] for k in ('direct_command', 'state', 'current_faults',
                                             'latched_faults', 'speed_reference_sdk', 'ramp_ms'))
        if signature != previous:
            print(json.dumps(sample), flush=True)
            previous = signature
        if seen is None and (ref != 0 or sample['direct_command'] == 1 or
                             sample['state'] != 0 or sample['current_faults']):
            seen = time.monotonic()
        # Keep several seconds after first event; no automatic second test.
        if seen is not None and time.monotonic() - seen >= 5:
            break
if run_samples:
    keys = ('iq_ref_sdk', 'id_ref_sdk', 'iq_sdk', 'id_sdk', 'vq_sdk', 'vd_sdk',
            'measured_motor_rpm')
    print(json.dumps({'run_samples': len(run_samples),
                      'run_ranges': {k: [min(s[k] for s in run_samples),
                                         max(s[k] for s in run_samples)] for k in keys},
                      'run_hall_raw_states': sorted({s['hall_pb6_pb8_raw'] for s in run_samples})}), flush=True)
print(json.dumps({'samples': samples, 'states_seen': sorted(states),
                  'event_observed': seen is not None, 'last': sample,
                  'log': str(args.output)}), flush=True)
