# Commit message (torque turn-count work)

```
esc1: add guarded torque turn-count diagnostic for 300-turn bench test

Add opt-in --torque-count-diagnostic NE profile with fixed positive
torque reference, IRQ-fed Hall edge counting, 14,400-edge (300-turn)
count stop, 2 s no-progress stall stop, invalid-Hall stop, 3 A
reference/sampled-current guards, 1000 RPM overspeed guard, 180 s
backup timeout, explicit CAL1/DARM/DRUN/DSTP sequencing and one-shot
latch. Reject legacy speed frames in guarded profiles.

Host suites cover count/invalid/stall/timeout/current/speed/fault
gates and legacy rejection. Document preparation, flashes,
calibrations and measured outcomes.
```

## What was done

- Front-right (NE) only; 8 pole pairs => 48 Hall edges/turn; 300 turns = 14,400 edges.
- Torque references tried: 1.5 A (fast overspeed), then 0.8 A (observable ~3 turns).
- Measured: +torque drives reverse; 0.8 A rerun gave 133 powered edges,
  948 RPM peak, 1000 RPM guard stop; coasting after cut is not counted.
- Installed ELFs under `/tmp/opencode/esc-ne-torque-count-*`; docs in
  `TORQUE_COUNT_VALIDATION.md`, `RPM_VALIDATION.md`, `HALF_AMP_DIAGNOSTIC.md`.

## How to replicate (battery OFF unless stated)

```sh
cd HW/B-G431B-ESC1/motor_config
python3 test_torque_count_guards.py
python3 test_rpm_guards.py
python3 test_half_amp_guards.py
PATH=/tmp/opencode/cm7-toolchain/root/usr/bin:$PATH \
python3 build_cli.py --motor NE --torque-count-diagnostic \
  --output /tmp/opencode/esc-ne-torque-count-0p8a
```

Flash (battery OFF, front-right probe `066AFF525266555067075126`):

```sh
CLI=$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
$CLI -c port=SWD sn=066AFF525266555067075126 \
  -w /tmp/opencode/esc-ne-torque-count-0p8a/motor_NE_torque_count_diagnostic.elf -v -rst
$CLI -c port=SWD mode=HOTPLUG sn=066AFF525266555067075126 \
  -r32 0x200008d0 0x70 -r32 0x200000d4 0x8
```

Powered single run (wheels clear, cutoff ready, Nucleo unpowered):

1. Reconnect battery, verify IDLE/fault-free via HOTPLUG read.
2. Write CAL1 `0x43414C31` to `0x200008D0`; expect CLI exit 1 but consumed.
   Readback `result=2/valid=1`; if `result=3`, repeat once after IDLE.
3. Write DARM `0x4441524D` to `0x200008E8`; verify `armed=0x434E5431/result=1`.
4. Write DRUN `0x4452554E` exactly once; monitor state/Hall/peak until IDLE.
   One-shot `used=1` blocks reruns until battery-off reset.
5. Disconnect battery; record log from `/tmp/opencode/front-right-torque-count-*.jsonl`.
```

Note: repo has unrelated modified/untracked files outside this work;
commit only torque files after review (`git add` selected paths).
