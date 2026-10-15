# Provenance: HW/B-G431B-ESC1 ESC firmware in this repo

Source: local reference clone /home/fh/sfurs_firmware/firmware-v2,
branch `fwu-rollout-autocal` @ 54e9a45 (can-bootloader + auto-CAL-all +
0x480 CAN-CAL RX + offset app builds). That branch is UNPUSHED; the
reference clone is scratch — this tree is now the maintained copy.
Future ESC work happens HERE on develop, not in the reference clone.

Scope imported: motor_config/ (Src Inc Drivers MCSDK_v6.4.1-Full
STM32CubeIDE ftl build_cli.py guard tests), bootloader/, Inc/ Src/
Includes/ (motor_controller_can), HW/common/CAN, run-history docs.
Deliberately EXCLUDED: ElectronicSpeedControl_ESC-G4/ (legacy CubeIDE
project with checked-in build artifacts), HW/STM32H5 + HW/STM32H7
(unrelated board lines), address_patches/ blobs (superseded ID-patch era).

Known duplications (do NOT unify without a plan):
- CAN FWU protocol: Common/{Inc,Src}/can_fwu_* (Nucleo/host side) vs
  HW/B-G431B-ESC1/bootloader/can_fwu_* (vendored standalone copy for the
  bootloader, which cannot depend on Common/). Keep in sync manually.
- G4 HAL Drivers/ here vs H7 Drivers/ at repo root: different MCUs,
  both required.
- motor_config/.gitignore + .mxproject came along; repo-root .gitignore
  already excludes *.o/*.elf/Debug/build outputs.
