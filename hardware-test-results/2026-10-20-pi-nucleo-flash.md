# Nucleo flashing through Pi (2026-10-20, Codex)

- Firmware: CM7 drive built without warnings from `015b937c2f1da84122897b3b0ec44b602993354a`; ELF SHA256 `5d1ce20866b7240c9112a274e6a04ad82237a4f32dd2e45b8951eaf68848609a`.
- Setup: user confirmed battery disconnected; Pi 4 `ssl-pi`, aarch64; ST-LINK `002200364142500620353451`; target supply approximately 3.293 V; no other probe owners detected. No motor commands sent.
- Procedure: key-authenticated SSH; OpenOCD backup of bank 1, program CM7 ELF, verify, reset; read runtime symbols without halting.
- Result: programming and verification passed. `uwTick` increased from `0x821d` to `0x8319` across a 250 ms observation; `ik_drive_state=0` (stopped). Probe released. ESC and CM4 flash were not written. This checks deployment and firmware execution, not robot motion accuracy.
- Data: Pi `/tmp/nucleo-cm7-flash.mc0h31/` contains flashed ELF and 1 MiB `cm7-bank1-before.bin` backup; PC build `/tmp/pi-nucleo-flash.5u1FrW/`. These temporary paths are not durable storage.
