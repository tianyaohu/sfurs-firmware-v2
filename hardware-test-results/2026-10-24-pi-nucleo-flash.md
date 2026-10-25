# Nucleo build and flash from the robot Pi  (2026-10-24, claude with Frank H)
- Firmware: `develop_ai` `b206474` (skynet serial link), built on the Pi with ARM GCC 14.2: `scripts/build.sh`
  0 warnings, drive ELF SHA256 `f45faa84531695e2…` (the PC's GCC 10.3 build differs byte-wise). Replaced Nucleo
  drive image `abc51ecf…` (9720d91 install). ESCs not touched.
- Setup: Pi 4 `ssl-pi` powered from a PC USB port, Nucleo ST-LINK `002200364142500620353451` on the Pi's USB.
  Battery OFF: `esc_tool status` through the installed ELF read all four links, no ESC replied on CAN.
- Procedure: OpenOCD 0.12 `dump_image` of 0x08000000 +1 MiB (8.4 s), backup prefix equal to the installed ELF's
  binary; then `program <elf> verify reset exit` (the commands now in `scripts/flash_nucleo.sh`).
- Result: Verified OK; after reset `esc_tool status` with the new ELF reads the mailbox ABI and all four link
  records. Throttle flags stayed `0x0` (max 55.5 °C) through the 2-job build. Earlier the same day the Pi left
  the network four times at the start of full-load jobs (544 MB download, 4-job build); cause not captured
  (flags reset at boot, journal not persistent).
- Data: `/home/fh/sfurs_firmware/hw-data/2026-10-24-pi-nucleo-flash/` (backup .bin, flash/build/power logs);
  ELF in `nucleo-installed/drive-b206474-pi-gcc14.elf`.
