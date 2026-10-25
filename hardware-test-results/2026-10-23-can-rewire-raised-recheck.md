# CAN wire repair and raised-wheel re-check (2026-10-23)

- Firmware: unchanged from `hall-100hz-install` (`9720d91`); app CRCs FR `34887E92`, BR `0B8442A5`, BL `94E17FEF`, FL `C85C810A` read back over CAN, no reflash.
- Setup: wheels raised (user's word), battery on, Vbus 11.32-11.39 V. A CAN wire broke; BL and FL gave "no reply" on `esc_tool status` (twice, 3 s apart) while FR and BR answered. User repaired and re-wired it; afterwards all four IDLE, no faults, Hall shifts -60/-64/-59/-62 known.
- Procedure: `esc_tool spin W --rpm +-30 --seconds 2` per wheel, default gains (kp 1024, ki 553), then broadcast stop.
- Result: 8/8 runs correct mean direction, 0 wrong-sign samples, no faults. Mean RPM +30/-30: FR 25.8/-24.2, BR 22.3/-23.3, BL 26.2/-22.8, FL 26.2/-23.1; peak Iq ref 0.44-0.69 A; Hall-speed sd 20-25 RPM (not a low-speed tracking result). Matches the pre-break run. Restart counters read 0 on this link. Final: all IDLE/disarmed, 0 RPM, 0 A.
- Data: `/home/fh/sfurs_firmware/hw-data/2026-10-23-hall-100hz-raised-rewire-044323Z/`.
- Scope: `spin` bypasses the Nucleo body-command stream; no 100 Hz timing and no mat run yet.
