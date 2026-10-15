# Firmware-update regression and hardware matrix

The first 232-byte blink transfer proved the v1 happy path. It did not prove
retry safety, remotely initiated entry, or interrupted-flash recovery. The
review fixes revise the wire protocol; host and target must be upgraded
together before another hardware transfer. No automatic CI hardware flashing.

## Automated gates

| Boundary | Required cases |
|---|---|
| Host ACK processing | Reply before enqueue; duplicate previous command before/after next enqueue; wrong command, session, device, detail/sequence; session rollover; no outstanding request |
| Target session | Invalid device/size/range; duplicate ENTER; duplicate DATA without reprogramming; changed duplicate payload; missing/out-of-order DATA; idle timeout and tick wraparound |
| Flash commit | Duplicate VERIFY does no programming; mismatched image is not blessed; bad magic/complement, CRC, vectors; torn metadata stays invalid |
| RX buffer/parser | 0/4/8/12/64-byte frames; standard/extended/remote, FD/classic, BRS; oversized input cannot change update state |
| Entry request | Exact single-node ID and magic/version; running/armed/calibrating rejection; matching mailbox consume once; bad magic/complement/device rejection; stack/BSS cannot overlap retained mailbox |
| Main-loop timing | Real pump uses fresh clock after 191ms CRC initialization; wraparound; faulted host is not pumped |
| Packaging | Actual blink ELF vector address, stack and reset bounds, size, CRC, binary-to-C byte identity; canonical/vendor protocol comparison |
| Existing operation | Wheel bench and sender guard suites remain passing |

`test_fwu_main_timing.py` compiles the actual main pump block with a simulated
clock, rather than relying only on a source-string assertion. Artifact checks
build the real ARM binary. They do not simulate the electrical CAN bus or flash
controller. Target-side metadata/entry tests use injected storage/transport;
real erase/program timing and brownout behavior still need hardware validation.

## Hardware gates (explicit operator authorization)

1. **Protocol upgrade smoke test:** install host and target together; request
   ENTER only and inspect command/session/status. Never test mixed versions by
   allowing an erase.
2. **Remote re-entry:** launch a valid app, request boot entry over CAN with no
   SWD reset, transfer another blinker; repeat a second update in the same host
   runtime. Confirm no motor gate activity throughout.
3. **Cold boot:** power completely off/on; verify CRC/metadata validation and
   entry into the newly installed app. Software reset alone does not cover it.
4. **Interrupted update:** cut power at app erase, mid-DATA, just before metadata
   commit, and during its final double-word. In each case reboot must never
   execute an incomplete image and must accept a fresh CAN update.
5. **Lost transport:** disconnect CAN; reset host; stop host; reconnect and issue
   a NEW session. Verify no silent erase restart or replay-driven advancement.
6. **Scale/load:** realistic 44KiB and maximum-size images, other ESC traffic,
   multiple consecutive updates, late/duplicate replies, bus-off recovery.
7. **Wrong target:** other three ESCs remain unchanged, including their metadata
   and per-wheel identity. CAN TX acknowledgement alone never proves this.

Start with motor-disabled payloads and SWD recovery available. A CRC is error
detection, not authentication. Signed firmware, anti-rollback and A/B images
remain separate requirements, not guarantees supplied by these tests.
