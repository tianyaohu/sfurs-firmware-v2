# Unexpected three-wheel motion: preliminary root-cause analysis

Status: source/binary analysis plus read-only inspection of the rear-left ESC.
The other two unmodified controllers' images have NOT been read. No drive
commands, resets or flashes during this RCA.
Keep motor power disconnected. Do not reproduce this with energized motors.

## Known event

The Nucleo selected MOTOR_NE bit 0x01 and transmitted one speed request:
standard CAN FD ID 0x081, payload `3C 00 E8 03` (60 motor RPM, 1000 ms ramp).
The explicitly armed test ESC entered RUN, accumulated no Hall transitions,
reported zero RPM, then returned via STOP to IDLE. Recorded sampled peak
`abs(iq)+abs(id)` was 0.744 A in SDK-derived units. The user observed the other
three wheels rotating, apparently quickly. No independent speeds/currents were
measured for those wheels. STOP requests were ID 0x781, NOT broadcast stops.

Only one ESC was flashed with the 0.25 A reference cap, explicit arming gate,
sampled-current guard and one-shot timeout. No such safeguards were established
on the other controllers. Their lack of corresponding telemetry meant the
Nucleo could not establish their physical stop state.

## Repository protocol: NE-only, not a broadcast

`HW/common/CAN/can_datatypes.h` defines NE/SE/SW/NW select bits 1/2/4/8 and
speed-write command 0x080. `0x081 & bit` selects NE only, whereas 0x08F would
select all four. All test/telemetry IDs had select bit 0x01 only:

| ID | Purpose | command bits (`ID & 0x780`) |
| -- | -- | -- |
| 0x081 | speed request | 0x080 |
| 0x101 | speed telemetry request | 0x100 |
| 0x301 | test-current request | 0x300 |
| 0x401 | explicit test arm | 0x400 |
| 0x781 | NE stop | 0x780 |

`motor_config/Src/main.c` and its tracked original both reject received frames
whose ID has no overlap with MOTOR_CAN_ID. Correctly built SE/SW/NW versions of
this source should reject 0x081. The new commands do not decode as speed or
torque writes under the repository's command mask. This does not prove safety
under an unknown older firmware's protocol.

## Strong evidence: pre-existing installed binary differs from local source

The saved 128 KiB original image is
`/home/fh/sfurs_firmware/esc-backups/front-right-before-telemetry-20261012.bin`.
It is from the one board originally identified as front-right, NOT from all four.

Thumb disassembly shows a CAN receive polling path rather than the repository's
address-validating receive callback:

- 0x08001E18 calls the receive-fill helper; 0x08001E2C calls the receive helper.
- 0x08001E30 loads the received ID.
- 0x08001E34 masks it with **0x780**, discarding motor-selection bits.
- 0x08001E38 compares to **0x080**, with speed dispatch at 0x08001EC8.
- 0x08001E3C compares to **0x180** for torque dispatch.
- 0x08001E42 compares to **0x780** for stop dispatch.
- There is no software motor-selection validation in this dispatch path.
- Speed dispatch sign-extends the payload's low half, divides by six (RPM to
  0.1 Hz units) and uses the upper half as ramp time. Thus the backed-up image
  would interpret the test's speed payload as 60 RPM, not a high-speed value.

The image's filter initialization at 0x08001CFA–0x08001D12 writes the structure
at RAM 0x2000040C as standard filter 0, MASK type 2, RX FIFO0 destination 1,
FilterID1=1, FilterID2=1. This selects the NE bit. Its global-filter call at
0x08001D1C–0x08001D26 passes reject values for nonmatching data frames.
Consequently this image relies on a correctly configured hardware filter for
address isolation; its receive handler does not enforce isolation again.

The disassembly is saved at `/tmp/opencode/esc-original-disassembly.txt`.
It demonstrates that trusting repository source as a description of installed
firmware was not justified. It does NOT identify what is installed on the
other three boards.

## Leading hypothesis, not yet confirmed

The other controllers also accepted the NE-addressed speed request, likely
because their flashed images select NE, use a broader/inverted acceptance
filter, or have different address/protocol handling. Multiple boards flashed
with a common NE image would fit the observed event: all see ID 0x081; only
the newly guarded board reacts to its local current guard and stops quickly;
the unmodified controllers have no established test timeout/current cap.

CubeIDE `.cproject` contains a **generic Release configuration hard-coded to
MOTOR_NE**, plus distinct DEBUG_NE/SE/SW/NW configurations. A generic Debug
configuration has no motor define. Thus selecting Release repeatedly without
per-wheel changes is a concrete route to duplicate NE addressing, but the
actual flash history is unknown. The test CLI build itself explicitly selected
NE; it did not program the other boards.

Alternatives to distinguish: stale firmware lacking address isolation, physical
board/wheel mapping errors, another active CAN command source, and unexpected
startup/button behavior. The backup's initialization also requests a start
after programming zero speed; this is another difference from current source,
but does not by itself explain three fast wheels.

## Why the reported fast motion is still unresolved

The transmitted target was 60 RPM. The backed-up firmware's speed decoding
agrees with that interpretation. We cannot equate the user's observation to a
particular measured speed or claim a proven unit-conversion error. Different
installed motor feedback/tuning/control code, rejected ramps retaining a
default reference, another command source, or poor speed regulation are
possible, but none is established for those three boards. The repository
normal profile's default speed is 1071 RPM and current reference bounds are
3 A; these are not verified values in the other installed binaries.

## Confirmed procedural/design failures

1. Other motor power stages were not physically isolated during a single-wheel
   test on an unverified shared CAN network.
2. Installed addresses/protocols on all four ESCs were not audited before a
   drive command. A repository definition and one board's telemetry cannot
   establish unique addressing on the others.
3. The current cap, arming gate and stop timer existed on only one board.
4. Observed NE telemetry was treated as sufficient to assess the whole robot.
5. An NE-addressed stop was not an independently verified all-wheel emergency
   stop. A CAN ACK can come from any node and is not application acceptance.
6. Software reference limits and sampled-current monitoring were not
   independently validated as a 0.5 A instantaneous phase-current ceiling.

## Safe confirmation plan

With the battery disconnected, USB-power and read each ESC individually through
ST-LINK. Record probe serial, physical wheel label and full original flash
backup BEFORE any firmware changes. Compare images and inspect each image's
filter/selection checks, command decoding, default target, feedback scaling,
current limits and stop behavior. Do not infer identity solely from reply IDs.
If images cannot be analyzed conclusively, use an isolated motor-power-disabled
setup with CAN receive logging/instrumentation, never an energized drive test.
Do not send drive frames to the shared robot bus to discover addresses.

Before another powered test, physically disconnect power to all non-test
controllers, establish address uniqueness and protocol compatibility, and
validate current sensing and an independent accessible power cutoff. Any
firmware remediation or further hardware test needs separate authorization.

## Confirmed rear-left result

User identified the USB-connected board as rear-left; ST-LINK serial
`0667FF525266555067073424`. Full 128 KiB flash was read in HOTPLUG mode to
`/home/fh/sfurs_firmware/esc-backups/rear-left-original-20261012.bin`.
SHA256: `54176551391c671089f775ff76715e1dd1dfead3f0f794e5076c3680280e7b4a`.
The entire image is BYTE-IDENTICAL to the front-right pre-telemetry backup.

Live read-only filter inspection confirms this physical rear-left board selects
NE bit 0x01, not SW bit 0x04:

- Filter structure at 0x2000040C: `0 0 2 1 1 1`, i.e. standard, index 0,
  MASK, FIFO0, FilterID1=1, FilterID2=1.
- Hardware standard-filter word at 0x4000A400: `0x88010001`, encoding
  MASK type 2, FIFO0 action 1, ID=1 and mask=1.
- Acceptance condition is `(received_ID & 1) == 1`; ID 0x081 matches.

Therefore duplicate NE addressing is CONFIRMED for rear-left and the original
front-right image. This explains why rear-left could respond to the test's
NE speed command, independently of physical wiring labels. Installed firmware
on rear-right and front-left still must be read before asserting this is the
cause on those two boards. Fast rotation is still not quantified/explained.
No firmware or RAM writes, reset, calibration or drive commands were issued
to the rear-left board. USB readback alone does not verify physical wheel mapping
beyond the user's label.

## Rear-left remediation

An explicitly authorized address-only binary patch was flashed with battery
disconnected to rear-left probe `0667FF525266555067073424`. Full image download
verification and reset succeeded. Post-reset RAM confirms FilterID1/FilterID2=4
and hardware filter word=0x88040004. Global filter register remains 0x0001002B.
Thus NE-only ID 0x081 no longer matches this board's standard filter; SW ID
0x084 matches. No drive frames were sent to test this. Original motor parameters,
current limits, control and startup logic remain unchanged. This does not
correct or validate the remaining boards. Patch details are under
`address_patches/rear_left_sw/README.md`; original backup is preserved.

## Confirmed rear-right result

User identified the next USB-connected board as back-right/rear-right (SE).
ST-LINK serial: `066AFF525266555067075126`. Full 128 KiB read-only backup:
`/home/fh/sfurs_firmware/esc-backups/rear-right-original-20261012.bin`.
SHA256: `54176551391c671089f775ff76715e1dd1dfead3f0f794e5076c3680280e7b4a`.
The image is byte-identical to both the original front-right and original
rear-left images. Live filter fields at 0x2000040C are `0 0 2 1 1 1`;
hardware word at 0x4000A400 is `0x88010001`, global filter is `0x0001002B`.
This independently confirms rear-right also accepts NE bit 0x01 rather than
its expected SE bit 0x02, explaining its acceptance of ID 0x081.
No flash, reset or motor commands were issued. Rear-right remains uncorrected;
front-left remains uninspected. Earlier source-only hypotheses are superseded
by these readbacks for the two rear controllers, not for front-left.

## Rear-right remediation

User authorized the address-only SE correction and confirmed battery disconnected.
Patched full-image SHA256:
`7ebf17b5e90ad7ffcdca4b091d5aaca0cb6e14d2511be1aeda69d18cb31099ba`.
Output: `/home/fh/sfurs_firmware/esc-backups/rear-right-SE-address-only-20261012.bin`.
The SHA-locked preparation script now supports `--motor SE` for this identical
original image, using selector 2; its default SW selector remains 4.
Download, verification and reset succeeded on `066AFF525266555067075126`.
Live readback confirms filter fields `0 0 2 1 2 2`, hardware word `0x88020002`,
unchanged global filter `0x0001002B`. NE-only ID 0x081 no longer matches;
SE speed ID 0x082 matches. No CAN drive commands were sent. Motor tuning,
startup logic and current limits are unchanged. Front-left remains uninspected.

## Confirmed front-left result

User corrected the connected wheel label to front-left (NW); ST-LINK
`066AFF515266555067023255`. Read-only full flash backup:
`/home/fh/sfurs_firmware/esc-backups/front-left-original-20261012.bin`.
SHA256 `54176551391c671089f775ff76715e1dd1dfead3f0f794e5076c3680280e7b4a`.
It is byte-identical to all three other original ESC images. Live fields are
`0 0 2 1 1 1`, hardware filter `0x88010001`, global filter `0x0001002B`.
Front-left therefore also selects NE bit 1, not expected NW bit 8.
All four original images were identical and NE-addressed; duplicate addressing
is now confirmed on each of the three unintentionally rotating controllers.
This explains acceptance of the NE speed command on all three, while their
reported fast rotation remains unquantified. Front-left is not yet corrected;
no flash, reset or motor commands were issued during this inspection.

## Front-left remediation

## Physical wheel mapping correction — subsequent user report

### Final front-left verification and authoritative mapping

User reconfirmed front-left connected, battery off. Probe
`066AFF515266555067023255` full 128 KiB readback exactly matches NW address-only
image SHA256 `6dadecfa5ecf1a0a3aa63d336e474239a3aa7071a5b07ac4ce1388570eb7df47`.
Live fields `0 0 2 1 8 8`, hardware `0x88080008`, global `0x0001002B`.
No reflash needed; no reset or motor commands issued.

The final user-corrected physical mapping supersedes earlier wheel labels:

| Physical wheel | Motor / bit | ST-LINK serial |
| --- | --- | --- |
| Front-right | NE / 1 | 066AFF525266555067075126 |
| Back-right | SE / 2 | 0667FF525266555067073424 |
| Back-left | SW / 4 | 066FFF525266555067073058 |
| Front-left | NW / 8 | 066AFF515266555067023255 |

All four individually verified firmware/filter selectors are now distinct and
correct for the user's physical labels. All retain original motor control/current
limits (three address-only patches, one original NE image); none retains the
guarded low-current test profile or its local timed stop. No new 1 A cap installed.
This is addressing verification only, not powered-drive safety validation.
Machine-readable record: `esc-backups/physical-wheel-mapping-verified-20261012.json`.

### Physically confirmed front-right NE restoration

User explicitly authorized restoring/flashing the original backup to ensure NE.
Probe `066AFF525266555067075126`: original full-image hash verified, download,
full-image verification and reset succeeded. Post-reset HOTPLUG fields
`0 0 2 1 1 1`, hardware `0x88010001`, global `0x0001002B` confirm NE=1.
Original motor settings/current limits restored, no guarded test profile added.
No motor commands sent. Record:
`esc-backups/physical-front-right-NE-restoration-20261012.json`.
Current physically confirmed mapping: front-right NE=1, back-right SE=2,
back-left SW=4. Front-left still requires physical reconfirmation after the
earlier mapping correction; do not treat this as powered-test validation.

### Physically confirmed front-right read-only check

User identifies probe `066AFF525266555067075126` as front-right (NE), superseding
its earlier rear-right label. Read-only full flash dump
`/tmp/opencode/physical-front-right-check-20261012.bin` exactly matches the
original-based SE address-only image, SHA256
`7ebf17b5e90ad7ffcdca4b091d5aaca0cb6e14d2511be1aeda69d18cb31099ba`.
Live fields `0 0 2 1 2 2`, hardware `0x88020002`, global `0x0001002B`.
Physically front-right currently incorrectly selects SE=2; expected NE=1.
It duplicates physically confirmed back-right. Restoring this probe's original
backup (historically `rear-right-original-20261012.bin`) would restore NE=1
without changing original motor parameters. No flash/reset/motor commands
performed during this check; correction awaits explicit authorization.

### Physically confirmed back-right correction

User authorized correcting/reflashing probe `0667FF525266555067073424` to SE
and confirmed battery off. Verified existing SE image against this probe's
original backup (historically labelled rear-left); original motor bytes are
unchanged outside the address patch. Download/full verification/reset succeeded.
Live fields `0 0 2 1 2 2`, hardware `0x88020002`, global `0x0001002B`.
Record: `esc-backups/physical-back-right-SE-correction-20261012.json`.
No motor commands sent. Physically confirmed rear mapping is now back-left
`066FFF525266555067073058` SW=4, back-right `0667FF525266555067073424` SE=2.
Remaining front boards must be physically reidentified; earlier SE-labelled
probe `066AFF525266555067075126` still has SE firmware, so unique fleet
addressing is not yet verified.

### Subsequent physical back-right check

User identifies connected probe `0667FF525266555067073424` as back-right (SE),
superseding its earlier back-left label. Read-only full flash dump
`/tmp/opencode/physical-back-right-check-20261012.bin` matches the SW address-only
image exactly (SHA256 `b2c8c06f30c6583601bf0f2e70d3390c3139bc05f0c2d9d10f901c30803cd512`).
Live filter fields `0 0 2 1 4 4`, hardware `0x88040004`, global `0x0001002B`.
This physically back-right controller is incorrectly SW-addressed; expected SE=2.
No flash, reset or motor commands performed in this check. Both physically
confirmed rear boards currently select SW; powered testing remains pending
correction and verification of remaining physical mappings.

User now positively identifies probe `066FFF525266555067073058` as physically
back-left (SW), not front-right as previously labelled. The earlier physical
mapping and statement that all physical wheel addresses are correct must not
be relied upon until the other probes are reidentified. Read-only full flash
dump `/tmp/opencode/physical-back-left-current-20261012.bin` SHA256
`5be237c43d7c0460fbe544300abc62ccf1aa257a2e82e02a9d10443f98adf5c8`
matches every flash byte generated from the guarded NE ELF. This physically
back-left board therefore currently has the guarded NE profile, not the legacy
SW address-only image. No flash or motor command issued during this check.
The legacy SHA-locked address patch cannot be applied to this different image.

## Earlier front-left remediation record

### Subsequent physically confirmed back-left restoration

User explicitly requested original backup rather than retaining the guarded
0.25 A profile. Prepared the SHA-locked SW address-only image from this probe's
original backup (historically named `front-right-before-telemetry-20261012.bin`).
Flashed `physical-back-left-SW-from-original-20261012.bin` to
`066FFF525266555067073058`; download, full verification and reset succeeded.
Live filter fields `0 0 2 1 4 4`, hardware `0x88040004`, global `0x0001002B`.
Guarded profile, its current cap and local timed stop are removed; original
motor settings/current limits restored, no new 1 A cap. No motor commands sent.
Battery confirmed disconnected. Probe `0667FF525266555067073424` still also
has SW firmware from the earlier incorrectly labelled correction. Its actual
physical wheel must be reidentified/corrected before a powered SW test; the
current fleet is NOT confirmed uniquely addressed.

User authorized the address-only NW correction and confirmed battery off.
Prepared image SHA256:
`6dadecfa5ecf1a0a3aa63d336e474239a3aa7071a5b07ac4ce1388570eb7df47`.
Image: `/home/fh/sfurs_firmware/esc-backups/front-left-NW-address-only-20261012.bin`.
The shared SHA-locked preparation script supports `--motor NW` (selector 8),
preserving every byte outside the same redirect and patch tail.
Download, verification and reset succeeded on `066AFF515266555067023255`.
Live readback confirms filter fields `0 0 2 1 8 8`, hardware `0x88080008`,
global filter `0x0001002B`. Motor settings/current limits are unchanged;
no motor commands sent. The three corrected legacy ESCs now select SW=4,
SE=2, NW=8 respectively; the source-built front-right test profile selects NE=1.
This completes per-board address inspection/correction, not powered-drive or
current-limit validation. A guarded SW sender/ESC test remains unprepared.
