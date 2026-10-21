/**
 * @file can_datatypes.h
 * @brief CAN protocol shared by the motor controllers (B-G431B-ESC1) and the Nucleo.
 *
 * 11-bit ID = command (top 4 bits) | device select bits (bottom 7 bits).
 * Example: 0b001 0000 1000 is STATUS_READ to MOTOR_NW; 0x10F addresses all.
 * Frames are CAN FD format without BRS. Commands carry 4 bytes; replies 8.
 * Motion and arming use the 8-byte CONTROL frame of motor_protocol.h.
 * All multi-byte fields are little endian.
 */
#ifndef CAN_DATATYPES_H
#define CAN_DATATYPES_H

/** Little-endian u32 from four payload bytes. */
#define CAN_U32_FROM_PAYLOAD(p) \
  ( ((uint32_t)((p)[0]))        | \
    ((uint32_t)((p)[1]) << 8)   | \
    ((uint32_t)((p)[2]) << 16)  | \
    ((uint32_t)((p)[3]) << 24) )

/** Store @c v as four little-endian payload bytes. */
#define CAN_PAYLOAD_FROM_U32(p, v)         \
  do                                       \
  {                                        \
    (p)[0] = (uint8_t)((v) & 0xFFU);       \
    (p)[1] = (uint8_t)(((v) >> 8) & 0xFFU);  \
    (p)[2] = (uint8_t)(((v) >> 16) & 0xFFU); \
    (p)[3] = (uint8_t)(((v) >> 24) & 0xFFU); \
  } while (0)

/** Command part (top 4 bits) of an 11-bit ID. */
#define CAN_COMMAND_FROM_ID(p) ((uint32_t)(p) & 0b11110000000)

/** Device select bits */
#define NUCLEO_CAN_ID   (0U)
#define MOTOR_NE_CAN_ID (1U << 0) /**< select bit of the NE wheel (FR) */
#define MOTOR_SE_CAN_ID (1U << 1) /**< select bit of the SE wheel (BR) */
#define MOTOR_SW_CAN_ID (1U << 2) /**< select bit of the SW wheel (BL) */
#define MOTOR_NW_CAN_ID (1U << 3) /**< select bit of the NW wheel (FL) */

/* 0x080 (SPEED_WRITE) and 0x780 (MOTOR_STOP) are retired and never reused. */

/** STATUS_READ (any payload) -> STATUS_REPLY:
 * int16 RPM, int16 Iq, int16 Iq reference (SDK counts, ESC_COUNTS_PER_AMP),
 * uint8 MCSDK state, uint8 flags (ESC_FLAG_*, low nibble = last ESC_STOP_*). */
#define STATUS_READ  (2U << 7)
#define STATUS_REPLY (5U << 7) /**< reply to STATUS_READ */
/** DIAG_READ (any payload) -> DIAG_REPLY:
 * uint16 Vbus centivolts, uint16 occurred MCSDK faults,
 * int16 Hall phase shift degrees, uint8 align result (ESC_ALIGN_*),
 * uint8 I2T load percent. */
#define DIAG_READ    (6U << 7)
#define DIAG_REPLY   (7U << 7) /**< reply to DIAG_READ */
/** HALL_ALIGN: payload "HALN". From IDLE only, the wheel turns ~1/4 rev
 * forward and back under forced-angle current, then the measured Hall phase
 * shift is applied in RAM (lost on reset). Read the result with DIAG_READ. */
#define HALL_ALIGN   (9U << 7)
#define HALL_ALIGN_MAGIC 0x4E4C4148U /**< "HALN" */
/** CONFIG_SET: uint16 key (ESC_CFG_*), int16 value. IDLE only, RAM only. */
#define CONFIG_SET   (10U << 7)
/** IDENT_READ (any payload) -> IDENT_REPLY: uint32 app CRC32, uint32 app size,
 * from the bootloader's validity record. The bootloader checks the flash
 * against that record before every jump, so a running app is that image. */
#define IDENT_READ   (3U << 7)
#define IDENT_REPLY  (11U << 7) /**< reply to IDENT_READ */
/** CAPTURE: one current-loop step test (no torque) and its readout.
 *  word = ESC_CAPTURE_START << 24 | step mA: from IDLE only, hold the rotor at
 *   a forced electrical angle with ESC_CAPTURE_HOLD_MA of d-axis current, then
 *   step the Id reference to step mA (<= IQMAX) for the rest of
 *   ESC_CAPTURE_SAMPLES FOC periods, recording every period. Iq ref stays 0.
 *  word = ESC_CAPTURE_READ << 24 | bank << 16 | count << 10 | first:
 *   count (1..32) CAPTURE_REPLY frames, uint16 tag (bank << 12 | sample) and
 *   three int16:
 *     bank 0: Id, Iq (counts, ESC_COUNTS_PER_AMP), Vd (s16V)
 *     bank 1: Vq (s16V), Id reference (counts), electrical angle (s16)
 *     bank 3, sample 0: status (ESC_CAPTURE_*), Vbus centivolts at the step,
 *       samples recorded.
 *   s16V: 32767 = Vbus / sqrt(3) phase-voltage amplitude. */
#define CAPTURE      (4U << 7)
#define CAPTURE_REPLY (14U << 7) /**< reply frame of a CAPTURE readout */
#define ESC_CAPTURE_START 0xC5U /**< command byte that starts a current-loop capture */
#define ESC_CAPTURE_READ  0x3DU /**< command byte that reads capture or recorder data */
#define ESC_CAPTURE_SAMPLES 512U /**< FOC periods recorded per capture */
#define ESC_CAPTURE_PRE 64U       /**< samples at the hold current before the step */
#define ESC_CAPTURE_HOLD_MA 1000U /**< d-axis hold current before the step, mA */
#define ESC_CAPTURE_INFO_BANK 3U /**< readout bank holding status, Vbus and sample count */
enum { ESC_CAPTURE_NONE = 0, ESC_CAPTURE_RUNNING = 1, ESC_CAPTURE_DONE = 2,
       ESC_CAPTURE_ABORTED = 3, ESC_CAPTURE_REJECTED = 4 };
/** CAPTURE word = ESC_RECORD_START << 24 | delay ms << 9 | pre: arm the
 * speed-loop recorder. It only observes (allowed while driving; rejected during
 * a current-loop capture, which it shares memory with). It records one sample
 * per speed-loop tick (1 kHz), triggers when the speed reference leaves zero
 * (|ref| >= 2 RPM, after first being zero) plus delay ms (0..4095), and keeps
 * pre (0..ESC_RECORD_SAMPLES-1) samples from before the trigger point. Read
 * with ESC_CAPTURE_READ: banks 0/1 carry channels 0-2/3-5, bank 0 with
 * ESC_RECORD_READ_CH67 set carries channels 6, 7, 0 (the Nucleo relay keeps
 * banks 0/1 only); bank 3 sample 1: status (ESC_RECORD_*), samples, trigger
 * sample index.
 * Channels: ramped ref RPM, measured RPM, P, I, ff, Iq reference, measured Iq
 * (counts), packed: bits 0-1 startup-loop state (DSC_*), 2-5 reason, 6 P cap,
 * 7 startup loop enabled, 8-11 net Hall edges (+ minus -) mod 16, 12-14 Hall
 * captures mod 8, 15 speed loop ran this tick (else ref, P, I, ff, Iq ref are
 * 0; state/reason keep the last values, e.g. FAILED). Legacy loop: I from the
 * PI integral, P = PI out - I, ff = Iq reference - PI out. */
#define ESC_RECORD_START 0x5EU
#define ESC_RECORD_SAMPLES 384U /**< speed-loop ticks recorded */
#define ESC_RECORD_INFO_SAMPLE 1U /**< bank 3 sample index holding the recorder status */
#define ESC_RECORD_READ_CH67 (1U << 18) /**< in the ESC_CAPTURE_READ word */
enum { ESC_RECORD_NONE = 0, ESC_RECORD_ARMED = 1, ESC_RECORD_TRIGGERED = 2,
       ESC_RECORD_DONE = 3, ESC_RECORD_REJECTED = 4 };

#define ESC_COUNTS_PER_AMP 544 /**< MCSDK current counts per ampere */

/** CURRENT_KP/KI set both the Iq and Id PI gains (dividers TF_KPDIV/TF_KIDIV).
 * FF_STATIC/KINETIC_MA: friction feed-forward added to the speed loop's Iq
 * reference (0..IQMAX mA; see drive_app.h). */
enum { ESC_CFG_SPEED_KP = 1, ESC_CFG_SPEED_KI = 2, ESC_CFG_HALL_SHIFT_DEG = 3,
       ESC_CFG_IQ_MAX_MA = 4, ESC_CFG_CURRENT_KP = 5, ESC_CFG_CURRENT_KI = 6,
       ESC_CFG_FF_STATIC_MA = 7, ESC_CFG_FF_KINETIC_MA = 8,
       /** Startup speed loop (drive_speed_ctrl.h), IDLE only, RAM only:
        * ENABLE 0/1; *_MA 0..IQMAX mA; SLOPE mA per ms; *_MS ms; UPDATES n. */
       ESC_CFG_DSC_ENABLE = 9, ESC_CFG_DSC_FF_RUN_MA = 10, ESC_CFG_DSC_FF_PER_KRPM_MA = 11,
       ESC_CFG_DSC_START_FROM_MA = 12, ESC_CFG_DSC_START_CAP_MA = 13,
       ESC_CFG_DSC_START_SLOPE_MA = 14, ESC_CFG_DSC_START_HOLD_MS = 15,
       ESC_CFG_DSC_PCAP_MA = 16, ESC_CFG_DSC_PCAP_MS = 17, ESC_CFG_DSC_PCAP_UPDATES = 18,
       ESC_CFG_DSC_PCAP_FALLBACK_MS = 19,
       /** Hall reversal filter on the speed fed to either loop (drive_speed_ctrl.h
        * dsc_reversal_filter), 0/1, default on. */
       ESC_CFG_SPEED_REV_FILTER = 20,
       /** Startup loop hand-off hold decay, mA per ms, 0..IQMAX (0 = hold until
        * the positive-P cap releases). */
       ESC_CFG_DSC_HOLD_DECAY_MA = 21,
       /** Startup loop motion confirmation, ms (0 = hand off at the first
        * edge, 0..2000), and breakaway preload, % of the last confirmed
        * hand-off current per direction (0..90, 0 = off). */
       ESC_CFG_DSC_WATCH_MS = 22, ESC_CFG_DSC_PRELOAD_PCT = 23,
       /** Low-speed speed from Hall edge timing for either loop, 0/1. */
       ESC_CFG_SPEED_EDGE_EST = 24 };
enum { ESC_STOP_NONE = 0, ESC_STOP_COMMAND = 1, ESC_STOP_TIMEOUT = 2,
       ESC_STOP_FAULT = 3, ESC_STOP_ALIGN = 4, ESC_STOP_CAPTURE = 5,
       ESC_STOP_PARKED = 6 /**< zero command at standstill; next non-zero restarts */,
       ESC_STOP_START_FAILED = 7 /**< startup loop: no breakaway or no Hall speed;
                                    latched until a zero command, STOP or gap */ };
#define ESC_FLAG_ACTIVE  0x10U /**< streaming speed commands accepted */
#define ESC_FLAG_DERATED 0x20U /**< I2T limit reached, current derated */
#define ESC_FLAG_ALIGN   0x40U /**< forced-angle test running: Hall alignment or capture */
#define ESC_FLAG_ALIGNED 0x80U /**< Hall shift known: built in or measured this boot */
enum { ESC_ALIGN_NONE = 0, ESC_ALIGN_OK = 1, ESC_ALIGN_REVERSED = 2,
       ESC_ALIGN_NO_EDGES = 3, ESC_ALIGN_ABORTED = 4, ESC_ALIGN_RUNNING = 5 };

#endif
