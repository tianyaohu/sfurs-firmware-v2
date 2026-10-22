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
/* (10U << 7) was CONFIG_SET: retired, replaced by protocol v1 CONFIG (motor_protocol.h); never reuse. */
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
 * (|ref| >= 10 RPM, MOTOR_SPEED_MOVING_RPM, after first being below it) plus delay ms (0..4095), and keeps
 * pre (0..ESC_RECORD_SAMPLES-1) samples from before the trigger point. Read
 * with ESC_CAPTURE_READ: banks 0/1 carry channels 0-2/3-5, bank 0 with
 * ESC_RECORD_READ_CH67 set carries channels 6, 7, 0 (the Nucleo relay keeps
 * banks 0/1 only); bank 3 sample 1: status (ESC_RECORD_*), samples, trigger
 * sample index.
 * Channels (motor_speed.h): applied ref RPM, speed estimate RPM, P, I, ff, Iq reference, measured Iq
 * (counts), packed: bits 0-1 loop mode (ESC_REC_MODE_*), 2 estimate valid, 3 request saturated,
 * 4 over-speed bleed this tick, 5 stall timer counting, 6-7 0, 8-11 net Hall edges (+ minus -)
 * mod 16, 12-14 raw Hall state (HALL_M1.HallState, 1..6; 0/7 = invalid) so tools know the
 * starting sector and the sector after each edge, 15 speed loop ran this tick (else ref, P, I,
 * ff and Iq ref are 0). */
#define ESC_RECORD_START 0x5EU
#define ESC_RECORD_SAMPLES 384U /**< speed-loop ticks recorded */
#define ESC_RECORD_INFO_SAMPLE 1U /**< bank 3 sample index holding the recorder status */
#define ESC_RECORD_READ_CH67 (1U << 18) /**< in the ESC_CAPTURE_READ word */
enum { ESC_RECORD_NONE = 0, ESC_RECORD_ARMED = 1, ESC_RECORD_TRIGGERED = 2,
       ESC_RECORD_DONE = 3, ESC_RECORD_REJECTED = 4 };
/** Recorder channel 7 bits 0-1: what the speed loop did this tick. */
enum { ESC_REC_MODE_OFF = 0, ESC_REC_MODE_RUN = 1, ESC_REC_MODE_HOLD = 2, ESC_REC_MODE_STALL_RAMP = 3 };

#define ESC_COUNTS_PER_AMP 544 /**< MCSDK current counts per ampere */

/** STATUS stop reasons; 6 and 7 (old parking and startup-loop failure) are retired. */
enum { ESC_STOP_NONE = 0, ESC_STOP_COMMAND = 1, ESC_STOP_TIMEOUT = 2,
       ESC_STOP_FAULT = 3, ESC_STOP_ALIGN = 4, ESC_STOP_CAPTURE = 5 };
#define ESC_FLAG_ACTIVE  0x10U /**< streaming speed commands accepted */
#define ESC_FLAG_DERATED 0x20U /**< I2T limit reached, current derated */
#define ESC_FLAG_ALIGN   0x40U /**< forced-angle test running: Hall alignment or capture */
#define ESC_FLAG_ALIGNED 0x80U /**< Hall shift known: built in or measured this boot */
enum { ESC_ALIGN_NONE = 0, ESC_ALIGN_OK = 1, ESC_ALIGN_REVERSED = 2,
       ESC_ALIGN_NO_EDGES = 3, ESC_ALIGN_ABORTED = 4, ESC_ALIGN_RUNNING = 5 };

#endif
