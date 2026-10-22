/**
 * @file ik_link.h
 * @brief SWD link to the Nucleo streaming IK image (ABI IK_ABI).
 *
 * Resolves the ik_drive_* mailbox and telemetry rows from the flashed ELF,
 * writes setpoints and the heartbeat, sends one-shot raw ESC frames and decodes
 * per-wheel telemetry. CAN ids and keys come from can_datatypes.h, so a
 * protocol change is a compile error here, not silent drift.
 */
#ifndef IK_LINK_H
#define IK_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ocd.h"

#define IK_ABI 0x494B0007U
#define IK_WHEELS 4
#define IK_RECORD_CHANNELS 8
#define IK_CAPTURE_READ_MAX 32U
#define IK_ROW_WORDS 19
#define IK_LINK_WORDS 19
#define IK_LAT_BINS 16
#define IK_LINK_NONE 0xFFU
#define IK_DEFAULT_SN "002200364142500620353451"
#define IK_DEFAULT_ELF "build/drive/sfurs_firmware_v2_CM7.elf"

/** Wheel names in telemetry order with their CAN select bits. */
extern const char *const IK_WHEEL_POS[IK_WHEELS];  /**< "FR","BR","BL","FL" */
extern const char *const IK_WHEEL_CORNER[IK_WHEELS]; /**< "NE","SE","SW","NW" */
extern const uint32_t IK_WHEEL_SELECT[IK_WHEELS];

/** Mailbox symbols (ik_drive_<name> in the ELF). */
typedef enum {
  IK_ABI_SYM, IK_VX_MMPS, IK_VY_MMPS, IK_OMEGA_MRADPS, IK_ENABLE, IK_HEARTBEAT,
  IK_ACCEL_RPMPS, IK_RAW_ID, IK_RAW_DATA, IK_RAW_SEQ, IK_STATE, IK_RAW_DONE,
  IK_TX_COUNT, IK_TX_FAIL, IK_BAD_INPUT, IK_TELEM_NOW, IK_TARGET_RPM, IK_RPM,
  IK_TELEM, IK_CAPTURE, IK_RAW_DATA_HI, IK_RAW_LEN, IK_CLEAR_SEQ, IK_CLEAR_DONE,
  IK_FAULT, IK_ARMED, IK_LINK, IK_FF_ENABLE, IK_FF_COULOMB_XY_MN,
  IK_FF_VISCOUS_XY_MN_PER_MPS, IK_FF_COULOMB_W_MNM, IK_FF_VISCOUS_W_MNM_PER_RADPS,
  IK_FF_A_PER_NM_MILLI, IK_FF_SCALE_PCT, IK_FF_MA, IK_REPLY_LAT_HIST, IK_REPLY_LAT_MAX_US, IK_SYM_COUNT
} ik_sym_t;

typedef struct {
  ocd_t *ocd;
  uint32_t addr[IK_SYM_COUNT];
  uint32_t heartbeat;
  char err[256];
} ik_link_t;

/** One decoded telemetry row (a wheel). valid is false if the read tore three times. */
typedef struct {
  bool valid;
  uint32_t generation;
  int32_t rpm, iq_counts, iq_ref_counts, hall_shift_deg;
  uint32_t state, flags, status_tick, status_count, vbus_centivolts, faults, align_result, i2t_percent,
      diag_tick, diag_count, app_crc, app_size, ident_tick, ident_count;
  double iq_a, iq_ref_a, vbus_v;
  const char *stop_reason, *mc_state, *align;
  bool active, derated, aligning, aligned;
  char fault_names[96]; /**< space-separated, empty if none */
} ik_row_t;

/** Protocol reply snapshot; valid is false after three torn reads. */
typedef struct {
  bool valid;
  uint32_t generation, lifecycle, outcome, reason, session, request_seq, applied_seq, first_edge;
  int32_t rpm, iq_ma; /**< Measured RPM and q-current in mA. */
  uint32_t reply_count, reply_us, misses, rejects, start_count, start_reply_us, first_start_reply_us;
  bool first_start_valid; /**< first_start_reply_us belongs to this drive session */
  uint32_t fault_reason;  /**< reason latched with the wheel's fault bit (0 for LOST) */
  const char *lifecycle_name, *outcome_name, *reason_name, *fault_reason_name;
} ik_link_row_t;

/** Drive state snapshot. */
typedef struct {
  uint32_t state, tx_count, tx_fail, bad_input, raw_done, telem_now, fault, armed;
  int32_t target_rpm[IK_WHEELS], rpm[IK_WHEELS];
} ik_status_t;

typedef struct {
  uint32_t tx_count, tx_fail, fault, can_rx;
  uint32_t hist[IK_WHEELS][IK_LAT_BINS], max_us[IK_WHEELS];
  ik_link_row_t rows[IK_WHEELS];
} ik_counters_t;
/** @brief Read cumulative counters and 250 us latency bins; take snapshots while stopped for stable CONTROL totals.
 * @param can_rx_addr Address of can_rx_frame_count resolved from the same ELF.
 */
int ik_counters(ik_link_t *l, uint32_t can_rx_addr, ik_counters_t *out);

/** Capture header words in Nucleo RAM (updated by every CAN reply). */
typedef struct { uint32_t frames, source, status, vbus_centivolts, samples; } ik_capture_header_t;

typedef struct { int16_t id, iq, vd, vq, id_ref, angle; } ik_capture_sample_t;

/** One speed-loop recorder sample; channel order matches the Python tools. */
typedef struct { int16_t ref_rpm, rpm, p, i, ff, iq_ref, iq, packed; } ik_record_sample_t;

/** Decoded recorder sample, physical units. */
typedef struct {
  int ref_rpm, rpm;
  double p_a, i_a, ff_a, iq_ref_a, iq_a;
  const char *mode;  /**< Loop mode (bits 0-1): "off", "run", "hold", "stall_ramp". */
  bool valid;        /**< Speed estimate valid. */
  bool saturated;    /**< Current request saturated at the limit. */
  bool bleed;        /**< Over-speed bleed applied this tick. */
  bool stall_timer;  /**< Stall timer counting (high current, no Hall progress). */
  bool loop;         /**< Speed loop ran this tick (else ref, P, I, ff, Iq ref are 0). */
  int edges_mod16;
  int hall_state; /**< Raw Hall state 1..6 sampled in the same tick; 0/7 invalid. */
} ik_record_unpacked_t;

/** @brief Resolve every ik_drive_* symbol from the ELF. Returns 0, or -1 with err set. */
int ik_resolve(const char *elf, uint32_t addr[IK_SYM_COUNT], char *err, size_t errlen);

/** @brief Attach (no halt/reset) and check the ABI word. Returns 0, or -1 with l->err set. */
int ik_link_open(ik_link_t *l, const char *serial, const char *elf);
/** @brief Detach; the Nucleo keeps running. */
void ik_link_close(ik_link_t *l);

/** @brief Read/write one mailbox word. Return 0 or -1. */
int ik_read(ik_link_t *l, ik_sym_t sym, uint32_t *value);
int ik_write(ik_link_t *l, ik_sym_t sym, uint32_t value);

/** @brief Publish a body twist (mm/s, mm/s, mrad/s) and bump the heartbeat (call at >= 10 Hz). */
int ik_drive(ik_link_t *l, int32_t vx_mmps, int32_t vy_mmps, int32_t omega_mradps, bool enable);
int ik_status(ik_link_t *l, ik_status_t *s);
/** @brief Four decoded telemetry rows; rows[i].valid is false if torn three times. */
int ik_rows(ik_link_t *l, ik_row_t rows[IK_WHEELS]);
/** @brief Decode IK_ROW_WORDS raw words. */
void ik_decode_row(const uint32_t words[IK_ROW_WORDS], ik_row_t *row);
/** @brief Read four seqlocked protocol rows; invalid rows must not be used.
 * @param l Open mailbox link.
 * @param rows Four snapshots; valid is false after three torn reads.
 * @return 0, or -1 with l->err on a transport error.
 */
int ik_link_rows(ik_link_t *l, ik_link_row_t rows[IK_WHEELS]);
/** @brief Read one seqlocked reply row (wheel 0..3); reply clocks are wrapping microseconds. */
int ik_link_row(ik_link_t *l, int wheel, ik_link_row_t *row);
/** @brief Decode IK_LINK_WORDS mailbox words, including stable protocol names.
 * @param words Stable mailbox words, in ABI 7 order; clocks are wrapping microseconds.
 * @param row Decoded row; rpm is mechanical RPM and iq_ma is q-current in mA.
 */
void ik_decode_link_row(const uint32_t words[IK_LINK_WORDS], ik_link_row_t *row);
/** @brief Protocol reason name, or UNKNOWN for an unrecognized code.
 * @param reason Protocol reason code.
 */
const char *ik_reason_name(uint32_t reason);
/** @brief Refuse latched drive faults or FAULTED links; set l->err with wheel/reason and recovery hint.
 * @param l Open mailbox link.
 * @param fault Snapshot of ik_drive_fault (bits w FAULTED, bits 4+w LOST).
 * @return 0 if healthy, -1 on fault or transport error.
 */
int ik_check_fault(ik_link_t *l, uint32_t fault);
int ik_capture_header(ik_link_t *l, ik_capture_header_t *h);

/** @brief One 4-byte CAN frame through the Nucleo; only accepted while STOPPED. */
int ik_send_raw(ik_link_t *l, uint32_t can_id, uint32_t word, double timeout_s);
/** @brief Send 8 CAN payload bytes while STOPPED.
 * @param l Open mailbox link.
 * @param can_id 11-bit CAN ID.
 * @param data Eight payload bytes from the shared codec for CONTROL frames.
 * @param timeout_s Maximum wait for raw_done, seconds.
 * @return 0 on transmission, -1 with l->err on failure.
 */
int ik_send_raw8(ik_link_t *l, uint32_t can_id, const uint8_t data[8], double timeout_s);

/* Command words (can_datatypes.h). Each returns 0 or -1 on an out-of-range argument. */
int ik_capture_start_word(int step_ma, uint32_t *word);
int ik_capture_read_word(int bank, int first, int count, uint32_t *word);
int ik_record_start_word(int pre, int delay_ms, uint32_t *word);
int ik_record_read_word(int bank, int first, int count, bool ch67, uint32_t *word);
/** @brief CONFIG_SET frame for a named key. Returns 0, or -1 for an unknown key or value outside int16. */
int ik_config_frame(uint32_t selector, const char *key, int value, uint32_t *can_id, uint32_t *word);
/** @brief Number of CONFIG_SET keys and their names (for help text and tests). */
size_t ik_config_key_count(void);
const char *ik_config_key_name(size_t i, int *number);

/** @brief Decode one recorder sample into physical units. */
void ik_unpack_record(const ik_record_sample_t *s, ik_record_unpacked_t *u);

/**
 * @brief Run one current-loop step test on one wheel (IDLE, battery on) and read it back.
 * @param selector wheel select bit.
 * @param step_ma  step current, 0..6000 mA.
 * @param header   receives the capture header (status, Vbus, count).
 * @param samples  ESC_CAPTURE_SAMPLES entries.
 * @return 0, or -1 with l->err set.
 */
int ik_capture(ik_link_t *l, uint32_t selector, int step_ma, ik_capture_header_t *header,
               ik_capture_sample_t *samples);
int ik_record_start(ik_link_t *l, uint32_t selector, int pre, int delay_ms);
/** @brief Read a finished recording: samples gets up to ESC_RECORD_SAMPLES entries; *count and *trigger are set. */
int ik_record_read(ik_link_t *l, uint32_t selector, ik_record_sample_t *samples, int *count, int *trigger);

#endif
