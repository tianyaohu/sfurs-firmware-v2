#include "ik_link.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "can_datatypes.h"
#include "elfsym.h"
#include "motor_protocol.h"

const char *const IK_WHEEL_POS[IK_WHEELS] = {"FR", "BR", "BL", "FL"};
const char *const IK_WHEEL_CORNER[IK_WHEELS] = {"NE", "SE", "SW", "NW"};
const uint32_t IK_WHEEL_SELECT[IK_WHEELS] = {MOTOR_NE_CAN_ID, MOTOR_SE_CAN_ID, MOTOR_SW_CAN_ID, MOTOR_NW_CAN_ID};

static const char *const SYM_NAMES[IK_SYM_COUNT] = {
    "abi", "vx_mmps", "vy_mmps", "omega_mradps", "enable", "heartbeat", "accel_rpmps", "raw_id",
    "raw_data", "raw_seq", "state", "raw_done", "tx_count", "tx_fail", "bad_input", "telem_now",
    "target_rpm", "rpm", "telem", "capture", "raw_data_hi", "raw_len", "clear_seq", "clear_done",
    "fault", "armed", "link", "ff_enable", "ff_coulomb_xy_mn", "ff_viscous_xy_mn_per_mps",
    "ff_coulomb_w_mnm", "ff_viscous_w_mnm_per_radps", "ff_a_per_nm_milli", "ff_scale_pct", "ff_ma"};

static const struct { const char *name; int number; } CONFIG_KEYS[] = {
    {"kp", ESC_CFG_SPEED_KP}, {"ki", ESC_CFG_SPEED_KI}, {"shift", ESC_CFG_HALL_SHIFT_DEG},
    {"iqmax_ma", ESC_CFG_IQ_MAX_MA}, {"current_kp", ESC_CFG_CURRENT_KP},
    {"current_ki", ESC_CFG_CURRENT_KI}, {"ff_static_ma", ESC_CFG_FF_STATIC_MA},
    {"ff_kinetic_ma", ESC_CFG_FF_KINETIC_MA},
    /* startup speed loop (drive_speed_ctrl.h) */
    {"dsc_enable", ESC_CFG_DSC_ENABLE}, {"dsc_ff_run_ma", ESC_CFG_DSC_FF_RUN_MA},
    {"dsc_ff_per_krpm_ma", ESC_CFG_DSC_FF_PER_KRPM_MA}, {"dsc_start_from_ma", ESC_CFG_DSC_START_FROM_MA},
    {"dsc_start_cap_ma", ESC_CFG_DSC_START_CAP_MA}, {"dsc_start_slope_ma", ESC_CFG_DSC_START_SLOPE_MA},
    {"dsc_start_hold_ms", ESC_CFG_DSC_START_HOLD_MS}, {"dsc_pcap_ma", ESC_CFG_DSC_PCAP_MA},
    {"dsc_pcap_ms", ESC_CFG_DSC_PCAP_MS}, {"dsc_pcap_updates", ESC_CFG_DSC_PCAP_UPDATES},
    {"dsc_pcap_fallback_ms", ESC_CFG_DSC_PCAP_FALLBACK_MS}, {"speed_rev_filter", ESC_CFG_SPEED_REV_FILTER},
    {"dsc_hold_decay_ma", ESC_CFG_DSC_HOLD_DECAY_MA}, {"dsc_watch_ms", ESC_CFG_DSC_WATCH_MS},
    {"dsc_preload_pct", ESC_CFG_DSC_PRELOAD_PCT}, {"speed_edge_est", ESC_CFG_SPEED_EDGE_EST}};

static const char *const STOP_REASONS[] = {"-", "stop", "timeout", "fault", "align", "capture", "parked"};
static const char *const ALIGN_RESULTS[] = {"-", "ok", "REVERSED", "NO_EDGES", "aborted", "running"};
static const char *const DSC_STATES[] = {"stopped", "starting", "running", "failed"};
static const char *const DSC_REASONS[] = {"none", "armed", "edge", "stop", "reverse", "no_edge",
                                          "no_capture", "inactive", "restick", "moving"};
static const struct { uint32_t bit; const char *name; } FAULT_BITS[] = {
    {0x01, "duration"}, {0x02, "overvolt"}, {0x04, "undervolt"}, {0x08, "overtemp"},
    {0x10, "startup"}, {0x20, "speed_fdbk"}, {0x40, "overcurrent"}, {0x80, "sw_error"}};
static const struct { uint32_t code; const char *name; } MC_STATES[] = {
    {0, "IDLE"}, {4, "START"}, {6, "RUN"}, {8, "STOP"}, {10, "FAULT_NOW"}, {11, "FAULT_OVER"},
    {12, "WAIT_STOP"}, {16, "CHARGE_BOOT_CAP"}, {17, "OFFSET_CALIB"}};

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static double now_s(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static void sleep_s(double s) { nanosleep(&(struct timespec){(time_t)s, (long)((s - (time_t)s) * 1e9)}, NULL); }

static int fail(ik_link_t *l, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static int fail(ik_link_t *l, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(l->err, sizeof l->err, fmt, ap);
  va_end(ap);
  return -1;
}

int ik_resolve(const char *elf, uint32_t addr[IK_SYM_COUNT], char *err, size_t errlen)
{
  FILE *f = fopen(elf, "rb");
  if (!f) {
    snprintf(err, errlen, "IK ELF not found: %s", elf);
    return -1;
  }
  fclose(f);
  for (int i = 0; i < IK_SYM_COUNT; i++) {
    char name[64];
    snprintf(name, sizeof name, "ik_drive_%s", SYM_NAMES[i]);
    if (elfsym_lookup(elf, name, &addr[i], NULL)) {
      snprintf(err, errlen, "Not a streaming IK ELF; missing %s", name);
      return -1;
    }
  }
  return 0;
}

int ik_link_open(ik_link_t *l, const char *serial, const char *elf)
{
  memset(l, 0, sizeof *l);
  if (ik_resolve(elf, l->addr, l->err, sizeof l->err)) return -1;
  char err[256];
  l->ocd = ocd_open(NULL, serial, "stm32h755", err, sizeof err);
  if (!l->ocd) return fail(l, "%s", err);
  uint32_t abi;
  if (ik_read(l, IK_HEARTBEAT, &l->heartbeat) || ik_read(l, IK_ABI_SYM, &abi)) {
    ik_link_close(l);
    return fail(l, "%s", "read failed");
  }
  if (abi != IK_ABI) {
    ik_link_close(l);
    return fail(l, "IK ABI 0x%08x != 0x%08x: flash %s first", abi, IK_ABI, elf);
  }
  return 0;
}

void ik_link_close(ik_link_t *l)
{
  ocd_close(l->ocd);
  l->ocd = NULL;
}

int ik_read(ik_link_t *l, ik_sym_t sym, uint32_t *value)
{
  if (ocd_read32(l->ocd, l->addr[sym], value)) return fail(l, "%s", ocd_error(l->ocd));
  return 0;
}

int ik_write(ik_link_t *l, ik_sym_t sym, uint32_t value)
{
  if (ocd_write32(l->ocd, l->addr[sym], value)) return fail(l, "%s", ocd_error(l->ocd));
  return 0;
}

int ik_drive(ik_link_t *l, int32_t vx, int32_t vy, int32_t omega, bool enable)
{
  l->heartbeat++;
  if (ik_write(l, IK_VX_MMPS, (uint32_t)vx) || ik_write(l, IK_VY_MMPS, (uint32_t)vy) ||
      ik_write(l, IK_OMEGA_MRADPS, (uint32_t)omega) || ik_write(l, IK_ENABLE, enable ? 1 : 0) ||
      ik_write(l, IK_HEARTBEAT, l->heartbeat))
    return -1;
  return 0;
}

int ik_status(ik_link_t *l, ik_status_t *s)
{
  uint32_t w[IK_WHEELS];
  if (ik_read(l, IK_STATE, &s->state) || ik_read(l, IK_TX_COUNT, &s->tx_count) ||
      ik_read(l, IK_TX_FAIL, &s->tx_fail) || ik_read(l, IK_BAD_INPUT, &s->bad_input) ||
      ik_read(l, IK_RAW_DONE, &s->raw_done) || ik_read(l, IK_TELEM_NOW, &s->telem_now) ||
      ik_read(l, IK_FAULT, &s->fault) || ik_read(l, IK_ARMED, &s->armed))
    return -1;
  if (ocd_read(l->ocd, l->addr[IK_TARGET_RPM], 32, IK_WHEELS, w)) return fail(l, "%s", ocd_error(l->ocd));
  for (int i = 0; i < IK_WHEELS; i++) s->target_rpm[i] = (int32_t)w[i];
  if (ocd_read(l->ocd, l->addr[IK_RPM], 32, IK_WHEELS, w)) return fail(l, "%s", ocd_error(l->ocd));
  for (int i = 0; i < IK_WHEELS; i++) s->rpm[i] = (int32_t)w[i];
  return 0;
}

void ik_decode_row(const uint32_t w[IK_ROW_WORDS], ik_row_t *r)
{
  memset(r, 0, sizeof *r);
  r->valid = true;
  r->generation = w[0];
  r->rpm = (int32_t)w[1];
  r->iq_counts = (int32_t)w[2];
  r->iq_ref_counts = (int32_t)w[3];
  r->state = w[4];
  r->flags = w[5];
  r->status_tick = w[6];
  r->status_count = w[7];
  r->vbus_centivolts = w[8];
  r->faults = w[9];
  r->hall_shift_deg = (int32_t)w[10];
  r->align_result = w[11];
  r->i2t_percent = w[12];
  r->diag_tick = w[13];
  r->diag_count = w[14];
  r->app_crc = w[15];
  r->app_size = w[16];
  r->ident_tick = w[17];
  r->ident_count = w[18];
  r->iq_a = (double)r->iq_counts / ESC_COUNTS_PER_AMP;
  r->iq_ref_a = (double)r->iq_ref_counts / ESC_COUNTS_PER_AMP;
  r->vbus_v = r->vbus_centivolts / 100.0;
  uint32_t stop = r->flags & 0x0F;
  static char unknown[2][16];
  r->stop_reason = stop < ARRAY_LEN(STOP_REASONS) ? STOP_REASONS[stop] : (snprintf(unknown[0], 16, "%u", stop), unknown[0]);
  r->active = r->flags & ESC_FLAG_ACTIVE;
  r->derated = r->flags & ESC_FLAG_DERATED;
  r->aligning = r->flags & ESC_FLAG_ALIGN;
  r->aligned = r->flags & ESC_FLAG_ALIGNED;
  r->mc_state = NULL;
  for (size_t i = 0; i < ARRAY_LEN(MC_STATES); i++)
    if (MC_STATES[i].code == r->state) r->mc_state = MC_STATES[i].name;
  if (!r->mc_state) r->mc_state = (snprintf(unknown[1], 16, "%u", r->state), unknown[1]);
  r->align = r->align_result < ARRAY_LEN(ALIGN_RESULTS) ? ALIGN_RESULTS[r->align_result] : "?";
  r->fault_names[0] = '\0';
  for (size_t i = 0; i < ARRAY_LEN(FAULT_BITS); i++)
    if (r->faults & FAULT_BITS[i].bit)
      snprintf(r->fault_names + strlen(r->fault_names), sizeof r->fault_names - strlen(r->fault_names), "%s%s",
               r->fault_names[0] ? " " : "", FAULT_BITS[i].name);
}

int ik_rows(ik_link_t *l, ik_row_t rows[IK_WHEELS])
{
  for (int i = 0; i < IK_WHEELS; i++) {
    uint32_t base = l->addr[IK_TELEM] + (uint32_t)i * 4 * IK_ROW_WORDS;
    rows[i].valid = false;
    for (int attempt = 0; attempt < 3; attempt++) { /* the Nucleo bumps generation while writing a row */
      uint32_t words[IK_ROW_WORDS], again;
      if (ocd_read(l->ocd, base, 32, IK_ROW_WORDS, words) || ocd_read32(l->ocd, base, &again))
        return fail(l, "%s", ocd_error(l->ocd));
      if (words[0] == again && !(words[0] & 1)) {
        ik_decode_row(words, &rows[i]);
        break;
      }
    }
  }
  return 0;
}

const char *ik_reason_name(uint32_t reason)
{
  static const char *const names[] = {
    "NONE", "BAD_VERSION", "BAD_MODE", "BAD_FLAGS", "RESERVED_NONZERO", "BAD_SELECT",
    "OUT_OF_RANGE", "NOT_ARMED", "BAD_SESSION", "STALE", "SEQ_AMBIGUOUS", "SEQ_JUMP",
    "NOT_ALIGNED", "BUSY", "MODE_CHANGE", "NOT_IDLE", "LIMITED", "STARTING", "FF_UNUSED",
    "CONFLICT", "BAD_LENGTH", "UNSUPPORTED", [MOTOR_R_FAULT_MCSDK] = "FAULT_MCSDK",
    [MOTOR_R_FAULT_STALL] = "FAULT_STALL", [MOTOR_R_FAULT_HALL_LOST] = "FAULT_HALL_LOST",
    [MOTOR_R_TIMEOUT] = "TIMEOUT", [MOTOR_R_LIFECYCLE_OVERFLOW] = "LIFECYCLE_OVERFLOW"
  };
  return reason < ARRAY_LEN(names) && names[reason] ? names[reason] : "UNKNOWN";
}

void ik_decode_link_row(const uint32_t w[IK_LINK_WORDS], ik_link_row_t *r)
{
  static const char *const lifecycle[] = {"DISARMED", "READY", "STARTING", "RUNNING", "HOLD", "FAULTED"};
  static const char *const outcome[] = {"APPLIED", "INHIBITED", "SUPERSEDED", "REJECTED", "DUPLICATE"};
  *r = (ik_link_row_t){.valid = true, .generation = w[0], .lifecycle = w[1], .outcome = w[2],
    .reason = w[3], .session = w[4], .request_seq = w[5], .applied_seq = w[6], .first_edge = w[7],
    .rpm = (int32_t)w[8], .iq_ma = (int32_t)w[9], .reply_count = w[10], .reply_us = w[11],
    .misses = w[12], .rejects = w[13], .start_count = w[14], .start_reply_us = w[15],
    .first_start_reply_us = w[16], .first_start_valid = w[17] != 0, .fault_reason = w[18]};
  r->lifecycle_name = r->lifecycle < ARRAY_LEN(lifecycle) ? lifecycle[r->lifecycle] :
                      r->lifecycle == IK_LINK_NONE ? "NO_REPLY" : "UNKNOWN";
  r->outcome_name = r->outcome < ARRAY_LEN(outcome) ? outcome[r->outcome] : "UNKNOWN";
  r->reason_name = ik_reason_name(r->reason);
  r->fault_reason_name = ik_reason_name(r->fault_reason);
}

int ik_link_rows(ik_link_t *l, ik_link_row_t rows[IK_WHEELS])
{
  memset(rows, 0, IK_WHEELS * sizeof *rows);
  for (int i = 0; i < IK_WHEELS; i++) {
    uint32_t base = l->addr[IK_LINK] + (uint32_t)i * 4 * IK_LINK_WORDS;
    for (int attempt = 0; attempt < 3; attempt++) {
      uint32_t before, words[IK_LINK_WORDS], after;
      if (ocd_read32(l->ocd, base, &before)) return fail(l, "%s", ocd_error(l->ocd));
      if (before & 1) continue;
      if (ocd_read(l->ocd, base, 32, IK_LINK_WORDS, words) || ocd_read32(l->ocd, base, &after))
        return fail(l, "%s", ocd_error(l->ocd));
      if (before == words[0] && before == after && !(after & 1)) {
        ik_decode_link_row(words, &rows[i]);
        break;
      }
    }
  }
  return 0;
}

int ik_check_fault(ik_link_t *l, uint32_t fault)
{
  ik_link_row_t rows[IK_WHEELS];
  if (ik_link_rows(l, rows)) return -1;
  char reasons[220] = "";
  for (int w = 0; w < IK_WHEELS; w++) {
    const ik_link_row_t *r = &rows[w];
    bool latched = fault & ((1U << w) | (1U << (w + 4)));
    bool faulted = (fault & (1U << w)) || (r->valid && r->lifecycle == MOTOR_LC_FAULTED);
    bool lost = fault & (1U << (w + 4));
    if (!faulted && !lost) continue;
    /* Later replies overwrite reason; fault_reason keeps the cause latched with the bit (0 = LOST). */
    const char *cause = !r->valid ? "unreadable link" :
                        !latched ? r->reason_name :
                        r->fault_reason ? r->fault_reason_name : lost ? "no replies" : r->reason_name;
    size_t used = strlen(reasons);
    snprintf(reasons + used, sizeof reasons - used, "%s%s %s%s%s (%s)", used ? "; " : "",
             IK_WHEEL_POS[w], faulted ? "FAULTED" : "", faulted && lost ? "/" : "",
             lost ? "LOST" : "", cause);
  }
  if (reasons[0] || fault) return fail(l, "drive stopped: %s; esc_tool clear", reasons[0] ? reasons : "unknown fault");
  return 0;
}

int ik_capture_header(ik_link_t *l, ik_capture_header_t *h)
{
  uint32_t w[5];
  if (ocd_read(l->ocd, l->addr[IK_CAPTURE], 32, 5, w)) return fail(l, "%s", ocd_error(l->ocd));
  *h = (ik_capture_header_t){w[0], w[1], w[2], w[3], w[4]};
  return 0;
}

static int send_raw(ik_link_t *l, uint32_t can_id, uint32_t lo, uint32_t hi, uint32_t len, double timeout_s)
{
  uint32_t state, seq, done;
  if (ik_read(l, IK_STATE, &state)) return -1;
  if (state != 0) return fail(l, "drive not stopped; raw frames are refused");
  if (ik_read(l, IK_RAW_SEQ, &seq)) return -1;
  seq++;
  if (ik_write(l, IK_RAW_ID, can_id) || ik_write(l, IK_RAW_DATA, lo) ||
      ik_write(l, IK_RAW_DATA_HI, hi) || ik_write(l, IK_RAW_LEN, len) || ik_write(l, IK_RAW_SEQ, seq)) return -1;
  double deadline = now_s() + timeout_s;
  for (;;) {
    if (ik_read(l, IK_RAW_DONE, &done)) return -1;
    if (done == seq) return 0;
    if (now_s() > deadline) return fail(l, "raw frame 0x%03x not sent", can_id);
    sleep_s(0.005);
  }
}

int ik_send_raw(ik_link_t *l, uint32_t can_id, uint32_t word, double timeout_s)
{
  return send_raw(l, can_id, word, 0, 4, timeout_s);
}

int ik_send_raw8(ik_link_t *l, uint32_t can_id, const uint8_t data[8], double timeout_s)
{
  uint32_t words[2] = {0};
  for (int i = 0; i < 8; i++) words[i / 4] |= (uint32_t)data[i] << (8 * (i % 4));
  return send_raw(l, can_id, words[0], words[1], 8, timeout_s);
}

int ik_capture_start_word(int step_ma, uint32_t *word)
{
  if (step_ma < 0 || step_ma > 6000) return -1; /* IQMAX */
  *word = ((uint32_t)ESC_CAPTURE_START << 24) | (uint32_t)step_ma;
  return 0;
}

int ik_capture_read_word(int bank, int first, int count, uint32_t *word)
{
  if ((bank != 0 && bank != 1 && bank != (int)ESC_CAPTURE_INFO_BANK) || first < 0 || first >= (int)ESC_CAPTURE_SAMPLES ||
      count < 1 || count > (int)IK_CAPTURE_READ_MAX)
    return -1;
  *word = ((uint32_t)ESC_CAPTURE_READ << 24) | ((uint32_t)bank << 16) | ((uint32_t)count << 10) | (uint32_t)first;
  return 0;
}

int ik_record_start_word(int pre, int delay_ms, uint32_t *word)
{
  if (pre < 0 || pre >= (int)ESC_RECORD_SAMPLES || delay_ms < 0 || delay_ms > 4095) return -1;
  *word = ((uint32_t)ESC_RECORD_START << 24) | ((uint32_t)delay_ms << 9) | (uint32_t)pre;
  return 0;
}

int ik_record_read_word(int bank, int first, int count, bool ch67, uint32_t *word)
{
  if ((bank != 0 && bank != 1) || (ch67 && bank != 0) || first < 0 || first >= (int)ESC_RECORD_SAMPLES ||
      count < 1 || count > (int)IK_CAPTURE_READ_MAX)
    return -1;
  *word = ((uint32_t)ESC_CAPTURE_READ << 24) | (ch67 ? ESC_RECORD_READ_CH67 : 0) | ((uint32_t)bank << 16) |
          ((uint32_t)count << 10) | (uint32_t)first;
  return 0;
}

int ik_config_frame(uint32_t selector, const char *key, int value, uint32_t *can_id, uint32_t *word)
{
  if (value < -32768 || value > 32767) return -1;
  for (size_t i = 0; i < ARRAY_LEN(CONFIG_KEYS); i++) {
    if (!strcmp(CONFIG_KEYS[i].name, key)) {
      *can_id = CONFIG_SET | selector;
      *word = (uint32_t)CONFIG_KEYS[i].number | (((uint32_t)value & 0xFFFFU) << 16);
      return 0;
    }
  }
  return -1;
}

size_t ik_config_key_count(void) { return ARRAY_LEN(CONFIG_KEYS); }

const char *ik_config_key_name(size_t i, int *number)
{
  if (i >= ARRAY_LEN(CONFIG_KEYS)) return NULL;
  if (number) *number = CONFIG_KEYS[i].number;
  return CONFIG_KEYS[i].name;
}

void ik_unpack_record(const ik_record_sample_t *s, ik_record_unpacked_t *u)
{
  unsigned p = (uint16_t)s->packed;
  unsigned reason = (p >> 2) & 15;
  u->ref_rpm = s->ref_rpm;
  u->rpm = s->rpm;
  u->p_a = (double)s->p / ESC_COUNTS_PER_AMP;
  u->i_a = (double)s->i / ESC_COUNTS_PER_AMP;
  u->ff_a = (double)s->ff / ESC_COUNTS_PER_AMP;
  u->iq_ref_a = (double)s->iq_ref / ESC_COUNTS_PER_AMP;
  u->iq_a = (double)s->iq / ESC_COUNTS_PER_AMP;
  u->state = DSC_STATES[p & 3];
  u->reason = reason < ARRAY_LEN(DSC_REASONS) ? DSC_REASONS[reason] : "?";
  u->capped = p & 0x40;
  u->dsc = p & 0x80;
  u->edges_mod16 = (int)((p >> 8) & 15);
  u->hall_state = (int)((p >> 12) & 7);
  u->loop = p & 0x8000;
}

/* Wait until the Nucleo has received at least `frames` CAN reply frames. */
static int wait_frames(ik_link_t *l, uint32_t frames, double timeout_s, bool *reached)
{
  double deadline = now_s() + timeout_s;
  ik_capture_header_t h;
  for (;;) {
    if (ik_capture_header(l, &h)) return -1;
    if (h.frames >= frames) {
      *reached = true;
      return 0;
    }
    if (now_s() > deadline) {
      *reached = false;
      return 0;
    }
    sleep_s(0.005);
  }
}

/* Request `count` reply frames with `word`; CAN frames are not retransmitted, so retry. */
static int read_frames(ik_link_t *l, uint32_t selector, uint32_t word, uint32_t count, const char *what)
{
  for (int attempt = 0; attempt < 3; attempt++) {
    ik_capture_header_t h;
    bool ok;
    if (ik_capture_header(l, &h) || ik_send_raw(l, CAPTURE | selector, word, 1.0) ||
        wait_frames(l, h.frames + count, 0.5, &ok))
      return -1;
    if (ok) return 0;
  }
  return fail(l, "%s incomplete", what);
}

static int read_samples(ik_link_t *l, size_t n, int16_t (*out)[6])
{
  uint8_t *raw = malloc(n * 12);
  if (!raw) return fail(l, "out of memory");
  int rc = ocd_read_bytes(l->ocd, l->addr[IK_CAPTURE] + 4 * 5, raw, n * 12);
  if (rc) fail(l, "%s", ocd_error(l->ocd));
  for (size_t i = 0; !rc && i < n; i++)
    for (int c = 0; c < 6; c++) out[i][c] = (int16_t)(raw[12 * i + 2 * c] | (raw[12 * i + 2 * c + 1] << 8));
  free(raw);
  return rc;
}

int ik_capture(ik_link_t *l, uint32_t selector, int step_ma, ik_capture_header_t *info, ik_capture_sample_t *samples)
{
  uint32_t word;
  if (ik_capture_start_word(step_ma, &word)) return fail(l, "step must be 0..6000 mA (IQMAX)");
  if (ik_send_raw(l, CAPTURE | selector, word, 1.0)) return -1;
  double deadline = now_s() + 3.0;
  for (;;) {
    sleep_s(0.05);
    ik_capture_header_t before;
    bool ok;
    if (ik_capture_header(l, &before) || ik_capture_read_word(ESC_CAPTURE_INFO_BANK, 0, 1, &word) ||
        ik_send_raw(l, CAPTURE | selector, word, 1.0) || wait_frames(l, before.frames + 1, 0.3, &ok))
      return -1;
    if (ok) {
      if (ik_capture_header(l, info)) return -1;
      if (info->status != ESC_CAPTURE_RUNNING) break;
    }
    if (now_s() > deadline) return fail(l, "capture did not finish");
  }
  if (info->status != ESC_CAPTURE_DONE) return fail(l, "capture status %u (2 = done)", info->status);
  for (int bank = 0; bank < 2; bank++) {
    for (int first = 0; first < (int)ESC_CAPTURE_SAMPLES; first += (int)IK_CAPTURE_READ_MAX) {
      if (ik_capture_read_word(bank, first, (int)IK_CAPTURE_READ_MAX, &word) ||
          read_frames(l, selector, word, IK_CAPTURE_READ_MAX, "capture readout"))
        return -1;
    }
  }
  int16_t (*raw)[6] = malloc(ESC_CAPTURE_SAMPLES * sizeof *raw);
  if (!raw) return fail(l, "out of memory");
  int rc = read_samples(l, ESC_CAPTURE_SAMPLES, raw);
  for (size_t i = 0; !rc && i < ESC_CAPTURE_SAMPLES; i++)
    samples[i] = (ik_capture_sample_t){raw[i][0], raw[i][1], raw[i][2], raw[i][3], raw[i][4], raw[i][5]};
  free(raw);
  return rc;
}

int ik_record_start(ik_link_t *l, uint32_t selector, int pre, int delay_ms)
{
  uint32_t word;
  if (ik_record_start_word(pre, delay_ms, &word)) return fail(l, "pre 0..%u, delay 0..4095 ms", ESC_RECORD_SAMPLES - 1);
  return ik_send_raw(l, CAPTURE | selector, word, 1.0);
}

int ik_record_read(ik_link_t *l, uint32_t selector, ik_record_sample_t *samples, int *count, int *trigger)
{
  /* The Nucleo stores the info frame in the capture header fields. */
  uint32_t word = ((uint32_t)ESC_CAPTURE_READ << 24) | ((uint32_t)ESC_CAPTURE_INFO_BANK << 16) | (1U << 10) |
                  ESC_RECORD_INFO_SAMPLE;
  ik_capture_header_t before, h;
  bool ok;
  if (ik_capture_header(l, &before) || ik_send_raw(l, CAPTURE | selector, word, 1.0) ||
      wait_frames(l, before.frames + 1, 0.3, &ok))
    return -1;
  if (!ok) return fail(l, "recorder info: no reply");
  if (ik_capture_header(l, &h)) return -1;
  if (h.status != ESC_RECORD_DONE) return fail(l, "recorder status %u (3 = done)", h.status);
  int n = (int)h.vbus_centivolts; /* sample count rides in this field */
  if (n < 1 || n > (int)ESC_RECORD_SAMPLES) return fail(l, "recorder reports %d samples", n);
  *count = n;
  *trigger = (int)h.samples;
  int16_t (*raw)[6] = malloc(ESC_RECORD_SAMPLES * sizeof *raw);
  if (!raw) return fail(l, "out of memory");
  int rc = 0;
  for (int pass = 0; !rc && pass < 2; pass++) { /* pass 0: banks 0,1 -> channels 0..5; pass 1: ch67 -> 6,7 */
    for (int bank = 0; !rc && bank < (pass ? 1 : 2); bank++) {
      for (int first = 0; !rc && first < n; first += (int)IK_CAPTURE_READ_MAX) {
        int c = n - first < (int)IK_CAPTURE_READ_MAX ? n - first : (int)IK_CAPTURE_READ_MAX;
        rc = ik_record_read_word(bank, first, c, pass == 1, &word) ||
             read_frames(l, selector, word, (uint32_t)c, "recorder readout");
      }
    }
    if (!rc) rc = read_samples(l, (size_t)n, raw);
    for (int i = 0; !rc && i < n; i++) {
      int16_t *s = (int16_t *)&samples[i];
      if (pass) {
        s[6] = raw[i][0];
        s[7] = raw[i][1];
      } else {
        for (int c = 0; c < 6; c++) s[c] = raw[i][c];
      }
    }
  }
  free(raw);
  return rc;
}
