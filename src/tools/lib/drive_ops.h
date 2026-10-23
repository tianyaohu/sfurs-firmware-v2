/** @file drive_ops.h
 * @brief Shared controls, telemetry checks and output for ground drive tools.
 */
#ifndef DRIVE_OPS_H
#define DRIVE_OPS_H
#include "pulse_run.h"
#include "tune_analysis.h"

#define DRIVE_KEYS 4 /**< kp, ki, stall_ramp_ma_per_s, stall_ms: every RAM tunable the tools restore */
#define DRIVE_MAX_MOVES 20 /**< moves per staged plan */

/** @brief Keyboard state: per axis (x, y, yaw) the held sign and its expiry, s; hold_s is the hold time, s. */
typedef struct { double hold_s, expiry_s[3]; int sign[3]; } drive_keys_t;
/** @brief Recorder window summary: samples, trigger index, peak |RPM|. */
typedef struct {
  int samples, trigger, peak_rpm;
} drive_record_summary_t;
/** @brief Nucleo friction feed-forward mailbox words (ik_drive_ff_*, units of friction_ff.c
 *  scaled to integers): forces mN, torques mN*m, boost time ms, 1/Kt mA per N*m, per-wheel trim %. */
typedef struct {
  bool enable;
  int32_t coulomb_x_mn, coulomb_y_mn, viscous_xy_mn_per_mps, coulomb_w_mnm, viscous_w_mnm_per_radps;
  int32_t breakaway_x_mn, breakaway_y_mn, breakaway_w_mnm, breakaway_ms, a_per_nm_milli;
  int32_t scale_pct[IK_WHEELS];
} drive_ff_t;
#define DRIVE_FF_SCALE_NOMINAL_PCT 100 /**< ik_drive_ff_scale_pct boot value (ik_drive.c) */
#define DRIVE_FF_BREAKAWAY_MS_DEFAULT 300 /**< boost time when a breakaway level is given without --ff-breakaway-ms */
#define DRIVE_FF_BREAKAWAY_MS_MAX 2000    /**< longest boost accepted, ms (the ESC stall rest comes after ~1 s anyway) */
/** @brief Per-wheel values of DRIVE_SETTING_KEYS, in each key's CONFIG unit, plus the
 *  Nucleo feed-forward model the stage runs with (off unless a tool sets it). */
typedef struct { int value[IK_WHEELS][DRIVE_KEYS]; drive_ff_t ff; } drive_settings_t;
extern const char *const DRIVE_SETTING_KEYS[DRIVE_KEYS];
/** @brief One staged run: telemetry, its analysis, fault names and lowest Vbus (V). */
typedef struct {
  pulse_result_t pulse;
  tune_log_entry *analysis;
  tune_response_result response;
  char faults[1024];
  double vbus_min_v;
} drive_stage_t;

/** @brief Hold the latest direction per axis until expiry; SPACE clears all.
 *  @param keys  key state
 *  @param key   character read (a/d x, w/s y, q/e yaw, ' ' clears; others ignored)
 *  @param now_s CLOCK_MONOTONIC time, s */
void drive_key_press(drive_keys_t *keys, int key, double now_s);
/** @brief Combined body twist in mm/s and mrad/s, with expired axes released.
 *  @param keys      key state
 *  @param now_s     CLOCK_MONOTONIC time, s
 *  @param speed_mps translation speed per held key, m/s
 *  @param yaw_radps yaw rate per held key, rad/s
 *  @param out       out: vx, vy (mm/s), omega (mrad/s) */
void drive_key_vector(drive_keys_t *keys, double now_s, double speed_mps, double yaw_radps, int32_t out[3]);
/** @brief Summarize one recorder window: sample count, trigger index, peak |RPM|.
 *  @param rows    unpacked recorder samples
 *  @param n       sample count
 *  @param trigger trigger sample index */
drive_record_summary_t drive_record_summary(const ik_record_unpacked_t *rows, int n, int trigger);
/** @brief Compare two four-wheel snapshots for live IDLE, fault-free replies and Vbus.
 *  @param before     telemetry rows read first
 *  @param rows       telemetry rows read later (a live row has a newer status count)
 *  @param min_vbus_v lowest acceptable bus voltage, V
 *  @param problems   out: one line per failing wheel
 *  @param capacity   size of problems, bytes
 *  @return number of failing wheels */
int drive_preflight_rows(const ik_row_t before[4], const ik_row_t rows[4], double min_vbus_v,
                         char *problems, size_t capacity);
/** @brief Read snapshots 0.3 s apart and refuse unsafe ground drive.
 *  @param link       open mailbox link
 *  @param min_vbus_v lowest acceptable bus voltage, V
 *  @return 0 safe, -1 refused or link error (link->err set) */
int drive_preflight(ik_link_t *link, double min_vbus_v);
/** @brief Initialize every ground-tool tunable (DRIVE_SETTING_KEYS) from firmware defaults;
 *  feed-forward off (drive_ff_off).
 *  @param settings out: per-wheel values
 *  @return 0, or -1 when a default is missing */
int drive_settings_defaults(drive_settings_t *settings);
/** @brief Override a named tunable on all four wheels, validating CONFIG range.
 *  @param settings per-wheel values
 *  @param key      one of DRIVE_SETTING_KEYS
 *  @param value    new value in the key's CONFIG unit (int16 range)
 *  @return 0, or -1 for an unknown key or out-of-range value */
int drive_setting_set(drive_settings_t *settings, const char *key, int value);
/** @brief Analyze a pulse result, retaining per-wheel faults and minimum Vbus.
 *  @param stage stage whose pulse result is analysed in place
 *  @return a TUNE_* status */
int drive_stage_analyze(drive_stage_t *stage);
/** @brief Run through pulse_run, log monotonic timestamps, restore every setting.
 *  After preflight it writes settings->ff (drive_ff_write, so it must be stopped), and
 *  afterwards always switches feed-forward off again (drive_ff_clear).
 *  @param link        open mailbox link
 *  @param directory   output directory
 *  @param name        stage name; the log is <directory>/<name>.jsonl
 *  @param plan        moves
 *  @param count       number of moves
 *  @param duration_s  total run time, s
 *  @param settings    per-wheel tunables to send first
 *  @param accel_rpmps wheel acceleration limit, RPM/s (0 = Nucleo default)
 *  @param out         out: result; free with drive_stage_free
 *  @return 0, or -1 with link->err set */
int drive_stage_run(ik_link_t *link, const char *directory, const char *name,
                    const esc_move_t *plan, size_t count, double duration_s,
                    const drive_settings_t *settings, int accel_rpmps, drive_stage_t *out);
/** @brief Free the stage's pulse and analysis samples.
 *  @param stage stage to clear */
void drive_stage_free(drive_stage_t *stage);
/** @brief Write stage fields including absolute and relative move windows (seconds).
 *  @param f        output
 *  @param name     stage name ("drive" writes wheel 0's settings only)
 *  @param stage    analysed stage
 *  @param settings tunables used
 *  @param plan     moves
 *  @param count    number of moves */
void drive_stage_json(FILE *f, const char *name, const drive_stage_t *stage,
                       const drive_settings_t *settings, const esc_move_t *plan, size_t count);
/** @brief Write telemetry response with Python-compatible null missing metrics.
 *  @param f        output
 *  @param response start metrics (s, RPM ratios) */
void drive_response_json(FILE *f, const tune_response_result *response);
/** @brief Print per-wheel start and overshoot metrics.
 *  @param response start metrics (s, RPM ratios) */
void drive_response_show(const tune_response_result *response);
/** @brief Write a JSON string with control-character escaping.
 *  @param f    output
 *  @param text string, or NULL for null */
void drive_json_string(FILE *f, const char *text);
/** @brief Write a finite JSON number or null.
 *  @param f     output
 *  @param value number; NaN or infinity writes null */
void drive_json_number(FILE *f, double value);
/** @brief Write all tunables, for one wheel or an array of four wheels; each object also
 *  carries the feed-forward model as "ff" (drive_ff_json).
 *  @param f        output
 *  @param settings per-wheel values
 *  @param wheel    wheel index 0..3, or < 0 for all four */
void drive_settings_json(FILE *f, const drive_settings_t *settings, int wheel);
/** @brief Write JSONL pulse telemetry with a CLOCK_MONOTONIC origin on every row.
 *  @param path   output file
 *  @param result pulse result
 *  @return 0 or -1 */
int drive_log_write(const char *path, const pulse_result_t *result);
/** @brief Create an output directory and its parents.
 *  @param path directory
 *  @return 0 or -1 */
int drive_mkdir(const char *path);
/** @brief Join an output path, rejecting truncation.
 *  @param out       out: "<directory>/<name>"
 *  @param capacity  size of out, bytes
 *  @param directory directory part
 *  @param name      file name
 *  @return 0, or -1 when it does not fit */
int drive_path(char *out, size_t capacity, const char *directory, const char *name);
/** @brief Median of a copied array; NaN on empty input or allocation failure.
 *  @param values input (not modified)
 *  @param count  number of values */
double drive_median(const double *values, size_t count);
/** @brief Write the local date and time as a JSON string.
 *  @param f output */
void drive_date_json(FILE *f);

/** @brief Value type of a CLI option. */
typedef enum { DRIVE_BOOL, DRIVE_INT, DRIVE_DOUBLE, DRIVE_STRING } drive_option_type;
/** @brief One long CLI option: name, type, destination, and whether it was given. */
typedef struct {
  const char *name;
  drive_option_type type;
  void *value;
  bool seen;
} drive_option;
#define DRIVE_OPT(n, t, v) {n, DRIVE_##t, v, false}
/** @brief Parse long CLI options; 1 means help, -1 means invalid input.
 *  @param argc    argument count
 *  @param argv    arguments
 *  @param options option table; values written through their pointers, seen set
 *  @param count   entries in options */
int drive_parse(int argc, char **argv, drive_option *options, size_t count);
/** @brief Save parsed arguments as string values, matching the Python result schema.
 *  @param f       output
 *  @param options parsed option table
 *  @param count   entries in options */
void drive_args_json(FILE *f, const drive_option *options, size_t count);

/** @brief Feed-forward CLI arguments in SI units, before conversion to mailbox words. */
typedef struct {
  double coulomb_x_n;            /**< x (strafe) Coulomb force, N */
  double coulomb_y_n;            /**< y (forward) Coulomb force, N */
  double viscous_xy_n_per_mps;   /**< translation viscous force, N per m/s */
  double coulomb_w_nm;           /**< yaw Coulomb torque, N*m */
  double viscous_w_nm_per_radps; /**< yaw viscous torque, N*m per rad/s */
  double breakaway_x_n;          /**< x force until the robot breaks free, N */
  double breakaway_y_n;          /**< y force until the robot breaks free, N */
  double breakaway_w_nm;         /**< yaw torque until the robot breaks free, N*m */
  int breakaway_ms;              /**< longest boost per stuck episode, ms */
  double a_per_nm;               /**< motor current per motor torque (1/Kt), A per N*m */
  const char *scale_pct;         /**< "FR,BR,BL,FL" per-wheel trim, integer % */
} drive_ff_args_t;
/** @brief The feed-forward options, for a tool's drive_option table. */
#define DRIVE_FF_OPTIONS(a)                                                               \
  DRIVE_OPT("ff-coulomb-x-n", DOUBLE, &(a)->coulomb_x_n),                                 \
  DRIVE_OPT("ff-coulomb-y-n", DOUBLE, &(a)->coulomb_y_n),                                 \
  DRIVE_OPT("ff-viscous-xy-n-per-mps", DOUBLE, &(a)->viscous_xy_n_per_mps),               \
  DRIVE_OPT("ff-coulomb-w-nm", DOUBLE, &(a)->coulomb_w_nm),                               \
  DRIVE_OPT("ff-viscous-w-nm-per-radps", DOUBLE, &(a)->viscous_w_nm_per_radps),           \
  DRIVE_OPT("ff-breakaway-x-n", DOUBLE, &(a)->breakaway_x_n),                             \
  DRIVE_OPT("ff-breakaway-y-n", DOUBLE, &(a)->breakaway_y_n),                             \
  DRIVE_OPT("ff-breakaway-w-nm", DOUBLE, &(a)->breakaway_w_nm),                           \
  DRIVE_OPT("ff-breakaway-ms", INT, &(a)->breakaway_ms),                                  \
  DRIVE_OPT("ff-a-per-nm", DOUBLE, &(a)->a_per_nm), DRIVE_OPT("ff-scale", STRING, &(a)->scale_pct)
/** @brief Usage text for DRIVE_FF_OPTIONS. */
#define DRIVE_FF_USAGE                                                                      \
  "  [--ff-coulomb-x-n N --ff-coulomb-y-n N --ff-viscous-xy-n-per-mps N --ff-coulomb-w-nm NM\n" \
  "   --ff-viscous-w-nm-per-radps NM --ff-breakaway-x-n N --ff-breakaway-y-n N --ff-breakaway-w-nm NM\n" \
  "   --ff-breakaway-ms 300 --ff-a-per-nm A_PER_NM --ff-scale FR,BR,BL,FL]  Nucleo friction feed-forward\n" \
  "   (SI units, scale %; breakaway levels apply until two wheels turn, at most --ff-breakaway-ms; any\n" \
  "   option turns it on and needs --ff-a-per-nm; it is switched off again on exit)"

/** @brief Feed-forward off: every term zero, trims at their boot value.
 *  @param ff out: mailbox words */
void drive_ff_off(drive_ff_t *ff);
/** @brief Convert the parsed DRIVE_FF_OPTIONS of a table to mailbox words.
 *  Any ff option given enables the model; then --ff-a-per-nm (> 0) is required. Forces and
 *  torques must be >= 0 and fit the int32 milli-unit words; trims are integers 1..200 %
 *  (the firmware model switches off entirely if any trim is <= 0). Nothing given: drive_ff_off.
 *  @param options parsed option table that contains DRIVE_FF_OPTIONS
 *  @param count   entries in options
 *  @param ff      out: mailbox words
 *  @param err     out: reason on failure
 *  @param errlen  size of err, bytes
 *  @return 0, or -1 for an invalid value */
int drive_ff_from_options(const drive_option *options, size_t count, drive_ff_t *ff, char *err, size_t errlen);
/** @brief Write the feed-forward words while the drive is stopped: enable off first, then
 *  the terms and trims, then the enable flag, so the Nucleo never runs a half-written model.
 *  @param link open mailbox link
 *  @param ff   mailbox words
 *  @return 0, or -1 (link->err set) when not stopped or on a link error */
int drive_ff_write(ik_link_t *link, const drive_ff_t *ff);
/** @brief Switch feed-forward off (enable first) and restore every word to drive_ff_off.
 *  Allowed in any drive state, since it only removes feed-forward.
 *  @param link open mailbox link
 *  @return 0, or -1 with link->err set */
int drive_ff_clear(ik_link_t *link);
/** @brief Write the model as JSON in SI units (N, N*m, A per N*m) and trims in %.
 *  @param f  output
 *  @param ff mailbox words */
void drive_ff_json(FILE *f, const drive_ff_t *ff);
#endif
