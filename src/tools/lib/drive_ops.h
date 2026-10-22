/** @file drive_ops.h
 * @brief Shared controls, telemetry checks and output for ground drive tools.
 */
#ifndef DRIVE_OPS_H
#define DRIVE_OPS_H
#include "pulse_run.h"
#include "tune_analysis.h"

#define DRIVE_KEYS 4 /**< kp, ki, stall_ramp_ma_per_s, stall_ms: every RAM tunable the tools restore */
#define DRIVE_MAX_MOVES 20

typedef struct { double hold_s, expiry_s[3]; int sign[3]; } drive_keys_t;
typedef struct {
  int samples, trigger, peak_rpm;
} drive_record_summary_t;
typedef struct { int value[IK_WHEELS][DRIVE_KEYS]; } drive_settings_t;
extern const char *const DRIVE_SETTING_KEYS[DRIVE_KEYS];
typedef struct {
  pulse_result_t pulse;
  tune_log_entry *analysis;
  tune_response_result response;
  char faults[1024];
  double vbus_min_v;
} drive_stage_t;

/** @brief Hold the latest direction per axis until expiry; SPACE clears all. */
void drive_key_press(drive_keys_t *keys, int key, double now_s);
/** @brief Combined body twist in mm/s and mrad/s, with expired axes released. */
void drive_key_vector(drive_keys_t *keys, double now_s, double speed_mps, double yaw_radps, int32_t out[3]);
/** @brief Summarize one recorder window: sample count, trigger index, peak |RPM|. */
drive_record_summary_t drive_record_summary(const ik_record_unpacked_t *rows, int n, int trigger);
/** @brief Compare two four-wheel snapshots for live IDLE, fault-free replies and Vbus. */
int drive_preflight_rows(const ik_row_t before[4], const ik_row_t rows[4], double min_vbus_v,
                         char *problems, size_t capacity);
/** @brief Read snapshots 0.3 s apart and refuse unsafe ground drive. */
int drive_preflight(ik_link_t *link, double min_vbus_v);
/** @brief Initialize every ground-tool tunable (DRIVE_SETTING_KEYS) from firmware defaults. */
int drive_settings_defaults(drive_settings_t *settings);
/** @brief Override a named tunable on all four wheels, validating CONFIG_SET range. */
int drive_setting_set(drive_settings_t *settings, const char *key, int value);
/** @brief Analyze a pulse result, retaining per-wheel faults and minimum Vbus. */
int drive_stage_analyze(drive_stage_t *stage);
/** @brief Run through pulse_run, log monotonic timestamps, restore every setting. */
int drive_stage_run(ik_link_t *link, const char *directory, const char *name,
                    const esc_move_t *plan, size_t count, double duration_s,
                    const drive_settings_t *settings, int accel_rpmps, drive_stage_t *out);
/** @brief Free the stage's pulse and analysis samples. */
void drive_stage_free(drive_stage_t *stage);
/** @brief Write stage fields including absolute and relative move windows (seconds). */
void drive_stage_json(FILE *f, const char *name, const drive_stage_t *stage,
                       const drive_settings_t *settings, const esc_move_t *plan, size_t count);
/** @brief Write telemetry response with Python-compatible null missing metrics. */
void drive_response_json(FILE *f, const tune_response_result *response);
/** @brief Print per-wheel start and overshoot metrics. */
void drive_response_show(const tune_response_result *response);
/** @brief Write a JSON string with control-character escaping. */
void drive_json_string(FILE *f, const char *text);
/** @brief Write a finite JSON number or null. */
void drive_json_number(FILE *f, double value);
/** @brief Write all tunables, for one wheel or an array of four wheels. */
void drive_settings_json(FILE *f, const drive_settings_t *settings, int wheel);
/** @brief Write JSONL pulse telemetry with a CLOCK_MONOTONIC origin on every row. */
int drive_log_write(const char *path, const pulse_result_t *result);
/** @brief Create an output directory and its parents. */
int drive_mkdir(const char *path);
/** @brief Join an output path, rejecting truncation. */
int drive_path(char *out, size_t capacity, const char *directory, const char *name);
/** @brief Median of a copied array; NaN on empty input or allocation failure. */
double drive_median(const double *values, size_t count);
/** @brief Write the local date and time as a JSON string. */
void drive_date_json(FILE *f);

typedef enum { DRIVE_BOOL, DRIVE_INT, DRIVE_DOUBLE, DRIVE_STRING } drive_option_type;
typedef struct {
  const char *name;
  drive_option_type type;
  void *value;
  bool seen;
} drive_option;
#define DRIVE_OPT(n, t, v) {n, DRIVE_##t, v, false}
/** @brief Parse long CLI options; 1 means help, -1 means invalid input. */
int drive_parse(int argc, char **argv, drive_option *options, size_t count);
/** @brief Save parsed arguments as string values, matching the Python result schema. */
void drive_args_json(FILE *f, const drive_option *options, size_t count);
#endif
