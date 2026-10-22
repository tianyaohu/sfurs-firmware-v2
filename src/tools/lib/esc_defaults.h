/**
 * @file esc_defaults.h
 * @brief RAM-tunable ESC values as the firmware boots with them (drive_parameters.h),
 *        by CONFIG_SET key name. The tools restore these after a run.
 */
#ifndef ESC_DEFAULTS_H_LOOKUP
#define ESC_DEFAULTS_H_LOOKUP

/**
 * @brief Boot default of a CONFIG_SET key.
 * @param key   key name as in ik_config_frame ("kp", "ki", "stall_ramp_ma_per_s", "stall_ms").
 * @param value receives the default.
 * @return 0 if the key has a default, -1 otherwise.
 */
int esc_default(const char *key, int *value);

#endif
