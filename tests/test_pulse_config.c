/* Real pulse_run: every override and every compiled-default restore uses CONFIG ack. */
#include "check.h"
#include "pulse_run.h"
#include "esc_defaults.h"
#include <string.h>

static unsigned calls, count, fail_call, restore_fail;
static uint32_t statuses;
static bool restored[4][4], speed_mode;
static const char *const keys[] = {"kp", "ki", "stall_ramp_ma_per_s", "stall_ms"};
static const uint16_t ids[] = {MOTOR_CFG_SPEED_KP, MOTOR_CFG_SPEED_KI, MOTOR_CFG_STALL_RAMP_MA_PER_S, MOTOR_CFG_STALL_MS};
static const int values[] = {1234, 234, 12000, 600};

int __wrap_ik_config_exchange(ik_link_t *l, int w, int op, uint16_t key, int value,
                              double timeout, struct motor_config_reply *reply)
{
  CHECK_EQ(op, MOTOR_CONFIG_SET); CHECK(timeout > 0); CHECK(w >= 0 && w < 4);
  unsigned k = 0; while (k < 4 && key != ids[k]) k++; CHECK(k < 4);
  int def; CHECK_EQ(esc_default(keys[k], &def), 0);
  calls++;
  if (calls > (fail_call <= 16 ? fail_call : 16)) { CHECK_EQ(value, def); restored[w][k] = true; }
  else CHECK_EQ(value, values[k]);
  bool fail = calls == fail_call || calls == restore_fail;
  *reply = (struct motor_config_reply){.key = key, .status = fail ? MOTOR_CFG_BAD_VALUE : 0};
  if (fail) { strcpy(l->err, "CONFIG BAD_VALUE"); return -1; }
  count++;
  return 0;
}
int __wrap_ik_set_mode(ik_link_t *l, unsigned mode) { (void)l; CHECK_EQ(mode, IK_MODE_SPEED); speed_mode = true; return 0; }
int __wrap_ik_read(ik_link_t *l, ik_sym_t sym, uint32_t *v)
{ (void)l; CHECK(sym == IK_STATE || sym == IK_FAULT); *v = 0; return 0; }
int __wrap_ik_write(ik_link_t *l, ik_sym_t sym, uint32_t v)
{ (void)l; (void)v; CHECK(sym == IK_ENABLE || sym == IK_ACCEL_RPMPS); return 0; }
int __wrap_ik_check_fault(ik_link_t *l, uint32_t fault) { (void)l; CHECK_EQ(fault, 0); return 0; }
int __wrap_ik_rows(ik_link_t *l, ik_row_t r[4])
{
  (void)l;
  statuses++;
  for (int w = 0; w < 4; w++) r[w] = (ik_row_t){.valid = true, .mc_state = "IDLE", .status_count = statuses};
  return 0;
}

static void run(unsigned fail, unsigned restore)
{
  calls = count = statuses = 0; fail_call = fail; restore_fail = restore; speed_mode = false;
  memset(restored, 0, sizeof restored);
  pulse_settings_t settings = {.n_keys = 4};
  for (int k = 0; k < 4; k++) {
    settings.keys[k] = keys[k];
    for (int w = 0; w < 4; w++) settings.value[w][k] = values[k];
  }
  ik_link_t l = {0}; pulse_result_t result;
  CHECK_EQ(pulse_run(&l, NULL, 0, 0, &settings, 0, &result), fail <= 16 || restore ? -1 : 0);
  CHECK(speed_mode);
  for (int w = 0; w < 4; w++) for (int k = 0; k < 4; k++) CHECK(restored[w][k]);
  if (fail <= 16 || restore) CHECK(strstr(l.err, "BAD_VALUE"));
  CHECK_EQ(calls, fail <= 16 ? fail + 16 : 32);
  pulse_result_free(&result);
}

int main(void)
{
  run(999, 0); run(5, 0); run(999, 20); run(5, 20);
  puts("pulse CONFIG: 4 test cases passed (all four wheels restored)");
  return 0;
}
