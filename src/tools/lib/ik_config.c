/** @file ik_config.c
 * @brief One acknowledged CONFIG exchange through the stopped Nucleo raw mailbox.
 */
#include "ik_link.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static int fail(ik_link_t *l, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static int fail(ik_link_t *l, const char *fmt, ...)
{
  va_list ap;
  va_start(ap, fmt); vsnprintf(l->err, sizeof l->err, fmt, ap); va_end(ap);
  return -1;
}
static double now_s(void)
{
  struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}
static void sleep_s(double s)
{
  nanosleep(&(struct timespec){(time_t)s, (long)((s - (time_t)s) * 1e9)}, NULL);
}

int ik_config_exchange(ik_link_t *l, int wheel, int op, uint16_t key, int value,
                       double timeout_s, struct motor_config_reply *reply)
{
  if (!reply) return fail(l, "CONFIG needs a reply destination");
  memset(reply, 0, sizeof *reply);
  if (wheel < 0 || wheel >= IK_WHEELS || value < INT16_MIN || value > INT16_MAX ||
      (op != MOTOR_CONFIG_SET && op != MOTOR_CONFIG_GET) || !isfinite(timeout_s) || timeout_s <= 0)
    return fail(l, "invalid CONFIG wheel/op/value/timeout");
  const double deadline = now_s() + timeout_s;
  ik_link_row_t before, row;
  if (ik_link_row(l, wheel, &before)) return -1;
  if (!before.valid) return fail(l, "%s CONFIG unreadable baseline", IK_WHEEL_POS[wheel]);
  /* Skip the last observed sequence even after opening a new host connection. */
  if (++l->config_seq == before.cfg_seq) ++l->config_seq;
  const struct motor_config c = {(uint8_t)op, l->config_seq, key, (int16_t)value};
  uint32_t id;
  uint8_t data[8];
  enum motor_reason reason = motor_config_encode(&c, IK_WHEEL_SELECT[wheel], &id, data);
  if (reason) return fail(l, "CONFIG %s", ik_reason_name(reason));
  double remaining = deadline - now_s();
  if (remaining <= 0) return fail(l, "CONFIG deadline expired before send");
  if (ik_send_raw8(l, id, data, remaining)) return -1;
  for (;;) {
    if (ik_link_row(l, wheel, &row)) return -1;
    if (row.valid && row.cfg_count != before.cfg_count && row.cfg_seq == c.seq && row.cfg_key == key) {
      *reply = (struct motor_config_reply){(uint8_t)row.cfg_seq, (uint8_t)row.cfg_status,
                   (uint16_t)row.cfg_key, (int16_t)row.cfg_stored, (int16_t)row.cfg_effective};
      if (reply->status) return fail(l, "%s CONFIG 0x%04x %s (%u): stored=%d effective=%d",
             IK_WHEEL_POS[wheel], key, ik_config_status_name(reply->status), reply->status,
             reply->stored, reply->effective);
      return 0;
    }
    if (now_s() >= deadline) return fail(l, "%s CONFIG 0x%04x seq %u timeout", IK_WHEEL_POS[wheel], key, c.seq);
    sleep_s(0.005);
  }
}

