#define _POSIX_C_SOURCE 200809L
#include "esc_ctl.h"
#include "motor_protocol.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static double now_s(void)
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double)t.tv_sec + (double)t.tv_nsec * 1e-9;
}

static void pause_poll(void) { nanosleep(&(struct timespec){0, 2000000}, NULL); }

void esc_ctl_init(esc_ctl_t *c, ik_link_t *l, double timeout_s)
{
  *c = (esc_ctl_t){.link = l, .timeout_s = timeout_s};
}

static int send_control(esc_ctl_t *c, uint32_t select, const struct motor_control *control)
{
  uint32_t id;
  uint8_t data[MOTOR_PROTOCOL_DLC];
  enum motor_reason reason = motor_control_encode(control, select, &id, data);
  if (reason != MOTOR_R_NONE) {
    snprintf(c->link->err, sizeof c->link->err, "CONTROL: %s", ik_reason_name(reason));
    return -1;
  }
  return ik_send_raw8(c->link, id, data, c->timeout_s);
}

int esc_ctl_disable_all(esc_ctl_t *c)
{
  for (int w = 0; w < IK_WHEELS; w++) c->wheel[w].armed = false;
  return send_control(c, MOTOR_CAN_SELECT_MASK, &(struct motor_control){.mode = MOTOR_MODE_DISABLE});
}

static int wheel_error(esc_ctl_t *c, int w, const char *operation, const ik_link_row_t *r)
{
  snprintf(c->link->err, sizeof c->link->err, "%s %s: %s %s %s (session %u)", IK_WHEEL_POS[w], operation,
           r->valid ? r->lifecycle_name : "unreadable link", r->valid ? r->outcome_name : "",
           r->valid ? r->reason_name : "", r->valid ? r->session : 0);
  return -1;
}

static int wait_reply(esc_ctl_t *c, int w, uint32_t before, uint8_t seq, ik_link_row_t *reply)
{
  double deadline = now_s() + c->timeout_s;
  for (;;) {
    ik_link_row_t rows[IK_WHEELS];
    if (ik_link_rows(c->link, rows)) return -1;
    *reply = rows[w];
    if (reply->valid && reply->reply_count != before && reply->request_seq == seq) return 0;
    if (now_s() >= deadline) return wheel_error(c, w, "ARM reply timed out", reply);
    pause_poll();
  }
}

int esc_ctl_arm(esc_ctl_t *c, int w)
{
  if (w < 0 || w >= IK_WHEELS) {
    snprintf(c->link->err, sizeof c->link->err, "wheel must be 0..3");
    return -1;
  }
  esc_ctl_wheel_t *s = &c->wheel[w];
  s->armed = false;
  ik_link_row_t rows[IK_WHEELS], reply;
  if (ik_link_rows(c->link, rows)) return -1;
  if (!rows[w].valid) return wheel_error(c, w, "ARM", &rows[w]);
  struct motor_control control = {.mode = MOTOR_MODE_DISABLE, .flags = MOTOR_FLAG_ARM, .seq = ++s->seq};
  if (send_control(c, IK_WHEEL_SELECT[w], &control) ||
      wait_reply(c, w, rows[w].reply_count, control.seq, &reply)) return -1;
  if (reply.outcome != MOTOR_OUT_REJECTED || reply.reason != MOTOR_R_BAD_SESSION || !reply.session)
    return wheel_error(c, w, "ARM probe rejected", &reply);
  s->session = (uint8_t)reply.session;
  control.session = s->session;
  control.seq = ++s->seq;
  uint32_t before = reply.reply_count;
  if (send_control(c, IK_WHEEL_SELECT[w], &control) || wait_reply(c, w, before, control.seq, &reply)) return -1;
  if (reply.outcome != MOTOR_OUT_APPLIED || reply.session != s->session ||
      reply.lifecycle < MOTOR_LC_READY || reply.lifecycle > MOTOR_LC_HOLD)
    return wheel_error(c, w, "ARM failed", &reply);
  s->armed = true;
  return 0;
}

int esc_ctl_speed(esc_ctl_t *c, int w, int rpm)
{
  if (w < 0 || w >= IK_WHEELS || !c->wheel[w].armed || rpm < -2976 || rpm > 2976) {
    snprintf(c->link->err, sizeof c->link->err, "SPEED requires an armed wheel (0..3) and |rpm| <= 2976");
    return -1;
  }
  esc_ctl_wheel_t *s = &c->wheel[w];
  struct motor_control control = {.mode = MOTOR_MODE_SPEED, .setpoint = (int16_t)rpm,
    .seq = ++s->seq, .session = s->session};
  return send_control(c, IK_WHEEL_SELECT[w], &control);
}

int esc_ctl_clear_faults(esc_ctl_t *c)
{
  ik_link_t *l = c->link;
  uint32_t state, seq, done;
  ik_link_row_t before[IK_WHEELS], rows[IK_WHEELS];
  if (ik_read(l, IK_STATE, &state)) return -1;
  if (state != 0) {
    snprintf(l->err, sizeof l->err, "drive not stopped; CLEAR_FAULT refused");
    return -1;
  }
  /* Acknowledge the Nucleo's stop latch as well as clearing the ESC causes. */
  if (ik_write(l, IK_ENABLE, 0) || ik_link_rows(l, before) || ik_read(l, IK_CLEAR_SEQ, &seq)) return -1;
  for (int w = 0; w < IK_WHEELS; w++) c->wheel[w].armed = false;
  if (ik_write(l, IK_CLEAR_SEQ, ++seq)) return -1;
  double deadline = now_s() + c->timeout_s;
  for (;;) {
    if (ik_read(l, IK_CLEAR_DONE, &done)) return -1;
    if (done == seq) break;
    if (now_s() >= deadline) {
      snprintf(l->err, sizeof l->err, "CLEAR_FAULT: clear_done timed out");
      return -1;
    }
    pause_poll();
  }
  deadline = now_s() + c->timeout_s;
  for (;;) {
    if (ik_link_rows(l, rows)) return -1;
    bool ok = true;
    for (int w = 0; w < IK_WHEELS; w++)
      if (!before[w].valid || !rows[w].valid || rows[w].reply_count == before[w].reply_count ||
          rows[w].lifecycle > MOTOR_LC_HOLD) ok = false;
    if (ok) return 0;
    if (now_s() >= deadline) {
      snprintf(l->err, sizeof l->err, "CLEAR_FAULT timed out:");
      for (int w = 0; w < IK_WHEELS; w++) {
        size_t used = strlen(l->err);
        snprintf(l->err + used, sizeof l->err - used, " %s %s/%s%s;", IK_WHEEL_POS[w],
                 rows[w].valid ? rows[w].lifecycle_name : "unreadable",
                 rows[w].valid ? rows[w].reason_name : "UNKNOWN",
                 rows[w].valid && rows[w].reply_count != before[w].reply_count ? "" : " (no fresh reply)");
      }
      return -1;
    }
    pause_poll();
  }
}
