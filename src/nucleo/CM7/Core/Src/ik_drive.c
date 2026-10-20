#include "ik_drive.h"
#include "can_datatypes.h"
#include "rover_kinematics.h"
#include <math.h>

volatile const uint32_t ik_drive_abi = IK_DRIVE_ABI;
volatile int32_t ik_drive_vx_mmps, ik_drive_vy_mmps, ik_drive_omega_mradps;
volatile uint32_t ik_drive_enable, ik_drive_heartbeat;
volatile int32_t ik_drive_accel_rpmps;
volatile uint32_t ik_drive_raw_id, ik_drive_raw_data, ik_drive_raw_seq;
volatile uint32_t ik_drive_state, ik_drive_raw_done;
volatile int32_t ik_drive_target_rpm[4], ik_drive_rpm[4];
volatile uint32_t ik_drive_tx_count, ik_drive_tx_fail, ik_drive_bad_input;

static uint32_t last_heartbeat, heartbeat_tick, period_tick, hold_tick, stop_frames;
static int heartbeat_seen;
static float command[4];

static int send_counted(int (*send)(uint32_t, const uint8_t[4]), uint32_t id, const uint8_t d[4])
{
  if (send(id, d)) { ++ik_drive_tx_count; return 1; }
  ++ik_drive_tx_fail;
  return 0;
}

/* Target wheel RPM for the requested twist; zero when disabled or invalid. */
static void targets(int drive, float target[4])
{
  for (unsigned i = 0; i < 4U; ++i) target[i] = 0.0f;
  if (!drive) return;
  const int32_t vx = ik_drive_vx_mmps, vy = ik_drive_vy_mmps, omega = ik_drive_omega_mradps;
  if (vx < -IK_DRIVE_MAX_LINEAR_MMPS || vx > IK_DRIVE_MAX_LINEAR_MMPS ||
      vy < -IK_DRIVE_MAX_LINEAR_MMPS || vy > IK_DRIVE_MAX_LINEAR_MMPS ||
      omega < -IK_DRIVE_MAX_OMEGA_MRADPS || omega > IK_DRIVE_MAX_OMEGA_MRADPS ||
      !rover_ik(rover_ik_provisional, vx / 1000.0f, vy / 1000.0f, omega / 1000.0f, target, 0)) {
    ++ik_drive_bad_input;
    for (unsigned i = 0; i < 4U; ++i) target[i] = 0.0f;
  }
}

/* Move every wheel toward its target; the wheel needing the largest change
 * moves at the acceleration limit and the others proportionally, so the
 * body accelerates along the commanded twist instead of curving. */
static void slew(const float target[4])
{
  const int32_t accel = ik_drive_accel_rpmps > 0 ? ik_drive_accel_rpmps : IK_DRIVE_DEFAULT_ACCEL_RPMPS;
  const float max_step = (float)accel * (float)IK_DRIVE_PERIOD_MS / 1000.0f;
  float largest = 0.0f;
  for (unsigned i = 0; i < 4U; ++i) {
    const float delta = fabsf(target[i] - command[i]);
    if (delta > largest) largest = delta;
  }
  const float scale = largest > max_step ? max_step / largest : 1.0f;
  for (unsigned i = 0; i < 4U; ++i) {
    command[i] = scale >= 1.0f ? target[i] : command[i] + (target[i] - command[i]) * scale;
    ik_drive_target_rpm[i] = (int32_t)lroundf(target[i]);
    ik_drive_rpm[i] = (int32_t)lroundf(command[i]);
  }
}

void ik_drive_tick(uint32_t now, int (*send)(uint32_t id, const uint8_t data[4]))
{
  const uint32_t heartbeat = ik_drive_heartbeat;
  if (heartbeat != last_heartbeat) {
    last_heartbeat = heartbeat;
    heartbeat_tick = now;
    heartbeat_seen = 1;
  }
  const int fresh = heartbeat_seen && (uint32_t)(now - heartbeat_tick) < IK_DRIVE_HEARTBEAT_MS;
  const int drive = fresh && ik_drive_enable != 0U;

  if (ik_drive_state == IK_STOPPED) {
    if (!drive) {
      const uint32_t raw_seq = ik_drive_raw_seq;
      if (raw_seq != ik_drive_raw_done) {
        uint8_t data[4];
        const uint32_t word = ik_drive_raw_data;
        CAN_PAYLOAD_FROM_U32(data, word);
        if (send_counted(send, ik_drive_raw_id & 0x7FFU, data)) ik_drive_raw_done = raw_seq;
      }
      return;
    }
    ik_drive_state = IK_DRIVING;
    period_tick = now - IK_DRIVE_PERIOD_MS;
  }
  if ((uint32_t)(now - period_tick) < IK_DRIVE_PERIOD_MS) return;
  period_tick = now;

  if (ik_drive_state == IK_STOPPING) {
    if (drive) {
      ik_drive_state = IK_DRIVING;
    } else {
      const uint8_t zero[4] = {0};
      if (send_counted(send, MOTOR_STOP | 0x0FU, zero) && ++stop_frames >= IK_DRIVE_STOP_FRAMES)
        ik_drive_state = IK_STOPPED;
      return;
    }
  }

  float target[4];
  targets(drive, target);
  slew(target);
  int moving = drive;
  for (unsigned i = 0; i < 4U; ++i) {
    const uint16_t rpm = (uint16_t)(int16_t)ik_drive_rpm[i];
    const uint8_t data[4] = {(uint8_t)rpm, (uint8_t)(rpm >> 8),
                             (uint8_t)IK_DRIVE_PERIOD_MS, 0U}; /* ESC ramps over one period */
    (void)send_counted(send, SPEED_WRITE | (1U << i), data);
    moving |= ik_drive_rpm[i] != 0;
  }
  if (moving) {
    hold_tick = now;
  } else if ((uint32_t)(now - hold_tick) >= IK_DRIVE_HOLD_MS) {
    ik_drive_state = IK_STOPPING;
    stop_frames = 0U;
  }
}
