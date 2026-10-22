/** @file esc_ctl.h
 * @brief Protocol v1 control of individual wheels while the Nucleo drive is stopped.
 */
#ifndef ESC_CTL_H
#define ESC_CTL_H
#include "ik_link.h"

typedef struct {
  uint8_t seq, session;
  bool armed;
} esc_ctl_wheel_t;
typedef struct {
  ik_link_t *link;
  esc_ctl_wheel_t wheel[IK_WHEELS];
  double timeout_s;
} esc_ctl_t;

/** @brief Initialize per-wheel sessions/sequences.
 * @param ctl Control state, owned by the caller.
 * @param link Open ABI 7 mailbox link.
 * @param timeout_s Timeout per mailbox/reply wait, seconds.
 */
void esc_ctl_init(esc_ctl_t *ctl, ik_link_t *link, double timeout_s);
/** @brief Broadcast plain DISABLE using the codec, regardless of session or sequence.
 * @param ctl Control state; wheel sessions are marked unarmed locally.
 * @return 0 on transmission, -1 with link->err on failure.
 */
int esc_ctl_disable_all(esc_ctl_t *ctl);
/** @brief Acknowledge the drive fault latch, clear ESC faults and require fresh healthy replies.
 * @param ctl Control state; the drive must be stopped.
 * @return 0 when all four ESCs reply without FAULTED, -1 with link->err otherwise.
 */
int esc_ctl_clear_faults(esc_ctl_t *ctl);
/** @brief Probe session zero, then ARM with the announced session.
 * @param ctl Control state; the drive must be stopped.
 * @param wheel Index 0..3 in mailbox order.
 * @return 0 on an armed reply, -1 with the rejection or timeout in link->err.
 */
int esc_ctl_arm(esc_ctl_t *ctl, int wheel);
/** @brief Send SPEED, advancing the wheel sequence; call every <=40 ms.
 * @param ctl Control state; the drive must be stopped.
 * @param wheel Previously armed wheel, index 0..3.
 * @param rpm Mechanical motor/wheel speed, RPM, magnitude <=2976.
 * @return 0 on transmission, -1 with link->err otherwise; replies are read separately.
 */
int esc_ctl_speed(esc_ctl_t *ctl, int wheel, int rpm);
#endif
