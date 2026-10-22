/** @file esc_ctl.h
 * @brief Protocol v1 control of individual wheels while the Nucleo drive is stopped.
 */
#ifndef ESC_CTL_H
#define ESC_CTL_H
#include "ik_link.h"
#include "motor_protocol.h"

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
 * @param link Open ABI 8 mailbox link.
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

/** @brief Send raw test bytes and wait for matching replies on every selected wheel.
 * @param replies Four rows, populated for the CAN select bits in can_id.
 * @return 0 on fresh replies within ctl->timeout_s seconds, -1 on failure.
 */
int esc_ctl_exchange(esc_ctl_t *ctl, uint32_t can_id, const uint8_t data[8], ik_link_row_t replies[IK_WHEELS]);
/** @brief Send plain DISABLE to one wheel and wait for its reply (timeout in seconds in ctl). */
int esc_ctl_disable(esc_ctl_t *ctl, int wheel);

typedef int (*esc_reject_builder_t)(int wheel, uint8_t seq, uint8_t session, uint32_t *id, uint8_t data[8]);
typedef struct {
  const char *name;
  esc_reject_builder_t build;
  enum motor_outcome outcome;
  enum motor_reason reason;
  bool disarmed;
} esc_reject_case_t;
/** @brief Return the no-motion reject case table; builders take the last accepted sequence/session. */
const esc_reject_case_t *esc_ctl_reject_cases(size_t *count);

typedef struct {
  uint64_t count;
  uint32_t p50_us, p99_us, max_us; /**< Bin upper edges, microseconds; UINT32_MAX means >3750 us. */
} esc_latency_t;
/** @brief Summarize 250 us histogram bins; empty histograms return zero edges/count. */
esc_latency_t esc_ctl_latency(const uint64_t bins[IK_LAT_BINS]);
/** @brief Estimate CAN load in percent at 500 kbit/s, using 130 bits/8-byte and 100 bits/4-byte frame.
 * @param seconds Measured counter interval in seconds; nonpositive returns zero.
 */
double esc_ctl_bus_load(uint64_t frames8, uint64_t frames4, double seconds);
/** @brief Infer per-wheel addressed frames after three stop broadcasts, or fail on incomplete cycles. */
int esc_ctl_stream_tx(uint32_t tx_count, uint32_t *per_wheel);
#endif
