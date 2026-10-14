#ifndef WHEEL_BENCH_H
#define WHEEL_BENCH_H
#include <stdint.h>

/* Original ESC profile only. Not a current limiter or an ESC-local watchdog. */
#define BENCH_ARM  0x41524D32U
#define BENCH_RUN  0x52554E32U
#define BENCH_STOP 0x53544F50U
#define BENCH_ARM_BUTTON 0x4142544EU /* ABTN: explicit 60-second button window */
enum { BENCH_IDLE, BENCH_ARMED, BENCH_RUNNING, BENCH_STOPPING, BENCH_FAULT };
enum { BENCH_TX_PENDING, BENCH_TX_OK, BENCH_TX_FAILED };
static inline int wheel_bench_tx_status(uint32_t occurred, uint32_t canceled,
                                        uint32_t mask)
{
  if ((occurred & mask) != 0U) return BENCH_TX_OK;
  if ((canceled & mask) != 0U) return BENCH_TX_FAILED;
  /* No completion flag yet: poll again, even if TXBRP has just cleared.
   * The next pass sees completion; the service still enforces 100 ms timeout. */
  return BENCH_TX_PENDING;
}
extern volatile uint32_t wheel_bench_request, wheel_bench_wheel;
extern volatile int32_t wheel_bench_direction;
extern volatile uint32_t wheel_bench_state, wheel_bench_result;
extern volatile uint32_t wheel_bench_run_count;
extern volatile int32_t wheel_bench_speed_rpm;
/* Speed setting (RPM magnitude), 0 = per-selection firmware default.
 * NE reverse: -120..-30 (0 keeps -100). NE forward: legacy +30, ignores
 * this. SE/SW/NW: 10..120 (0 keeps 30). Clamped at RUN time. */
extern volatile uint32_t wheel_bench_last_request, wheel_bench_request_count;
extern volatile uint32_t wheel_bench_tx_count, wheel_bench_tx_ack_count;
extern volatile uint32_t wheel_bench_first_fault, wheel_bench_first_fault_tick;
/* First fault: 1=enqueue failed, 2=TX canceled/failed, 3=100 ms timeout.
 * Diagnostics only; fault remains latched and blocks drive commands. */
/* result: 0=none, 1=armed, 2=run queued, 3=STOP frames ACKed (not stopped
 * feedback), 4=rejected, 5=transport fault. run_count: completed RUNs, no
 * latch — repeats need a fresh ARM after the cooldown. */
void wheel_bench_tick(uint32_t now);
void wheel_bench_button(uint32_t now, uint8_t pressed);
/* Implemented by the Nucleo adapter; host tests substitute a fake transport. */
int wheel_bench_send(uint32_t id, const uint8_t data[4]);
int wheel_bench_poll(void);
void wheel_bench_abort(void);
#endif
