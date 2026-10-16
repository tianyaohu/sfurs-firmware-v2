#ifndef WHEEL_BENCH_H
#define WHEEL_BENCH_H
#include <stdint.h>

/* Original ESC profile only. Not a current limiter or an ESC-local watchdog. */
#define BENCH_ARM  0x41524D32U
#define BENCH_RUN  0x52554E32U
#define BENCH_STOP 0x53544F50U
#define BENCH_ARM_BUTTON 0x4142544EU /* ABTN: explicit 60-second button window */
#define BENCH_TUNE 0x47535431U /* GST1: queue RPM2-tune GAIN_SET (0x500|dev) */
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
/* RPM2 speed-PI numerators for BENCH_TUNE (int16 range, both negative;
 * kp magnitude >= 1000). Applied by the ESC only while armed+IDLE; RAM-only. */
extern volatile int32_t wheel_bench_tune_kp, wheel_bench_tune_ki;
extern volatile uint32_t wheel_bench_state, wheel_bench_result;
extern volatile uint32_t wheel_bench_run_count;
extern volatile int32_t wheel_bench_speed_rpm;
/* Speed setting (RPM magnitude), 0 = per-selection firmware default.
 * NE: +-300 (0 keeps -100 reverse / +30 forward). SE/SW/NW: 10..150
 * (0 keeps 30). Clamped at RUN time. */
extern volatile uint32_t wheel_bench_last_request, wheel_bench_request_count;
extern volatile uint32_t wheel_bench_tx_count, wheel_bench_tx_ack_count;
extern volatile uint32_t wheel_bench_first_fault, wheel_bench_first_fault_tick;
/* Tier-1 ESC state mirror (RAM only, no ESC changes). Logical widths:
 * rpm i16, state u8, age_ms u32 saturating. Stored in 32-bit words so the
 * debugger/TUI can use plain read32. age 0xFFFFFFFF = no reply yet.
 * Updated only from wheel_bench_esc_rx (0x280|ID replies) and aged by
 * wheel_bench_tick; never blocks ARM/RUN/STOP. */
extern volatile int32_t wheel_bench_esc_rpm;
extern volatile uint32_t wheel_bench_esc_state, wheel_bench_esc_age_ms;
/* First fault: 1=enqueue failed, 2=TX canceled/failed, 3=100 ms timeout.
 * Diagnostics only; fault remains latched and blocks drive commands. */
/* result: 0=none, 1=armed, 2=run queued, 3=STOP frames ACKed (not stopped
 * feedback), 4=rejected, 5=transport fault, 6=tune frame queued (TX ACK only;
 * ESC acceptance is proven by ESC RAM readback, never by this code).
 * run_count: completed RUNs, no latch — repeats need a fresh ARM after the
 * cooldown. */
void wheel_bench_tick(uint32_t now);
void wheel_bench_button(uint32_t now, uint8_t pressed);
/* Tier-1 ESC poll hooks. esc_rx stores a validated 8-byte 0x280|ID reply
 * (rpm i16 LE, state u8, hall u8, hall-changes u32) when it matches the
 * current wheel_bench_wheel selection (NE/SE only). poll_due returns the
 * SPEED_READ CAN id (0x100|wheel) when a poll frame may be sent now, else 0:
 * selected wheel is NE/SE, bench is IDLE/ARMED/RUNNING/FAULT (never
 * STOPPING), no bench TX in flight, and 500 ms since the last poll. The
 * poll frame itself is sent by the adapter (HAL-direct, not via
 * wheel_bench_send) so bench TX tracking is untouched. */
void wheel_bench_esc_rx(uint32_t id, const uint8_t data[8], uint32_t now);
uint32_t wheel_bench_esc_poll_due(uint32_t now);
/* Implemented by the Nucleo adapter; host tests substitute a fake transport. */
int wheel_bench_send(uint32_t id, const uint8_t data[4]);
int wheel_bench_poll(void);
void wheel_bench_abort(void);
#endif
