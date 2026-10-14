#ifndef CAN_CAL_SENDER_H
#define CAN_CAL_SENDER_H
#include <stdint.h>
#include "can_cal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Nucleo-side CAL1 sender. Mirrors the wheel_bench safety model:
 * nothing is sent unless a debugger explicitly writes can_cal_request,
 * the caller passes bench-idle true (BENCH_IDLE), and the device has
 * not already been calibrated this boot (one-shot used-mask).
 * Exactly one 0x480|dev + "CAL1" frame goes out per device per boot.
 * TX status codes match wheel_bench_tx_status so firmware can wire
 * wheel_bench_send/poll/abort directly as the transport. */
enum { CAN_CAL_TX_PENDING, CAN_CAL_TX_OK, CAN_CAL_TX_FAILED };

/* Explicit debugger abort, same value as BENCH_STOP ("STOP"). */
#define CAN_CAL_STOP 0x53544F50U

extern volatile uint32_t can_cal_request; /* dev bits 1/2/4/8, or CAN_CAL_STOP */
extern volatile uint32_t can_cal_result;  /* enum can_cal_result */
extern volatile uint32_t can_cal_used_mask;
extern volatile uint32_t can_cal_last_request, can_cal_request_count;
extern volatile uint32_t can_cal_tx_count, can_cal_tx_ack_count;
extern volatile uint32_t can_cal_first_fault, can_cal_first_fault_tick;
/* First fault: 1=enqueue failed, 2=TX canceled/failed, 3=100 ms timeout.
 * Diagnostics only; fault remains latched and blocks further CAL sends. */

/* Consumes can_cal_request exactly once per call. bench_idle must be
 * nonzero (BENCH_IDLE); refused otherwise. STOP aborts a pending frame
 * even when not idle. */
void can_cal_tick(uint32_t now, int bench_idle);

/* Implemented by the Nucleo adapter; host tests substitute a fake. */
int can_cal_send(uint32_t id, const uint8_t data[4]);
int can_cal_poll(void);
void can_cal_abort(void);

#ifdef __cplusplus
}
#endif
#endif
