#ifndef CAN_CAL_H
#define CAN_CAL_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CAN-triggered ESC calibration command, shared Nucleo <-> ESC.
 * Bus: 500 kbit/s, FD_NO_BRS, 4-byte payload (same framing as bench drive).
 * Command slot 9 (0x480 base) is free: drive/telemetry use
 * 1,2,3,5,6,7,8,15 (x<<7), FWU owns 0x600..0x6FF, STOP is 0x78F.
 * Low nibble carries the wheel device bits (MOTOR_NE=1, SE=2, SW=4, NW=8).
 * Payload bytes read "CAL1" and match the ESC debugger convention
 * test_calibration_request == 0x43414C31 ("CAL1"). */
#define CAL_REQUEST (9U << 7)
#define CAL_MAGIC 0x43414C31U /* "CAL1" */

enum can_cal_result {
  CAN_CAL_NONE = 0U,      /* idle, nothing requested yet */
  CAN_CAL_REQUESTED = 1U, /* CAL frame queued, TX pending */
  CAN_CAL_ACKED = 2U,     /* bus TX acknowledged (not ESC calibrated) */
  CAN_CAL_REJECTED = 3U,  /* refused: busy, used, not idle, bad dev, aborted */
  CAN_CAL_FAILED = 4U     /* transport fault, latched */
};

static inline uint32_t can_cal_id(uint32_t dev)
{
  return (uint32_t)(CAL_REQUEST | (dev & 0xFU));
}

static inline int can_cal_dev_valid(uint32_t dev)
{
  return dev == 1U || dev == 2U || dev == 4U || dev == 8U;
}

/* Wire order is MSB-first so the four payload bytes read "CAL1" and
 * reconstruct CAL_MAGIC for the ESC's u32 compare. */
static inline void can_cal_pack(uint8_t out[4])
{
  out[0] = (uint8_t)(CAL_MAGIC >> 24);
  out[1] = (uint8_t)(CAL_MAGIC >> 16);
  out[2] = (uint8_t)(CAL_MAGIC >> 8);
  out[3] = (uint8_t)CAL_MAGIC;
}

static inline int can_cal_is_magic(const uint8_t in[4])
{
  return in[0] == (uint8_t)(CAL_MAGIC >> 24) &&
         in[1] == (uint8_t)(CAL_MAGIC >> 16) &&
         in[2] == (uint8_t)(CAL_MAGIC >> 8) &&
         in[3] == (uint8_t)CAL_MAGIC;
}

#ifdef __cplusplus
}
#endif
#endif
