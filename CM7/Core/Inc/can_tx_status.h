#ifndef CAN_TX_STATUS_H
#define CAN_TX_STATUS_H
#include <stdint.h>

/* FDCAN TX buffer outcome from TXBTO (occurred) / TXBCF (cancel finished).
 * No flag yet means pending, even if TXBRP just cleared; callers time out. */
enum { CAN_TX_PENDING = 0, CAN_TX_OK = 1, CAN_TX_FAILED = 2 };
static inline int can_tx_status(uint32_t occurred, uint32_t canceled, uint32_t mask)
{
  if ((occurred & mask) != 0U) return CAN_TX_OK;
  if ((canceled & mask) != 0U) return CAN_TX_FAILED;
  return CAN_TX_PENDING;
}
#endif
