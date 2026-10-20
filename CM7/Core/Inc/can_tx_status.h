#ifndef CAN_TX_STATUS_H
#define CAN_TX_STATUS_H
#include <stdint.h>

/** @file can_tx_status.h
 *  @brief Outcome of one FDCAN transmit buffer. */

/** Result of ::can_tx_status. */
enum {
  CAN_TX_PENDING = 0, /**< no flag yet */
  CAN_TX_OK = 1,      /**< transmitted */
  CAN_TX_FAILED = 2   /**< cancelled */
};
/**
 * @brief FDCAN TX buffer outcome from TXBTO (occurred) / TXBCF (cancel finished).
 *
 * No flag yet means pending, even if TXBRP just cleared; callers time out.
 * @param occurred TXBTO register
 * @param canceled TXBCF register
 * @param mask     buffer bit of the request
 */
static inline int can_tx_status(uint32_t occurred, uint32_t canceled, uint32_t mask)
{
  if ((occurred & mask) != 0U) return CAN_TX_OK;
  if ((canceled & mask) != 0U) return CAN_TX_FAILED;
  return CAN_TX_PENDING;
}
#endif
