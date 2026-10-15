#ifndef ESC_BOOT_RX_H
#define ESC_BOOT_RX_H
#include "can_fwu_boot.h"
/* HAL writes the complete payload before returning the header. */
#define ESC_BOOT_RX_CAPACITY 64U
static inline int esc_boot_rx_header_valid(int standard, int data, int fd,
                                            int no_brs, int eight_bytes)
{ return standard && data && fd && no_brs && eight_bytes; }
static inline int esc_boot_rx_frame(struct can_fwu_boot *boot, int header_valid,
    uint32_t id, const uint8_t data[ESC_BOOT_RX_CAPACITY], uint32_t now,
    uint32_t *ack_id, uint8_t ack[8])
{
  if (!header_valid) return 0;
  return can_fwu_boot_frame(boot, id, data, now, ack_id, ack);
}
#endif
