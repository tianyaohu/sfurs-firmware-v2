#ifndef ESC_BLINK_CAN_BOOT_H
#define ESC_BLINK_CAN_BOOT_H
#include <stdint.h>
int esc_blink_can_init(void);
int esc_blink_can_poll(uint32_t *id, uint8_t payload[8]);
#endif
