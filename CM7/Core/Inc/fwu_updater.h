#ifndef FWU_UPDATER_H
#define FWU_UPDATER_H
#include <stdint.h>
#include "can_fwu_host.h"

/* Nucleo updater image: sends the embedded ESC application (fwu_test_image)
 * to one ESC over CAN. Nothing happens without an explicit request:
 *  - debugger: set can_fwu_dev (1/2/4/8), then can_fwu_request = 1;
 *  - cable-free: hold B1 for 3 s from power-up.
 * ENTER repeats every 300 ms until the ESC bootloader answers (reset the ESC
 * to open its window). After ENTER is acknowledged, any failure latches and
 * needs a new request; erased/written sessions are never retried silently.
 * can_fwu_result: 0 none, 1 active, 2 rejected, 3 done, 4 failed.
 * can_fwu_enter_boot = 1 asks a running ESC app to reboot into its
 * bootloader (0x650|dev); can_fwu_boot_result: 1 sent, 2 rejected, 3 TX failed. */
extern volatile uint32_t can_fwu_request, can_fwu_dev, can_fwu_result;
extern volatile uint32_t can_fwu_enter_boot, can_fwu_boot_result, can_fwu_rx_cnt;
extern struct can_fwu_host fwu_host;

/* clock() is re-read after begin(), whose image CRC takes ~190 ms. */
void fwu_updater_init(const struct can_fwu_transport *transport, uint32_t (*clock)(void));
void fwu_updater_tick(uint32_t now, int button_held);
void fwu_updater_rx(uint32_t id, const uint8_t data[8], uint32_t now);
int fwu_updater_active(void);
#endif
