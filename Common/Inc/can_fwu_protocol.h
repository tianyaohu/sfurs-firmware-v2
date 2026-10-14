#ifndef CAN_FWU_PROTOCOL_H
#define CAN_FWU_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* P2: CAN firmware-update wire protocol, shared Nucleo <-> ESC.
 * Bus: 500 kbit/s, FD_NO_BRS, 8-byte payloads (classic-compatible).
 * Drive/telemetry live in 0x080..0x4FF/0x780; FWU owns 0x600..0x6FF.
 * Low nibble carries the wheel device bits (MOTOR_NE=1, SE=2, SW=4, NW=8). */
#define CAN_FWU_PROTO_VERSION 1U
#define CAN_FWU_MAGIC 0x46555743U /* "CFWU" little-endian */
#define CAN_FWU_BOOT_VERSION 1U

#define CAN_FWU_ENTER_BASE 0x600U
#define CAN_FWU_ERASE_BASE 0x610U
#define CAN_FWU_DATA_BASE 0x620U
#define CAN_FWU_VERIFY_BASE 0x630U
#define CAN_FWU_JUMP_BASE 0x640U
#define CAN_FWU_ACK_BASE 0x680U

#define CAN_FWU_PAGE_SIZE 2048U
#define CAN_FWU_BOOT_SIZE (16U * 1024U)
#define CAN_FWU_APP_MAX (112U * 1024U)
#define CAN_FWU_APP_OFFSET 0x4000U
#define CAN_FWU_CHUNK 6U
#define CAN_FWU_ENTER_TIMEOUT_MS 2000U
#define CAN_FWU_HOST_ACK_TIMEOUT_MS 100U
#define CAN_FWU_TARGET_IDLE_TIMEOUT_MS 2000U

enum can_fwu_status {
  CAN_FWU_OK = 0U,
  CAN_FWU_REJECT = 1U,      /* bad state, motor active, bad size/magic */
  CAN_FWU_BAD_SEQ = 2U,
  CAN_FWU_CRC_MISMATCH = 3U,
  CAN_FWU_FLASH_ERR = 4U,
  CAN_FWU_BAD_SIZE = 5U
};

static inline uint32_t can_fwu_id(uint32_t base, uint32_t dev)
{
  return base | (dev & 0xFU);
}

/* Payload layouts (all 8 bytes, little-endian):
 * ENTER:  [magic u32][img_size u32]
 * ERASE:  [start_page u32][num_pages u32]
 * DATA:   [seq u16][6 chunk bytes]
 * VERIFY: [crc u32][size u32]
 * JUMP:   [magic u32][0 u32]
 * ACK:    [status u32][detail u32] (detail = seq / crc / boot version) */
void can_fwu_put_u32(uint8_t out[8], uint32_t v0, uint32_t v1);
void can_fwu_enter_pack(uint8_t out[8], uint32_t img_size);
void can_fwu_erase_pack(uint8_t out[8], uint32_t start_page, uint32_t num_pages);
void can_fwu_data_pack(uint8_t out[8], uint16_t seq, const uint8_t chunk[6],
                       uint32_t chunk_len);
void can_fwu_verify_pack(uint8_t out[8], uint32_t crc, uint32_t size);
void can_fwu_jump_pack(uint8_t out[8]);
void can_fwu_ack_pack(uint8_t out[8], uint32_t status, uint32_t detail);
uint32_t can_fwu_get_u32(const uint8_t in[8], unsigned word);
uint16_t can_fwu_data_seq(const uint8_t in[8]);
uint32_t can_fwu_data_count(uint32_t img_size);
uint32_t can_fwu_pages_for_size(uint32_t img_size);

uint32_t can_fwu_crc32(const uint8_t *data, size_t len);
uint32_t can_fwu_crc32_update(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
#endif
