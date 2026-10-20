#ifndef CAN_FWU_PROTOCOL_H
#define CAN_FWU_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file can_fwu_protocol.h
 * @brief CAN firmware-update wire protocol, shared Nucleo <-> ESC.
 *
 * Bus: 500 kbit/s, FD_NO_BRS, 8-byte payloads (classic-compatible).
 * Drive/telemetry live in 0x080..0x4FF/0x780; FWU owns 0x600..0x6FF.
 * Low nibble carries the wheel device bits (MOTOR_NE=1, SE=2, SW=4, NW=8).
 */
#define CAN_FWU_PROTO_VERSION 2U  /**< wire protocol version */
#define CAN_FWU_MAGIC 0x3257U     /**< v2 ENTER magic16; deliberately not v1 "CFWU" */
#define CAN_FWU_BOOT_VERSION 1U   /**< bootloader version reported in the ENTER ACK */

#define CAN_FWU_ENTER_BASE 0x600U  /**< ENTER command ID base */
#define CAN_FWU_ERASE_BASE 0x610U  /**< ERASE command ID base */
#define CAN_FWU_DATA_BASE 0x620U   /**< DATA command ID base */
#define CAN_FWU_VERIFY_BASE 0x630U /**< VERIFY command ID base */
#define CAN_FWU_JUMP_BASE 0x640U   /**< JUMP command ID base */
#define CAN_FWU_ACK_BASE 0x680U    /**< ACK reply ID base */

#define CAN_FWU_PAGE_SIZE 2048U            /**< flash page size, bytes */
#define CAN_FWU_BOOT_SIZE (16U * 1024U)    /**< bootloader size, bytes */
#define CAN_FWU_APP_MAX (110U * 1024U)     /**< largest application; final 2K page reserved for validity metadata */
#define CAN_FWU_APP_MIN 8U                 /**< smallest application: initial stack pointer and reset vector */
#define CAN_FWU_APP_OFFSET 0x4000U         /**< application offset from flash base */
#define CAN_FWU_CHUNK 6U                   /**< image bytes per DATA frame */
#define CAN_FWU_ENTER_TIMEOUT_MS 2000U     /**< ENTER window after ESC reset, ms */
#define CAN_FWU_HOST_ACK_TIMEOUT_MS 100U   /**< host waits this long for an ACK, ms */
#define CAN_FWU_TARGET_IDLE_TIMEOUT_MS 2000U /**< target drops a session after this idle time, ms */

/** @brief ACK status codes. */
enum can_fwu_status {
  CAN_FWU_OK = 0U,
  CAN_FWU_REJECT = 1U,      /**< bad state, motor active, bad size/magic */
  CAN_FWU_BAD_SEQ = 2U,     /**< DATA out of sequence */
  CAN_FWU_CRC_MISMATCH = 3U, /**< image CRC differs from VERIFY */
  CAN_FWU_FLASH_ERR = 4U,   /**< erase, program or readback failed */
  CAN_FWU_BAD_SIZE = 5U     /**< image size out of range */
};

/** @brief Command ID for wheel @p dev; callers must validate, never alias invalid device bits. */
static inline uint32_t can_fwu_id(uint32_t base, uint32_t dev)
{
  return base | dev;
}
/** @brief Nonzero if @p dev is exactly one wheel bit (1, 2, 4 or 8). */
int can_fwu_dev_valid(uint32_t dev);

/**
 * Payload layouts (all 8 bytes, little-endian):
 * ENTER:  [magic u16][nonzero session u16][img_size u32]
 * ERASE:  [start_page u32][num_pages u32]
 * DATA:   [seq u16][6 chunk bytes]
 * VERIFY: [crc u32][size u32]
 * JUMP:   [magic u32][0 u32]
 * ACK:    [status u8][command u8 = request base >> 4][session u16][detail u32]
 * Detail identifies the request on ALL statuses: ENTER boot version,
 * ERASE requested page count, DATA requested seq, VERIFY requested crc, JUMP 0.
 * These are v2-only codecs; v1 peers must not be mixed with v2 peers.
 */
/** @brief Write two little-endian words into @p out. */
void can_fwu_put_u32(uint8_t out[8], uint32_t v0, uint32_t v1);
/** @brief Pack an ENTER frame with @p session and the image size. */
void can_fwu_enter_session_pack(uint8_t out[8], uint16_t session, uint32_t img_size);
/** @brief ENTER magic16 of a payload. */
uint16_t can_fwu_enter_magic(const uint8_t in[8]);
/** @brief ENTER session of a payload. */
uint16_t can_fwu_enter_session(const uint8_t in[8]);
/** @brief Convenience v2 ENTER using session 1; runtime hosts use the session API. */
void can_fwu_enter_pack(uint8_t out[8], uint32_t img_size);
/** @brief Pack an ERASE frame. */
void can_fwu_erase_pack(uint8_t out[8], uint32_t start_page, uint32_t num_pages);
/** @brief Pack a DATA frame: @p seq plus up to ::CAN_FWU_CHUNK bytes of @p chunk, padded with 0xFF. */
void can_fwu_data_pack(uint8_t out[8], uint16_t seq, const uint8_t chunk[6],
                       uint32_t chunk_len);
/** @brief Pack a VERIFY frame: image CRC-32 and size. */
void can_fwu_verify_pack(uint8_t out[8], uint32_t crc, uint32_t size);
/** @brief Pack a JUMP frame. */
void can_fwu_jump_pack(uint8_t out[8]);
/** @brief Pack a correlated ACK: status, the request's command base, session and detail. */
void can_fwu_ack_reply_pack(uint8_t out[8], uint32_t status, uint32_t command_base,
                            uint16_t session, uint32_t detail);
/** @brief Uncorrelated v2 ACK (command/session zero), NOT accepted by the host. */
void can_fwu_ack_pack(uint8_t out[8], uint32_t status, uint32_t detail);
/** @brief For port-level commit/readback failures: set status, preserve command/session/detail. */
void can_fwu_ack_set_status(uint8_t ack[8], uint32_t status);
/** @brief Preserve command/session while deliberately replacing status/detail. */
void can_fwu_ack_set_status_detail(uint8_t ack[8], uint32_t status, uint32_t detail);
/** @brief ACK status code, see ::can_fwu_status. */
uint32_t can_fwu_ack_status(const uint8_t in[8]);
/** @brief ACK command: the request base >> 4. */
uint32_t can_fwu_ack_command(const uint8_t in[8]);
/** @brief ACK session. */
uint16_t can_fwu_ack_session(const uint8_t in[8]);
/** @brief ACK detail word. */
uint32_t can_fwu_ack_detail(const uint8_t in[8]);
/** @brief Little-endian word @p word (0 or 1) of a payload. */
uint32_t can_fwu_get_u32(const uint8_t in[8], unsigned word);
/** @brief DATA sequence number of a payload. */
uint16_t can_fwu_data_seq(const uint8_t in[8]);
/** @brief DATA frames needed for an image of @p img_size bytes. */
uint32_t can_fwu_data_count(uint32_t img_size);
/** @brief Flash pages needed for an image of @p img_size bytes. */
uint32_t can_fwu_pages_for_size(uint32_t img_size);

/** @brief CRC-32 of a buffer. */
uint32_t can_fwu_crc32(const uint8_t *data, size_t len);
/** @brief Continue a CRC-32 over more bytes; start from 0 to match ::can_fwu_crc32. */
uint32_t can_fwu_crc32_update(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
#endif
