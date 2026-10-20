#ifndef IK_TELEMETRY_H
#define IK_TELEMETRY_H
#include <stdint.h>

/**
 * @file ik_telemetry.h
 * @brief Per-wheel ESC telemetry mirrored into RAM for the debugger.
 *
 * Debugger ABI: four rows in FR/BR/BL/FL (selectors 1/2/4/8) order, every
 * field 32 bits. Read a row between two equal, even generation values.
 * Status, diag and ident replies (can_datatypes.h) refresh separately, each
 * with its own tick and count; count == 0 means that reply never arrived.
 */

/** @brief Last reply data of one wheel; layout is the host ABI (19 words). */
struct ik_telemetry_wheel {
  uint32_t generation;       /**< even = stable, odd = being written */
  int32_t rpm;               /**< motor speed, RPM */
  int32_t iq_counts;         /**< measured Iq, 544 counts per A */
  int32_t iq_ref_counts;     /**< Iq reference, 544 counts per A */
  uint32_t state;            /**< MCSDK state machine state */
  uint32_t flags;            /**< status flags, see DRIVE.md */
  uint32_t status_tick;      /**< Nucleo ms of the last status reply */
  uint32_t status_count;     /**< status replies received */
  uint32_t vbus_centivolts;  /**< bus voltage, 0.01 V */
  uint32_t faults;           /**< MCSDK faults that occurred */
  int32_t hall_shift_deg;    /**< Hall phase shift, degrees */
  uint32_t align_result;     /**< last Hall alignment result */
  uint32_t i2t_percent;      /**< I2T budget used, % */
  uint32_t diag_tick;        /**< Nucleo ms of the last diag reply */
  uint32_t diag_count;       /**< diag replies received */
  uint32_t app_crc;          /**< CRC of the installed ESC application */
  uint32_t app_size;         /**< size of the installed ESC application, bytes */
  uint32_t ident_tick;       /**< Nucleo ms of the last ident reply */
  uint32_t ident_count;      /**< ident replies received */
};
/** @brief Words per wheel row of ::ik_telemetry_wheel. */
#define IK_TELEMETRY_WORDS 19U
/** @brief Telemetry rows, FR/BR/BL/FL. */
extern volatile struct ik_telemetry_wheel ik_drive_telem[4];
/** @brief Nucleo ms at the last telemetry update. */
extern volatile uint32_t ik_drive_telem_now;

/** @brief Number of 6-channel current-loop capture samples stored. */
#define IK_CAPTURE_SAMPLES 512U
/**
 * @brief Current-loop capture readout.
 *
 * CAPTURE_REPLY frames from any wheel land here by sample index; the host
 * requests one wheel's readout at a time (raw CAPTURE frames) and counts
 * frames to know when its chunk has arrived.
 */
struct ik_capture {
  uint32_t frames;  /**< CAPTURE_REPLY frames stored */
  uint32_t source;  /**< selector of the last frame */
  uint32_t status;          /**< info bank: capture status */
  uint32_t vbus_centivolts; /**< info bank: bus voltage, 0.01 V */
  uint32_t samples;         /**< info bank: samples recorded */
  int16_t sample[IK_CAPTURE_SAMPLES][6]; /**< Id, Iq, Vd, Vq, Id ref, angle */
};
/** @brief Capture buffer, written by the CAN receive path. */
extern volatile struct ik_capture ik_drive_capture;

/** @brief Spacing of telemetry poll requests, ms. */
#define IK_TELEMETRY_POLL_MS 5U
/** @brief Slots in the poll cycle. */
#define IK_TELEMETRY_SLOTS 17U
/**
 * @brief Next telemetry request that is due.
 *
 * Call every loop. A 17-slot cycle: three status polls and one diag poll per
 * wheel, then one ident poll for the next wheel in turn.
 * @param now Nucleo time, ms.
 * @return STATUS_READ, DIAG_READ or IDENT_READ id (send 4 zero bytes) once
 *         per ::IK_TELEMETRY_POLL_MS, otherwise 0.
 */
uint32_t ik_telemetry_poll_due(uint32_t now);
/**
 * @brief Feed one received CAN frame to the telemetry parser.
 *
 * The caller checks standard ID, data frame and DLC 8.
 * @param id   CAN ID.
 * @param data 8 payload bytes.
 * @param now  Nucleo time, ms.
 * @return 1 if the frame was a telemetry reply and was consumed, else 0.
 */
int ik_telemetry_rx(uint32_t id, const uint8_t data[8], uint32_t now);
#endif
