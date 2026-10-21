/**
 * @file ocd.h
 * @brief SWD memory access through OpenOCD: one OpenOCD process per ST-LINK
 *        probe, driven over its Tcl RPC port.
 *
 * Attach only: never halts, resets (except ocd_reset_sysreq) or flashes.
 * Flashing goes through STM32CubeProgrammer.
 */
#ifndef OCD_H
#define OCD_H

#include <stddef.h>
#include <stdint.h>

typedef struct ocd ocd_t;

/**
 * @brief Start OpenOCD for one probe and connect to its Tcl port.
 * @param binary OpenOCD executable (NULL: $OPENOCD, else "openocd").
 * @param serial ST-LINK serial number.
 * @param chip   "stm32h755" (Nucleo CM7) or "stm32g431" (ESC).
 * @param err    receives a message on failure.
 * @param errlen size of err in bytes.
 * @return session, or NULL on failure.
 */
ocd_t *ocd_open(const char *binary, const char *serial, const char *chip, char *err, size_t errlen);

/** @brief Stop OpenOCD; the target keeps running. Safe on NULL. */
void ocd_close(ocd_t *o);

/** @brief Last error message of this session. */
const char *ocd_error(const ocd_t *o);

/**
 * @brief Read count items of 8 or 32 bits.
 * @param width bits per item, 8 or 32.
 * @param out   count entries, one value each.
 * @return 0 on success, -1 on error (see ocd_error).
 */
int ocd_read(ocd_t *o, uint32_t addr, unsigned width, size_t count, uint32_t *out);

/** @brief Read one 32-bit word. */
int ocd_read32(ocd_t *o, uint32_t addr, uint32_t *value);

/** @brief Read n bytes into buf. */
int ocd_read_bytes(ocd_t *o, uint32_t addr, uint8_t *buf, size_t n);

/** @brief Write one 32-bit word. */
int ocd_write32(ocd_t *o, uint32_t addr, uint32_t value);

/** @brief Core reset through AIRCR.SYSRESETREQ; a dropped link is not an error. */
void ocd_reset_sysreq(ocd_t *o);

/**
 * @brief OpenOCD command that selects a probe serial: "adapter serial" from
 *        0.12, "hla_serial" before (0.11 has only the ST-LINK one).
 * @param version_text output of `openocd --version`.
 * @param serial       probe serial.
 * @param out          buffer for the command (including the serial).
 * @param outlen       size of out in bytes.
 */
void ocd_serial_command(const char *version_text, const char *serial, char *out, size_t outlen);

#endif
