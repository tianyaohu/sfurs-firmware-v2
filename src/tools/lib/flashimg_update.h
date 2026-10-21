#ifndef FLASHIMG_UPDATE_H
#define FLASHIMG_UPDATE_H
#include "ik_link.h"
#include <stdio.h>

#define FLASHIMG_FLASH_SIZE 0x20000U
#define FLASHIMG_APP_OFFSET 0x4000U
#define FLASHIMG_META_OFFSET 0x1F800U
#define FLASHIMG_NUCLEO_SN "002200364142500620353451"
#define FLASHIMG_DEFAULT_BACKUPS "/home/fh/sfurs_firmware/esc-backups"
#define FLASHIMG_DEFAULT_CLI "/home/fh/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
#ifndef FLASHIMG_DEFAULT_BUILD
#define FLASHIMG_DEFAULT_BUILD "build"
#endif

typedef struct {
  int wheel;
  bool can, battery_off, target_in_boot, keep_updater;
  const char *build, *backups, *cli;
} flashimg_options_t;
extern const char *const FLASHIMG_ESC_SN[4];

typedef struct { uint32_t sensor, mci, tick; } flashimg_battery_symbols_t;
typedef struct { uint32_t tick; double vbus_v; uint8_t mc_state; } flashimg_battery_t;

/** @brief Parse the installer CLI without accessing files or hardware; 1 means help. */
int flashimg_parse(int argc, char **argv, flashimg_options_t *o, char *err, size_t cap);
/** @brief Print the installation procedure and options. */
void flashimg_usage(FILE *f);
/** @brief Run one installation, closing all probes on every return. */
int flashimg_install(const flashimg_options_t *o, char *err, size_t cap);
/** @brief Read an entire file; caller frees *data. */
int flashimg_read_file(const char *path, uint8_t **data, size_t *size);
/** @brief Compare allocated file-backed ELF sections at their physical load addresses, never padding. */
bool flashimg_matches(const char *elf, const uint8_t *flash, size_t size);
/** @brief Find the first matching built or archived ELF; 1 found, 0 unidentified, -1 error. */
int flashimg_installed(const char *build, const char *backups, const uint8_t *flash,
                      char *path, size_t cap);
/** @brief Resolve the installed app's own RAM symbols, refusing undersized objects. */
int flashimg_battery_symbols(const char *elf, flashimg_battery_symbols_t *s, char *err, size_t cap);
/** @brief Read the app's tick, voltage sensor and MC state through OpenOCD. */
int flashimg_battery_read(ocd_t *o, const flashimg_battery_symbols_t *s,
                         flashimg_battery_t *b, char *err, size_t cap);
/** @brief Refuse stale ticks, battery >= 8 V or motor states other than 0, 10, 11. */
int flashimg_battery_validate(uint32_t before_tick, const flashimg_battery_t *b, char *err, size_t cap);
/** @brief Check a running installed app twice, 50 ms apart, before allowing an SWD installation. */
int flashimg_check_battery_off(ocd_t *o, const char *elf, flashimg_battery_t *b, char *err, size_t cap);
/** @brief Invalidate cached status rows whose counters did not advance. */
void flashimg_live_rows(const ik_row_t before[4], ik_row_t after[4]);
/** @brief Require answering, IDLE wheels with Vbus; skip is -1 or the recovery wheel index. */
int flashimg_check_bus(const ik_row_t rows[4], int skip, double vbus_v[4], char *err, size_t cap);
/** @brief Program only the Nucleo with fork/exec and require successful verification text and exit. */
int flashimg_program_nucleo(const char *cli, const char *elf, const char *log, char *err, size_t cap);
/** @brief Check bootloader, app bytes and metadata against the full before/after flash images. */
void flashimg_verify_flash(const uint8_t before[FLASHIMG_FLASH_SIZE],
                          const uint8_t after[FLASHIMG_FLASH_SIZE], const uint8_t *app,
                          size_t size, uint32_t crc, bool checks[3]);
#endif
