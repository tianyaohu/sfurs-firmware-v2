/**
 * @file elfsym.h
 * @brief Symbol lookup in a little-endian ELF32 or ELF64 file (firmware ELFs are
 *        ELF32; ELF64 lets the host tests use their own binary).
 */
#ifndef ELFSYM_H
#define ELFSYM_H

#include <stdint.h>

/**
 * @brief Find a symbol in .symtab.
 * @param path ELF file.
 * @param name symbol name.
 * @param addr receives st_value (truncated to 32 bits).
 * @param size receives st_size in bytes (may be NULL).
 * @return 0 if found, -1 if the file is unreadable, not ELF, or has no such symbol.
 */
int elfsym_lookup(const char *path, const char *name, uint32_t *addr, uint32_t *size);

#endif
