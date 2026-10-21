/**
 * @file crc32.h
 * @brief CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320), same as zlib.crc32.
 */
#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief CRC-32 of a buffer.
 * @param data bytes.
 * @param len  number of bytes.
 * @return CRC-32 value; crc32_ieee("123456789", 9) == 0xCBF43926.
 */
uint32_t crc32_ieee(const void *data, size_t len);

#endif
