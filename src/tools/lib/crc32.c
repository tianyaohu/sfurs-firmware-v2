#include "crc32.h"

uint32_t crc32_ieee(const void *data, size_t len)
{
  const uint8_t *p = data;
  uint32_t crc = 0xFFFFFFFFU;
  while (len--) {
    crc ^= *p++;
    for (int bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1U));
  }
  return ~crc;
}
