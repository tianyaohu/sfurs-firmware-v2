#ifndef SHA256_H
#define SHA256_H
#include <stddef.h>
#include <stdint.h>
/** @brief SHA-256 of a byte buffer, as 64 lowercase hexadecimal digits plus NUL. */
void sha256_hex(const void *data, size_t size, char hex[65]);
#endif
