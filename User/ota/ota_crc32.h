#ifndef __OTA_CRC32_H
#define __OTA_CRC32_H

#include <stdint.h>

/* CRC32计算 */
void crc32_init(void);
void crc32_update(uint32_t *crc, const uint8_t *data, uint32_t len);
uint32_t crc32_finalize(uint32_t crc);

/* 一次性计算CRC32 */
uint32_t crc32_calc(const uint8_t *data, uint32_t len);

#endif /* __OTA_CRC32_H */
