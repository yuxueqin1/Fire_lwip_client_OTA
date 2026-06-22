#ifndef __OTA_FLASH_H
#define __OTA_FLASH_H

#include <stdint.h>

/* Flash操作返回值 */
typedef enum {
    OTA_FLASH_OK = 0,
    OTA_FLASH_ERR_ERASE,
    OTA_FLASH_ERR_WRITE,
    OTA_FLASH_ERR_ADDR,
    OTA_FLASH_ERR_SIZE
} ota_flash_result_t;

/* 初始化Flash */
void ota_flash_init(void);

/* 擦除指定地址和大小的Flash */
ota_flash_result_t ota_flash_erase(uint32_t addr, uint32_t size);

/* 读Flash */
ota_flash_result_t ota_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

/* 写Flash */
ota_flash_result_t ota_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/* 从App1读取头信息 */
ota_flash_result_t ota_flash_read_app1_header(void *header);

/* 从App2读取头信息 */
ota_flash_result_t ota_flash_read_app2_header(void *header);

/* 写App1头信息 */
ota_flash_result_t ota_flash_write_app1_header(const void *header);

/* 写App2头信息 */
ota_flash_result_t ota_flash_write_app2_header(const void *header);

/* 写App头信息 */
ota_flash_result_t ota_flash_write_header(uint32_t app_addr, const void *header);

/* 写BootInfo */
int boot_info_write(const void *info);

/* 读BootInfo */
void* boot_info_read(void);

#endif /* __OTA_FLASH_H */
