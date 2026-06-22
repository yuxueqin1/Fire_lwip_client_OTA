#include "ota_flash.h"
#include "ota_config.h"
#include "ota_manager.h"
#include "stm32f4xx_hal.h"
#include <string.h>

extern void vTaskDelay(const uint32_t xTicksToDelay);

/* STM32F429IGT6闪存扇区划分 */
//static const uint32_t flash_sectors[] = {
//    0x08000000, 0x08004000, 0x08008000, 0x0800C000,  // Sector 0-3  (16KB each)
//    0x08010000,                                       // Sector 4    (64KB)
//    0x08020000, 0x08040000, 0x08060000, 0x08080000, 0x080A0000, 0x080C0000, 0x080E0000// Sector 5-11  (128KB each)
//};

void ota_flash_init(void)
{
    /* Flash已在系统启动时初始化，此处为预留接口 */
    HAL_FLASH_Unlock();
}

static uint32_t ota_get_sector_index(uint32_t addr)
{
    if (addr < 0x08004000) return FLASH_SECTOR_0;
    if (addr < 0x08008000) return FLASH_SECTOR_1;
    if (addr < 0x0800C000) return FLASH_SECTOR_2;
    if (addr < 0x08010000) return FLASH_SECTOR_3;
    if (addr < 0x08020000) return FLASH_SECTOR_4;
    if (addr < 0x08040000) return FLASH_SECTOR_5;
    if (addr < 0x08060000) return FLASH_SECTOR_6;
    if (addr < 0x08080000) return FLASH_SECTOR_7;
    if (addr < 0x080A0000) return FLASH_SECTOR_8;
    if (addr < 0x080C0000) return FLASH_SECTOR_9;
    if (addr < 0x080E0000) return FLASH_SECTOR_10;
    if (addr < 0x08100000) return FLASH_SECTOR_11;
    return FLASH_SECTOR_11;
}

ota_flash_result_t ota_flash_erase(uint32_t addr, uint32_t size)
{
    uint32_t end_addr = addr + size;
    uint32_t curr_addr = addr;
    
    /* 参数检查 */
    if (addr < APP1_ADDR || (end_addr > (APP2_ADDR + APP2_SIZE))) {
        return OTA_FLASH_ERR_ADDR;
    }
    
    FLASH_EraseInitTypeDef erase_init;
    uint32_t error = 0;
    
    HAL_FLASH_Unlock();
    
    while (curr_addr < end_addr) {
        uint32_t sector = ota_get_sector_index(curr_addr);
        
        erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase_init.Sector = sector;
        erase_init.NbSectors = 1;
        erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        
        if (HAL_FLASHEx_Erase(&erase_init, &error) != HAL_OK) {
            HAL_FLASH_Lock();
            return OTA_FLASH_ERR_ERASE;
        }
        
        curr_addr += SECTOR_SIZE;
    }
    
    HAL_FLASH_Lock();
    return OTA_FLASH_OK;
}

ota_flash_result_t ota_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!buf || addr < APP1_ADDR) {
        return OTA_FLASH_ERR_ADDR;
    }
    
    memcpy(buf, (void *)addr, len);
    return OTA_FLASH_OK;
}

ota_flash_result_t ota_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!buf || addr < APP1_ADDR || len % 4 != 0) {
        return OTA_FLASH_ERR_ADDR;
    }
    
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t data = *(uint32_t *)(buf + i);

        /* 等待Flash空闲 */
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET)
        {
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, data) != HAL_OK) {
            HAL_FLASH_Lock();
            return OTA_FLASH_ERR_WRITE;
        }
    }
    
    HAL_FLASH_Lock();
    
    // 给 Flash 时间完成写入操作，避免立即进行下一次写入时出现冲突
    
    vTaskDelay(5);
    
    return OTA_FLASH_OK;
}

ota_flash_result_t ota_flash_read_app1_header(void *header)
{
    return ota_flash_read(APP1_ADDR, (uint8_t *)header, OTA_HEADER_SIZE);
}

ota_flash_result_t ota_flash_read_app2_header(void *header)
{
    return ota_flash_read(APP2_ADDR, (uint8_t *)header, OTA_HEADER_SIZE);
}

ota_flash_result_t ota_flash_write_app1_header(const void *header)
{
    return ota_flash_write(APP1_ADDR, (const uint8_t *)header, OTA_HEADER_SIZE);
}

ota_flash_result_t ota_flash_write_app2_header(const void *header)
{
    return ota_flash_write(APP2_ADDR, (const uint8_t *)header, OTA_HEADER_SIZE);
}

ota_flash_result_t ota_flash_write_header(uint32_t app_addr, const void *header)  
{
    if (app_addr != APP1_ADDR && app_addr != APP2_ADDR) {
        return OTA_FLASH_ERR_ADDR;
    }
    
    return ota_flash_write(app_addr, (const uint8_t *)header, OTA_HEADER_SIZE);
}

int boot_info_write(const void *info)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase;

    uint32_t sector_error;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;

    erase.Sector = FLASH_SECTOR_11;

    erase.NbSectors = 1;

    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase(
        &erase,
        &sector_error
    );

    uint32_t *p = (uint32_t *)info;

    for (int i = 0; i < sizeof(boot_info_t)/4; i++)
    {
        HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            BOOT_INFO_ADDR + i * 4,
            p[i]
        );
    }

    HAL_FLASH_Lock();

    return 0;
}

void* boot_info_read(void)
{
    return (boot_info_t *)BOOT_INFO_ADDR;
}
