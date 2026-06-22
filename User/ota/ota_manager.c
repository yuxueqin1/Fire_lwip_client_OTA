#include "ota_manager.h"
#include "ota_flash.h"
#include "ota_crc32.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

static ota_manager_t g_ota_manager = {
    .state = OTA_STATE_IDLE,
    .downloaded_size = 0,
    .total_size = 0,
    .download_crc = 0xFFFFFFFF,
    .remote_crc = 0,
    .remote_version = 0
};

static uint32_t g_current_app_addr;
static uint32_t g_download_addr;
static uint32_t g_download_size;

/* 处理服务器的升级推送请求 */
int32_t ota_handle_push_update_request(uint32_t new_version, uint32_t new_size, uint32_t new_crc)
{
    uint32_t curr_version = ota_get_current_version();
    
    printf("[OTA] Received update request from server\n");
    printf("[OTA] Current version: 0x%08lX\n", curr_version);
    printf("[OTA] New version:     0x%08X\n", new_version);
    printf("[OTA] New size:        %u bytes\n", new_size);
    printf("[OTA] New CRC32:       0x%08X\n", new_crc);
    
    // 检查大小是否超限
    if (new_size > (g_download_size - OTA_HEADER_SIZE)) {
        printf("[OTA] Error: Firmware too large (%u > %u)\n", new_size, g_download_size - OTA_HEADER_SIZE);
        g_ota_manager.state = OTA_STATE_ERROR_SIZE;
        return g_ota_manager.state;
        // return -1;
    }
    
    // 对比版本：只有新版本更大才升级
    if (new_version <= curr_version) {
        printf("[OTA] Info: Already running this version or newer\n");
        return 0;  // 不需要升级
    }
    
    printf("[OTA] Upgrade available, preparing download\n");
    
    g_ota_manager.state = OTA_STATE_REQUEST_RECEIVED;
    g_ota_manager.remote_version = new_version;
    g_ota_manager.remote_crc = new_crc;
    g_ota_manager.total_size = new_size;
    
    return 1;  // 版本更新，同意升级
}

void ota_manager_init(void)
{
    boot_info_t *current_boot_info;

    ota_flash_init();

    memset(&g_ota_manager, 0, sizeof(ota_manager_t));
    g_ota_manager.state = OTA_STATE_IDLE;
    g_ota_manager.download_crc = 0xFFFFFFFF;

    current_boot_info = boot_info_read();

    // if (current_boot_info == NULL) {
    // return ERROR;
    // }

    if (current_boot_info->magic == BOOT_INFO_MAGIC) {
        if (current_boot_info->boot_app == BOOT_APP1) 
        {
            g_current_app_addr = APP1_ADDR;
            g_download_addr = APP2_ADDR;
            g_download_size = APP2_SIZE;
        } else {
            g_current_app_addr = APP2_ADDR;
            g_download_addr = APP1_ADDR;
            g_download_size = APP1_SIZE;
        }
    } else {
        // 默认运行APP1
        g_current_app_addr = APP1_ADDR;
        g_download_addr = APP2_ADDR;
        g_download_size = APP2_SIZE;
    }

}

ota_manager_t* ota_get_manager(void)
{
    return &g_ota_manager;
}

uint32_t ota_get_current_version(void)
{
    ota_app_header_t header;
    if (ota_flash_read_app1_header(&header) != OTA_FLASH_OK) {
        return 0;
    }
    
    if (header.magic != OTA_MAGIC) {
        return FW_VERSION;  // 返回当前编译的版本
    }
    
    return header.version;
}

ota_state_t ota_get_state(void)
{
    return g_ota_manager.state;
}

uint32_t ota_get_progress(void)
{
    if (g_ota_manager.total_size == 0) {
        return 0;
    }
    return (g_ota_manager.downloaded_size * 100) / g_ota_manager.total_size;
}

int32_t ota_start_download(const char *server_ip, uint16_t server_port)
{
    if (g_ota_manager.state != OTA_STATE_IDLE) {
        printf("[OTA] Error: OTA in progress or failed\n");
        return -1;
    }
    
    g_ota_manager.state = OTA_STATE_DOWNLOADING;
    g_ota_manager.downloaded_size = 0;
    // 注意：不重置 total_size，它应该在 ota_handle_push_update_request() 中设置
    g_ota_manager.download_crc = 0xFFFFFFFF;
    
    // 擦除App区域准备接收新固件
    printf("[OTA] Erasing download area (0x%08lX)...\n", g_download_addr);
    if (ota_flash_erase(g_download_addr, g_download_size) != OTA_FLASH_OK) {
        printf("[OTA] Error: Failed to erase 0x%08lX\n", g_download_addr);
        g_ota_manager.state = OTA_STATE_FAILED;
        return -1;
    }
    
    printf("[OTA] Download area erased successfully\n");
    printf("[OTA] Ready for firmware download (expected size: %lu bytes)\n", g_ota_manager.total_size);
    
    return 0;
}

int32_t ota_write_data_block(const uint8_t *data, uint32_t len, uint32_t offset)
{
    if (g_ota_manager.state != OTA_STATE_DOWNLOADING) {
        printf("[OTA] ERROR: Not in DOWNLOADING state (state=%u)\n", g_ota_manager.state);
        return -1;
    }
    
    uint32_t write_addr = g_download_addr + OTA_HEADER_SIZE + offset;
    
    printf("[OTA] Writing block: offset=%lu, len=%lu, addr=0x%08lX\n", offset, len, write_addr);
    
    // 检查写入范围
    if ((write_addr + len) > (g_download_addr + g_download_size)) {
        printf("[OTA] Error: Write overflow (write would exceed 0x%08lX)\n", (uint32_t)(g_download_addr + g_download_size));
        g_ota_manager.state = OTA_STATE_FAILED;
        return -1;
    }
    
    // 写入数据到Flash
    if (ota_flash_write(write_addr, data, len) != OTA_FLASH_OK) {
        printf("[OTA] Error: Failed to write Flash\n");
        g_ota_manager.state = OTA_STATE_FAILED;
        return -1;
    }
    
    // 更新CRC32
    crc32_update(&g_ota_manager.download_crc, data, len);
    
    g_ota_manager.downloaded_size += len;
    
    uint32_t progress = (g_ota_manager.total_size > 0) ? ((g_ota_manager.downloaded_size * 100) / g_ota_manager.total_size) : 0;
    printf("[OTA] Block written OK. Progress: %lu/%lu bytes (%lu%%)\n", 
           g_ota_manager.downloaded_size, g_ota_manager.total_size, progress);
    
    return 0;
}

int32_t ota_finish_download(void)
{
    if (g_ota_manager.state != OTA_STATE_DOWNLOADING) {
        return -1;
    }
    
    g_ota_manager.state = OTA_STATE_VERIFYING;
    
    // 完成CRC32计算
    g_ota_manager.download_crc = crc32_finalize(g_ota_manager.download_crc);
    
    printf("[OTA] Download finished. Total: %lu bytes\n", g_ota_manager.downloaded_size);
    printf("[OTA] Downloaded CRC32: 0x%08lX\n", g_ota_manager.download_crc);
    printf("[OTA] Expected CRC32: 0x%08X\n", g_ota_manager.remote_crc);
    
    // 验证CRC32
    if (g_ota_manager.download_crc != g_ota_manager.remote_crc) {
        printf("[OTA] Error: CRC32 mismatch!\n");
        g_ota_manager.state = OTA_STATE_FAILED;
        return -1;
    }
    
    printf("[OTA] Verification passed!\n");
    g_ota_manager.state = OTA_STATE_SUCCESS;
    
    return 0;
}

int32_t ota_prepare_update(void)
{
    boot_info_t new_boot_info;

    new_boot_info.magic = BOOT_INFO_MAGIC;

    if (g_ota_manager.state != OTA_STATE_SUCCESS) {
        printf("[OTA] Error: Download not successful\n");
        return -1;
    }
    
    g_ota_manager.state = OTA_STATE_UPDATING;
    
    // 准备App头信息
    ota_app_header_t new_header = {
        .magic = OTA_MAGIC,
        .version = g_ota_manager.remote_version,
        .size = g_ota_manager.downloaded_size,
        .crc32 = g_ota_manager.download_crc,
        .timestamp = 0  // 可以添加时间戳
    };
    
    // 写入新固件头
    if (ota_flash_write_header(g_download_addr, &new_header) != OTA_FLASH_OK) {
        printf("[OTA] Error: Failed to write App header\n");
        return -1;
    }
    
    printf("[OTA] Update prepared. Device will reboot to apply new firmware.\n");
    printf("[OTA] New version: %lu\n", new_header.version);
    
    // 设置标志位，告诉Bootloader进行固件切换
    /* 当前运行APP1 */
    if (g_current_app_addr == APP1_ADDR)
    {
        new_boot_info.boot_app = BOOT_APP2;

        printf("[OTA] Next boot -> APP2\r\n");
    }
    else
    {
        new_boot_info.boot_app = BOOT_APP1;

        printf("[OTA] Next boot -> APP1\r\n");
    }

    new_boot_info.crc = 0;

    new_boot_info.reserved = 0;

    /* 写入BootInfo */
    if (boot_info_write(&new_boot_info) != 0)
    {
        printf("[OTA] Failed to write BootInfo\r\n");

        return -1;
    }

    printf("[OTA] BootInfo updated successfully\r\n");
    
    return 0;
}
