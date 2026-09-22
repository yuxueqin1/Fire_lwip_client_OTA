#ifndef __OTA_WDT_H
#define __OTA_WDT_H

#include <stdint.h>

/* ==================================================================
 *  外部看门狗 (SP706S) 喂狗
 *
 *  背景:
 *    STM32F4 内部 Flash 在擦除/编程期间, 总线被 Flash 控制器占用,
 *    从 Flash 取指的 CPU 会被 stall —— 连中断都进不来。
 *    擦除一个 128KB 扇区典型 1.1s(最坏可达数秒), 而 SP706S 的
 *    超时只有 1.6s, 所以"擦除中途喂不上狗 → 复位 → 升级失败"。
 *
 *  解决:
 *    ota_wdt_hw_start() 让 TIM7 每 100ms 触发一次 DMA 传输,
 *    DMA 直接把 2 个字(SET/RESET)轮流写进 GPIOF->BSRR,
 *    从而在 CPU 完全停摆的情况下由硬件自动翻转 PF10 喂狗。
 *    整个过程不需要 CPU 参与, 也不需要任何中断。
 *
 *  用法(必须成对, 且只包住真正会 stall 的短窗口):
 *      ota_wdt_hw_start();
 *      ota_flash_erase_range(...);      // 或 ota_flash_crc32(...)
 *      ota_wdt_hw_stop();
 *
 *  ⚠ 不要把 hw 喂狗常开: 那会让"任务卡死 → 看门狗复位"这条
 *    最后的保护失效。
 * ================================================================== */

/* 上电初始化(探测 DWT 周期计数器是否可用, 打开 DMA1 时钟)。
 * 在 ota_manager_init() 中调用。 */
void ota_wdt_init(void);

/* 一次软件喂狗 —— 就是板级的 WatchDog_Clear()(翻转 PF10)。
 * 传给 ota_flash_erase_range()/ota_flash_crc32() 的 feed_cb 用这个。 */
void ota_wdt_feed(void);

/* 启用/停用 TIM7+DMA 硬件自动喂狗。
 * 返回 1 = 启用成功(自检已通过); 0 = 失败, 已回退为只靠软件喂狗。 */
uint8_t ota_wdt_hw_start(void);
void    ota_wdt_hw_stop(void);

/* 硬件喂狗当前是否生效 */
uint8_t ota_wdt_hw_is_active(void);

#endif /* __OTA_WDT_H */
