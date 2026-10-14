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
 * 传给 ota_flash_erase_range()/ota_flash_crc32() 的 feed_cb 用这个。
 * 顺带 ota_task_wdt_progress(): 能把狗喂上说明 OTA 任务还在推进,
 * 于是 OTA 任务看门狗的截止时刻也往后顺延。 */
void ota_wdt_feed(void);

/* 启用/停用 TIM7+DMA 硬件自动喂狗。
 * 返回 1 = 启用成功(自检已通过); 0 = 失败, 已回退为只靠软件喂狗。 */
uint8_t ota_wdt_hw_start(void);
void    ota_wdt_hw_stop(void);

/* 硬件喂狗当前是否生效 */
uint8_t ota_wdt_hw_is_active(void);


/* ==================================================================
 *  OTA 任务看门狗 —— "升级任务卡死太久 → 重启通信板"
 *
 *  与上面 SP706S 喂狗是两条独立的链路:
 *    上面那条解决"CPU 被 Flash 总线 stall, 喂不上狗";
 *    这条解决"OTA 任务卡死, 但 App_TaskStart 还在照常喂狗 → 板子永不复位"。
 *
 *  时序(全部用 HAL_GetTick() 的 ms 时基, 与 FreeRTOS tick 无关):
 *
 *    OTA 任务                          App_TaskStart(每 500ms)
 *    ----------                        -----------------------
 *    ota_task_wdt_arm()   ── 之后才判超时
 *    ota_task_wdt_kick()  ── 每轮循环上报"我活着"
 *                                       ota_task_wdt_timed_out()?
 *                                         0 → 照常喂狗
 *                                         1 → 不喂狗 → SP706S 复位板子
 *    ota_task_wdt_busy(长操作窗口) ── 进 Flash 擦除/connect 前声明
 *    ota_task_wdt_progress()       ── 长操作内部报进度(每次喂狗时顺带)
 *
 *  ⚠ arm() 之前一律不判超时: OTA 任务没被创建(例如 xTaskCreate 因堆不足
 *    失败)时不能把板子拖进"复位→重启→再复位"的死循环。
 * ================================================================== */

/* OTA 任务启动时调用一次(第一条语句), 此后才开始判超时 */
void ota_task_wdt_arm(void);

/* OTA 任务每轮循环调用: 上报存活, 并把截止时间恢复到正常窗口 */
void ota_task_wdt_kick(void);

/* 进入"预计最长 max_ms 不返回"的操作前调用(Flash 擦除/校验、TCP connect)。
 * 窗口内不判超时; 超过 max_ms 仍不返回则判卡死。 */
void ota_task_wdt_busy(uint32_t max_ms);

/* 长操作内部报进度: 把截止时间按当前窗口往后推。
 * 每次喂狗(ota_wdt_feed)时顺带调用 —— 只要 Flash 还在一个扇区一个扇区地
 * 干活, 就不算卡死。 */
void ota_task_wdt_progress(void);

/* 给 main.c 的 App_TaskStart 判超时用: 1 = OTA 任务卡死, 应停止喂狗等复位 */
uint8_t ota_task_wdt_timed_out(void);


/* ==================================================================
 *  诊断接口 (v42 临时加, 只为抓"复位码 31"的成因, 只读)
 *
 *  配合 ota_client_stage() 一起看:
 *    report_cnt 不涨 + stage 停在 OTA_STAGE_LOOP   ⇒ 任务被饿死(排不上 CPU)
 *    report_cnt 不涨 + stage 停在某个具体调用     ⇒ 任务卡在那个调用里
 * ================================================================== */

/* 累计上报次数(kick/busy/progress 都算), 只增不减 */
uint32_t ota_task_wdt_report_cnt(void);

/* 距"判卡死"还剩 ms: >0 还来得及; <=0 已判超时 */
int32_t ota_task_wdt_left_ms(void);

/* 1 = OTA 任务已经接管(arm 过了), 0 = 还没(此时 left 无意义) */
uint8_t ota_task_wdt_armed(void);

#endif /* __OTA_WDT_H */
