#include "ota_wdt.h"
#include "ota_config.h"     /* OTA_TASK_WDT_TIMEOUT_MS 等超时常数 */
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 板级喂狗函数定义在 User/board.c:647, 原型在 User/main.h:62。
 * 这里用 extern 而不是 #include "main.h", 避免把 Modbus / FreeRTOS
 * 一大串头文件拖进 OTA 模块。 */
extern void WatchDog_Clear(void);


/* ==================================================================
 *  WDI 引脚 —— ⚠ 必须与 User/board.c:34-36 的 WDI_PORT / WDI_PIN 一致
 * ================================================================== */
#define OTA_WDI_PORT            GPIOF
#define OTA_WDI_PIN             GPIO_PIN_10
#define OTA_WDI_BSRR_SET        (1UL << 10U)            /* PF10 置 1 */
#define OTA_WDI_BSRR_RESET      (1UL << (10U + 16U))    /* PF10 置 0 */
#define OTA_WDI_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()

/* ==================================================================
 *  DMA 请求映射 (RM0090 "DMA1 request mapping"):
 *      TIM7_UP  →  DMA1 Stream2 / Channel 1
 *
 *  ⚠ 这张映射表与芯片型号/系列相关, 无法在编译期校验, 所以
 *    ota_wdt_hw_start() 是"先起硬件再自检", 自检不过就自动回退软件喂狗。
 *    若现场打印"自检失败", 只需按 RM0090 改下面两个宏, 其余代码不用动。
 *
 *  为什么不用 TIM6: 本工程 TIM6 已被 FreeModbus (mbport/mbporttimer.c)
 *  用作 T3.5 超时定时器, 占用了更新中断。
 * ================================================================== */
#define OTA_WDT_DMA_STREAM      DMA1_Stream2
#define OTA_WDT_DMA_CHANNEL     DMA_CHANNEL_1

#define OTA_WDT_TOGGLE_MS       100U        /* PF10 每 100ms 翻转一次 */
#define OTA_WDT_TICK_HZ         10000U      /* TIM7 计数频率 10kHz */
#define OTA_WDT_SELF_CHECK_N    6U          /* 自检采样次数 (6 x 50ms = 300ms) */
#define OTA_WDT_SELF_CHECK_MS   50U


static uint8_t  g_dwt_ok    = 0U;   /* DWT 周期计数器可用 */
static uint8_t  g_hw_active = 0U;   /* 硬件喂狗已启用 */

static DMA_HandleTypeDef g_hdma_wdt;
static TIM_HandleTypeDef g_htim_wdt;

/* DMA 搬运的 2 个字: 依次写进 GPIOF->BSRR, 循环执行
 *   → 先置位 PF10, 再复位 PF10, 每 2 个更新事件完成一次翻转。
 *
 * ⚠ 必须放在 SRAM (当前 scatter 的 RW_IRAM1 @0x20000000)。
 *   不能写成 const: const 会落到 Flash, 而擦除期间 Flash 总线被占用,
 *   DMA 从 Flash 取源数据同样会被 stall —— 那正是要解决的问题。
 *   当前 scatter 只有 RW_IRAM1, 不会落到 CCM(0x10000000, DMA1 不可达)。 */
static uint32_t g_wdi_pattern[2] = { OTA_WDI_BSRR_SET, OTA_WDI_BSRR_RESET };


/* ==================================================================
 *  计时: DWT 周期计数器
 *  自检需要"等几十毫秒再看一眼引脚", 而这时不能依赖 SysTick 中断
 *  (擦除前虽然能用, 但为了不引入任何中断依赖, 直接用 CPU 周期计数)。
 * ================================================================== */
static void dwt_probe(void)
{
    uint32_t i;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    for (i = 0U; i < 2000U; i++) {
        __NOP();
    }

    g_dwt_ok = (DWT->CYCCNT != 0U) ? 1U : 0U;

    if (g_dwt_ok == 0U) {
        DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
        printf("[WDT] warn: DWT->CYCCNT unavailable, skip hw kick self-test\r\n");
    }
}

/* 忙等 ms 毫秒; 返回 1 = 正常, 0 = 计时不可用(已超时保护退出) */
static uint8_t dwt_delay_ms(uint32_t ms)
{
    uint32_t cycles;
    uint32_t start;
    uint32_t guard = 0U;

    if (g_dwt_ok == 0U) {
        return 0U;
    }

    cycles = ms * (SystemCoreClock / 1000U);
    start  = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles) {
        if (++guard > 50000000U) {      /* CYCCNT 不走时防止死等 */
            g_dwt_ok = 0U;
            return 0U;
        }
    }
    return 1U;
}


/* ==================================================================
 *  APB1 定时器时钟
 *  APB1 预分频 = 1 时 定时器时钟 = PCLK1, 否则 = PCLK1 x 2
 *  (本板 180MHz: PCLK1 = 45MHz → TIM7 时钟 = 90MHz)
 * ================================================================== */
static uint32_t rcc_tim_apb1_clk(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;

    /* PPRE1 编码: 0xx = 不分频, 1xx = 2/4/8/16 分频 */
    return ((ppre1 & 0x04U) == 0U) ? pclk1 : (pclk1 * 2U);
}


/* ==================================================================
 *  硬件搭建 / 拆除
 * ================================================================== */
static uint8_t wdt_hw_setup(void)
{
    GPIO_InitTypeDef gpio;
    uint32_t tim_clk;
    uint32_t psc;
    uint32_t arr;

    /* ---- 1. WDI 引脚: 推挽输出(与 board.c 的 WDI_GPIO_Init 一致) ---- */
    OTA_WDI_CLK_ENABLE();

    gpio.Pin   = OTA_WDI_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTA_WDI_PORT, &gpio);
    HAL_GPIO_WritePin(OTA_WDI_PORT, OTA_WDI_PIN, GPIO_PIN_RESET);

    /* ---- 2. DMA: MEM -> GPIOF->BSRR, 循环模式 ---- */
    __HAL_RCC_DMA1_CLK_ENABLE();

    g_hdma_wdt.Instance                 = OTA_WDT_DMA_STREAM;
    g_hdma_wdt.Init.Channel             = OTA_WDT_DMA_CHANNEL;
    g_hdma_wdt.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    g_hdma_wdt.Init.PeriphInc           = DMA_PINC_DISABLE;   /* BSRR 固定地址 */
    g_hdma_wdt.Init.MemInc              = DMA_MINC_ENABLE;    /* 2 个字轮流搬 */
    g_hdma_wdt.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_hdma_wdt.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    g_hdma_wdt.Init.Mode                = DMA_CIRCULAR;
    g_hdma_wdt.Init.Priority            = DMA_PRIORITY_HIGH;
    g_hdma_wdt.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&g_hdma_wdt) != HAL_OK) {
        printf("[WDT] DMA Init failed\r\n");
        return 0U;
    }

    /* 非中断版本: 不打开 NVIC / 不打开 TC 中断 —— 全程无中断依赖 */
    if (HAL_DMA_Start(&g_hdma_wdt, (uint32_t)g_wdi_pattern,
                      (uint32_t)&OTA_WDI_PORT->BSRR, 2U) != HAL_OK) {
        printf("[WDT] DMA Start failed\r\n");
        return 0U;
    }

    /* ---- 3. TIM7: 每 OTA_WDT_TOGGLE_MS 产生一次更新事件 ---- */
    tim_clk = rcc_tim_apb1_clk();
    psc     = tim_clk / OTA_WDT_TICK_HZ;

    if (psc == 0U) {
        psc = 1U;
    }
    if (psc > 65536U) {         /* TIM7 的 PSC 是 16 位 */
        printf("[WDT] timer clock %luHz too high, adjust OTA_WDT_TICK_HZ\r\n",
               (unsigned long)tim_clk);
        return 0U;
    }

    arr = (OTA_WDT_TICK_HZ / 1000U) * OTA_WDT_TOGGLE_MS;    /* 10 x 100 = 1000 */

    __HAL_RCC_TIM7_CLK_ENABLE();

    g_htim_wdt.Instance               = TIM7;
    g_htim_wdt.Init.Prescaler         = psc - 1U;
    g_htim_wdt.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g_htim_wdt.Init.Period            = arr - 1U;
    g_htim_wdt.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g_htim_wdt.Init.RepetitionCounter = 0U;

    if (HAL_TIM_Base_Init(&g_htim_wdt) != HAL_OK) {
        printf("[WDT] TIM Base Init failed\r\n");
        return 0U;
    }

    /* 打开 TIM7 的更新 DMA 请求(UDE), 再启动计数 */
    __HAL_TIM_ENABLE_DMA(&g_htim_wdt, TIM_DMA_UPDATE);

    if (HAL_TIM_Base_Start(&g_htim_wdt) != HAL_OK) {
        printf("[WDT] TIM Base Start failed\r\n");
        return 0U;
    }

    return 1U;
}

static void wdt_hw_teardown(void)
{
    HAL_TIM_Base_Stop(&g_htim_wdt);
    __HAL_TIM_DISABLE_DMA(&g_htim_wdt, TIM_DMA_UPDATE);
    (void)HAL_DMA_Abort(&g_hdma_wdt);
    (void)HAL_DMA_DeInit(&g_hdma_wdt);
    (void)HAL_TIM_Base_DeInit(&g_htim_wdt);

    g_hw_active = 0U;
}


/* ==================================================================
 *  对外接口
 * ================================================================== */
void ota_wdt_init(void)
{
    dwt_probe();
    HAL_GPIO_WritePin(OTA_WDI_PORT, OTA_WDI_PIN, GPIO_PIN_RESET);
}

void ota_wdt_feed(void)
{
    /* 硬件喂狗生效期间这里也照常翻转: 多一个边沿只会更安全,
     * 而且万一 DMA 中途出问题, 软件喂狗仍然在兜底。 */
    WatchDog_Clear();

    /* 这里只可能是 OTA 任务在 Flash 擦除/校验过程中调用(见 ota_flash 的
     * feed_cb), 所以顺带上报一次"长操作还在推进" —— 一个扇区一个扇区地
     * 干活就不算卡死, 但整个窗口内一次都没喂上就会判超时。 */
    ota_task_wdt_progress();
}

uint8_t ota_wdt_hw_start(void)
{
    uint32_t seen = 0U;
    uint8_t  i;

    if (g_hw_active != 0U) {
        return 1U;
    }

    if (wdt_hw_setup() == 0U) {
        printf("[WDT] TIM7+DMA hw kick setup failed, fallback to sw kick\r\n");
        wdt_hw_teardown();
        return 0U;
    }

    /* 自检: 在 300ms 内采样 PF10 的实际输出电平, 应该同时看到高和低。
     * 这是"DMA 确实在往 BSRR 写"的直接证据, 同时也验证了
     * TIM7_UP → DMA1 Stream2/Channel1 这个请求映射在本板是对的。 */
    for (i = 0U; i < OTA_WDT_SELF_CHECK_N; i++) {
        seen |= ((OTA_WDI_PORT->ODR & OTA_WDI_PIN) != 0U) ? 0x2U : 0x1U;
        if (dwt_delay_ms(OTA_WDT_SELF_CHECK_MS) == 0U) {
            break;
        }
    }

    /* bit0 = 见过低电平, bit1 = 见过高电平; 两者都有才算通过 */
    if (seen != 0x3U) {
        printf("[WDT] hw kick self-test failed (ODR=0x%lX), fallback to sw kick\r\n",
               (unsigned long)seen);
        wdt_hw_teardown();
        return 0U;
    }

    g_hw_active = 1U;
    printf("[WDT] TIM7+DMA hw kick enabled (PF10 toggles every %lums)\r\n",
           (unsigned long)OTA_WDT_TOGGLE_MS);
    return 1U;
}

void ota_wdt_hw_stop(void)
{
    if (g_hw_active == 0U) {
        return;
    }

    wdt_hw_teardown();
    printf("[WDT] hw kick disabled, sw kick resumed\r\n");
}

uint8_t ota_wdt_hw_is_active(void)
{
    return g_hw_active;
}


/* ==================================================================
 *  OTA 任务看门狗 (见 ota_wdt.h 的时序说明)
 *
 *  只有一个"截止时刻"需要比较, 三种上报方式都只是把它往后推:
 *      kick()   : 截止 = now + 正常窗口(30s)
 *      busy(ms) : 截止 = now + ms        ← 进长操作前
 *      progress(): 截止 = now + 当前窗口  ← 长操作里报进度
 *
 *  时基用 HAL_GetTick()(1ms, SysTick 累加): 与 FreeRTOS 调度无关,
 *  所以"OTA 任务被更高优先级任务饿死"这种情况也能被查出来。
 *  比较用有符号差值, 32 位 ms 计数回绕(约 49.7 天)时仍然正确。
 * ================================================================== */
static volatile uint32_t g_ota_task_deadline_ms = 0U;   /* 允许的最晚上报时刻 */
static volatile uint32_t g_ota_task_window_ms   = 0U;   /* 当前窗口长度(progress 按它顺延) */
static volatile uint8_t  g_ota_task_armed       = 0U;   /* 0 = 任务还没起来, 不判超时 */

/* 内部: 把截止时刻设为 now + ms */
static void ota_task_wdt_set_window(uint32_t ms)
{
    g_ota_task_window_ms   = ms;
    g_ota_task_deadline_ms = HAL_GetTick() + ms;
}

void ota_task_wdt_arm(void)
{
    g_ota_task_armed = 1U;
    ota_task_wdt_set_window(OTA_TASK_WDT_TIMEOUT_MS);

    printf("[WDT] ota task wdt armed (timeout %lums)\r\n",
           (unsigned long)OTA_TASK_WDT_TIMEOUT_MS);
}

void ota_task_wdt_kick(void)
{
    if (g_ota_task_armed == 0U) {
        return;
    }
    ota_task_wdt_set_window(OTA_TASK_WDT_TIMEOUT_MS);
}

void ota_task_wdt_busy(uint32_t max_ms)
{
    if (g_ota_task_armed == 0U) {
        return;
    }
    ota_task_wdt_set_window(max_ms);
}

void ota_task_wdt_progress(void)
{
    if (g_ota_task_armed == 0U) {
        return;
    }
    /* 按当前窗口顺延: 擦除期间窗口 = OTA_TASK_BUSY_FLASH_MS */
    g_ota_task_deadline_ms = HAL_GetTick() + g_ota_task_window_ms;
}

uint8_t ota_task_wdt_timed_out(void)
{
    if (g_ota_task_armed == 0U) {
        return 0U;
    }
    /* (int32_t 差值 >= 0) ⇒ now 已过截止时刻; 回绕安全 */
    return ((int32_t)(HAL_GetTick() - g_ota_task_deadline_ms) >= 0) ? 1U : 0U;
}
