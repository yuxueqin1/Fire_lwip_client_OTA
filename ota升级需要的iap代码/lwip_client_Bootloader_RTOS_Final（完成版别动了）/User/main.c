/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   FreeRTOS V9.0.0 + STM32 LwIP
  *********************************************************************
  * @attention
  *
  * 实验平台:野火  STM32全系列开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/ 
#include "main.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t Ota_Task_Handle = NULL;/* KEY任务句柄 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */

/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
/* ==================================================================
 *  Flash partition / BootInfo layout
 *
 *  MUST stay byte-for-byte identical to the APP side:
 *      User/ota/ota_config.h   (boot_info_t / APPx_ADDR / APPx_SIZE)
 *
 *    +------------------+ 0x08000000
 *    |  Bootloader      |   32 KB   S0, S1        never updated
 *    +------------------+ 0x08008000
 *    |  APP1  (A slot)  |  480 KB   S2..S7
 *    +------------------+ 0x08080000
 *    |  APP2  (B slot)  |  384 KB   S8..S10
 *    +------------------+ 0x080E0000
 *    |  BootInfo        |  128 KB   S11 (only the first 32 bytes used)
 *    +------------------+ 0x08100000
 *
 *  A/B ping-pong: an upgrade is always written into the *other* slot,
 *  so the running slot is never touched and a failed upgrade simply
 *  falls back to it.
 *
 *  NOTE: APP2_ADDR was 0x08088000 in the earlier layout. It moved to
 *  0x08080000 so that every partition starts on a sector boundary
 *  (0x08088000 is in the middle of sector 8 and cannot be erased
 *  independently of APP1's tail).
 * ================================================================== */
#define BOOT_INFO_ADDR      0x080E0000
#define BOOT_INFO_MAGIC     0x55AA55AA

#define APP1_ADDR           0x08008000
#define APP1_SIZE           (480 * 1024)

#define APP2_ADDR           0x08080000
#define APP2_SIZE           (384 * 1024)

/* SRAM range used to validate an image's initial stack pointer (IRAM1) */
#define SRAM_START_ADDR     0x20000000
#define SRAM_END_ADDR       0x20030000

typedef enum
{
    BOOT_APP1 = 1,
    BOOT_APP2 = 2

} boot_app_t;

/* 8 words - must match boot_info_t in User/ota/ota_config.h exactly */
typedef struct
{
    uint32_t magic;

    uint32_t boot_app;

    uint32_t crc;

    uint32_t size;

    uint32_t version;

    uint32_t reserved[3];

} boot_info_t;

/* 版本号可读格式: 0x01000001 -> "v1.0.0.1"
 * 与 APP 侧 User/ota/ota_config.h 的 FW_VER_FMT/FW_VER_ARG 一致 */
#define FW_VER_FMT      "v%u.%u.%u.%u"
#define FW_VER_ARG(v)   (unsigned)(((uint32_t)(v) >> 24) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >> 16) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >>  8) & 0xFFU), \
                        (unsigned)( (uint32_t)(v)        & 0xFFU)

/* 槽位字母: BOOT_APP1 -> 'A', BOOT_APP2 -> 'B'
 * 日志里写 A/B 不写 1/2, 免得和版本号的数字看混 */
#define SLOT_CHAR(s)    (((s) == BOOT_APP2) ? 'B' : 'A')

typedef void (*pFunction)(void);

/* 延时函数 - 基于SysTick的毫秒延时 */
static void Delay_ms(uint32_t ms)
{
    uint32_t i = 0;
    /* SystemClock_Config() 设置系统时钟为180MHz */
    /* SysTick 时钟 = 180000000 / 8 = 22500000 Hz */
    /* 每毫秒需要22500次循环 (约22.5K) */
    while(ms--)
    {
        i = 22500;
        while(i--);
    }
}


/******************************* 宏定义 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些宏定义。
 */


/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

static void Ota_Task(void* pvParameters);/* Ota_Task任务实现 */
int boot_info_write(const void *info);
void JumpToApplication(uint32_t app_addr);

/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化 
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  /* 开发板硬件初始化 */
  BSP_Init();

  /* 创建AppTaskCreate任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )2, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
  /* 启动任务调度 */           
  if(pdPASS == xReturn)
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  else
    return -1;  
  
  while(1);   /* 正常不会执行到这里 */    
}


/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
  
  taskENTER_CRITICAL();           //进入临界区

  /* 创建Ota_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Ota_Task,  /* 任务入口函数 */
                        (const char*    )"Ota_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&Ota_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("Create Ota_Task sucess...\n\n");

  taskEXIT_CRITICAL();            //退出临界区

  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
}



static void Startup_LED_Running(void)
{
      LED_RUN_ON;    HAL_Delay(80);  LED_RUN_OFF;
      LED_485_6_ON;  HAL_Delay(80);  LED_485_6_OFF;
      LED_485_5_ON;  HAL_Delay(80);  LED_485_5_OFF;
      LED_485_4_ON;  HAL_Delay(80);  LED_485_4_OFF;
      LED_485_3_ON;  HAL_Delay(80);  LED_485_3_OFF;
      LED_485_2_ON;  HAL_Delay(80);  LED_485_2_OFF;
      LED_485_1_ON;  HAL_Delay(80);  LED_485_1_OFF;

}

/**********************************************************************
  * @ 函数名  ： Ota_Task
  * @ 功能说明： Ota_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
/* ==================================================================
 *  Image validation (mirrors the APP side ota_flash_check_image)
 *
 *  Return 0 = bootable, negative = not bootable:
 *    -1 : initial stack pointer outside SRAM
 *    -2 : reset vector outside this partition  <- catches a wrong-slot image
 *         (e.g. the A image pushed into the B slot: its reset vector
 *          points into 0x08008xxx which is not inside APP2)
 *    -3 : size illegal, or CRC32 mismatch
 *
 *  img_size == 0 means "unknown" (e.g. flashed with J-Link, never went
 *  through OTA) -> only the two structural checks above are applied.
 *
 *  CRC32 must stay identical to the APP side (User/ota/ota_crc32.c) and
 *  to Python zlib.crc32: reflected, poly 0xEDB88320, init/final 0xFFFFFFFF.
 *  Table-less on purpose - the bootloader only has 32 KB of Flash.
 * ================================================================== */
static uint32_t crc32_flash(uint32_t addr, uint32_t size)
{
    const uint8_t *p = (const uint8_t *)addr;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint32_t b;

    for (i = 0; i < size; i++)
    {
        crc ^= (uint32_t)p[i];

        for (b = 0; b < 8; b++)
        {
            if ((crc & 1u) != 0u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFu;
}


static int app_image_check(uint32_t app_addr, uint32_t app_size,
                           uint32_t img_size, uint32_t expect_crc)
{
    uint32_t sp;
    uint32_t rv;

    /* word 0 = initial stack pointer */
    sp = *(__IO uint32_t *)app_addr;
    if ((sp < SRAM_START_ADDR) || (sp > SRAM_END_ADDR))
    {
        return -1;
    }

    /* word 1 = reset vector, must live inside the partition being booted */
    rv = *(__IO uint32_t *)(app_addr + 4);
    if ((rv < app_addr) || (rv >= (app_addr + app_size)))
    {
        return -2;
    }

    if (img_size != 0u)
    {
        if (img_size > app_size)
        {
            return -3;
        }

        if (crc32_flash(app_addr, img_size) != expect_crc)
        {
            return -3;
        }
    }

    return 0;
}


static const char *check_str(int rc)
{
    switch (rc)
    {
        case  0: return "OK";
        case -1: return "bad stack pointer";
        case -2: return "reset vector outside slot";
        case -3: return "size/CRC mismatch";
        default: return "unknown error";
    }
}


static void boot_slot(uint32_t slot)
{
    if (slot == BOOT_APP2)
    {
        JumpToApplication(APP2_ADDR);
    }
    else
    {
        JumpToApplication(APP1_ADDR);
    }
}


/**********************************************************************
  * @brief   Ota_Task
  * @note    Pick a bootable slot and jump to it.
  *
  *          Order of decisions:
  *            1. BootInfo invalid  -> persist a default (APP1)
  *            2. Validate the requested slot (stack ptr / reset vector / CRC)
  *            3. If it is not bootable, try the other slot and re-point
  *               BootInfo at it (rollback after a failed OTA)
  *            4. If neither slot is bootable, blink forever
  ********************************************************************/
static void Ota_Task(void* parameter)
{
  boot_info_t new_info;
  boot_info_t local;
  const boot_info_t *flash_info = (const boot_info_t *)BOOT_INFO_ADDR;
  uint32_t want;
  uint32_t want_addr;
  uint32_t want_size;
  uint32_t alt;
  int rc;

    (void)parameter;

    /* Work on a RAM copy: BootInfo lives in Flash and gets erased by
     * boot_info_write(), so never hand it a pointer into Flash. */
    local = *flash_info;

    printf("\r\n===== OTA Bootloader =====\r\n");

    if (local.magic == BOOT_INFO_MAGIC)
    {
        printf("[IAP] boot_info: boot_app=%c, ver=" FW_VER_FMT
               ", size=%lu, crc=0x%08lX\r\n",
               SLOT_CHAR(local.boot_app), FW_VER_ARG(local.version),
               (unsigned long)local.size, (unsigned long)local.crc);
    }
    else
    {
        printf("[IAP] boot_info invalid (magic=0x%08lX)\r\n",
               (unsigned long)local.magic);
    }

    if (local.magic != BOOT_INFO_MAGIC)
    {
        /* First power-up (erased chip): default to APP1 and persist it */

        new_info.magic       = BOOT_INFO_MAGIC;
        new_info.boot_app    = BOOT_APP1;
        new_info.crc         = 0;
        new_info.size        = 0;
        new_info.version     = 0;
        new_info.reserved[0] = 0;
        new_info.reserved[1] = 0;
        new_info.reserved[2] = 0;

        boot_info_write(&new_info);

        local = new_info;

        printf("[IAP] boot_info missing, default slot A written\r\n");


        LED_485_ALL_ON;
        Delay_ms(300);
        LED_485_ALL_OFF;
    }

    /* ---- 1. which slot did BootInfo ask for ---- */
    if (local.boot_app == BOOT_APP2)
    {
        want = BOOT_APP2;
    }
    else
    {
        want = BOOT_APP1;
    }

    want_addr = (want == BOOT_APP1) ? APP1_ADDR : APP2_ADDR;
    want_size = (want == BOOT_APP1) ? APP1_SIZE : APP2_SIZE;

    printf("[IAP] boot_info asks for slot %c (0x%08lX, %luKB)\r\n",
           SLOT_CHAR(want), (unsigned long)want_addr,
           (unsigned long)(want_size / 1024u));

    /* ---- 2. is it bootable ---- */
    rc = app_image_check(want_addr, want_size, local.size, local.crc);

    printf("[IAP] slot %c image check: %s (rc=%d)\r\n",
           SLOT_CHAR(want), check_str(rc), rc);

    if (rc == 0)
    {
        if (want == BOOT_APP1)
        {
            LED_RUN_ON;
            Delay_ms(3000);
            LED_RUN_OFF;
        }
        else
        {
            LED_485_2_ON;
            Delay_ms(2000);
            LED_485_2_OFF;
        }

        printf("[IAP] jumping to slot %c @ 0x%08lX ...\r\n",
               SLOT_CHAR(want), (unsigned long)want_addr);

        boot_slot(want);        /* does not return */
    }

    /* ---- 3. rollback: try the other slot ----
     * BootInfo only carries size/crc for one slot, so the fallback is
     * validated structurally (stack pointer + reset vector) only. */
    alt = (want == BOOT_APP1) ? BOOT_APP2 : BOOT_APP1;

    printf("[IAP] slot %c not bootable, try slot %c\r\n",
           SLOT_CHAR(want), SLOT_CHAR(alt));

    if (app_image_check((alt == BOOT_APP1) ? APP1_ADDR : APP2_ADDR,
                        (alt == BOOT_APP1) ? APP1_SIZE : APP2_SIZE,
                        0u, 0u) == 0)
    {
        boot_info_t fixed;

        /* Re-point BootInfo so the next boot goes straight there */
        fixed            = local;
        fixed.magic      = BOOT_INFO_MAGIC;
        fixed.boot_app   = alt;
        fixed.crc        = 0;
        fixed.size       = 0;
        fixed.version    = 0;
        fixed.reserved[0] = 0;
        fixed.reserved[1] = 0;
        fixed.reserved[2] = 0;

        boot_info_write(&fixed);

        LED_485_3_ON;
        Delay_ms(1000);
        LED_485_3_OFF;
        Delay_ms(1000);

        printf("[IAP] rollback: jumping to slot %c @ 0x%08lX ...\r\n",
               SLOT_CHAR(alt),
               (unsigned long)((alt == BOOT_APP1) ? APP1_ADDR : APP2_ADDR));

        boot_slot(alt);         /* does not return */
    }

    /* ---- 4. neither slot is usable ----
     * Blink all 485 LEDs forever.  PF10 (WDI) is deliberately left
     * floating: the SP706S watchdog is disabled while WDI is not driven,
     * so this loop will not be interrupted by a reset.  The board is only
     * recoverable through the debug port from here. */
    while (1)
    {
        printf("[IAP] FATAL: neither slot bootable, blink forever\r\n");
        Startup_LED_Running();
    }
}


int boot_info_write(const void *info)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error;
    uint32_t *p = (uint32_t *)info;

    HAL_FLASH_Unlock();

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;

    erase.Sector = FLASH_SECTOR_11;

    erase.NbSectors = 1;

    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASHEx_Erase(
        &erase,
        &sector_error
    );

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



void JumpToApplication(uint32_t app_address)
{
    uint32_t app_stack;
    uint32_t app_reset_handler;

    pFunction JumpToApp;

   

    /* 获取APP栈顶地址 */
    app_stack = *(__IO uint32_t*)app_address;

    /* 检查栈地址是否合法 */
    if ((app_stack < 0x20000000) ||(app_stack > 0x20030000))   //256KB SRAM
    
    {
        /* 非法APP */
        while (1);
    }

    /* 获取APP复位中断地址 */
    app_reset_handler =
        *(__IO uint32_t*)(app_address + 4);

    /* 关闭全局中断 */
    __disable_irq();

    /* 关闭SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* 关闭所有中断 */
    for (uint8_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* 关闭Fault异常屏蔽 */
    __set_FAULTMASK(0);

    /* HAL反初始化 */
    HAL_DeInit();

    /* 时钟恢复默认 */
    HAL_RCC_DeInit();

    /* 停止调度器Tick */
    /* 防止FreeRTOS PendSV/SysTick残留 */
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk;

    /* 切换到MSP */
    __set_CONTROL(0);

    /* 设置主栈指针 */
    __set_MSP(app_stack);

    /* 重定位中断向量表 */
    SCB->VTOR = app_address;

    /* 数据同步 */
    __DSB();
    __ISB();
    /* 开总中断: PRIMASK 是 CPU 内核寄存器, 跳转到 APP 不会把它清零,
     * 上面 __disable_irq() 关掉之后必须在这里放开 —— 否则 APP 里
     * SysTick 中断永远进不来, HAL_Delay() 会死等 uwTick 不返回,
     * 症状是 APP 卡死 / LED 不闪 / 串口连初始化都到不了。 */
    __enable_irq();


    /* 跳转到APP */
    JumpToApp = (pFunction)app_reset_handler;

    JumpToApp();

    /* 不应该执行到这里 */
    while (1);
}


