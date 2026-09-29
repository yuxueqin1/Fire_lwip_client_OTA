/*
 * MODBUS Library: STM32F4 porting layer for RS485 serial.
 * Target MCU: STM32F429IGT6
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "stm32f4xx_hal.h"
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbportlayer.h"
#include "mbtypes.h"
#include "mbframe.h"
#include "mbutils.h"

#define MBP_SERIAL_PORTS            ( 7 )
#define IDX_INVALID                 ( 255 )

/*
 * 自定义串口 RX 环形缓冲区 (与 bsp_rs485.h 完全一致, 518 字节).
 * 严格 SPSC: 本文件的 RXNE 中断只写 buffer/head, 任务侧
 * (uart_custom_exchange 的流式排空循环) 只写 tail.
 * 两个字段各只有一个写者, 占用用 head-tail 算 → 不需要任何临界区.
 * (原实现用 head/tail/count 三元组, 任务侧 count-- 的"读"被编译器提到 CPSID
 *  之前, ISR 的 count++ 会被任务回写的老值覆盖, 见 bsp_rs485.h 里的说明.)
 * 注意: 修改此处时必须同步更新 bsp_rs485.h 中的 UART_RxRing_t.
 */
#define UART_RX_RING_SIZE           512
#define UART_RX_RING_MASK           (UART_RX_RING_SIZE - 1U)
typedef struct {
    uint8_t  buffer[UART_RX_RING_SIZE];  /* 512 B                        */
    volatile uint16_t head;              /*   2 B, offset 512  ISR 只写 / 任务只读 */
    volatile uint16_t tail;              /*   2 B, offset 514  任务只写 / ISR 只读 */
    volatile uint8_t  rx_active;         /*   1 B, offset 516            */
    uint8_t  _pad;                       /*   1 B, 对齐到 518            */
} UART_RxRing_t;
extern UART_RxRing_t       uart_rx_ring[6];
extern uint8_t             UART_Custom[6];
/* 诊断计数器: 发送窗口内到达、被 TC 中断冲掉的残字节次数 (见下方 TC 分支) */
extern volatile uint16_t   uart_custom_flush_cnt[6];
/* 诊断计数器 (见 bsp_rs485.c 的 [DC-SHORT] 打印):
 *   dc_rx_store_cnt: ISR 实际存进 ring 的字节数
 *   dc_rx_ore_cnt  : ISR 入口快照里 ORE 置位次数 (= 硬件因溢出丢掉 1 字节)
 *   dc_rx_steal_cnt: 错误标志块读 DR 时 RXNE 仍置位次数 (= 原本会被吃掉的字节) */
extern volatile uint16_t   dc_rx_store_cnt[6];
extern volatile uint16_t   dc_rx_ore_cnt[6];
extern volatile uint16_t   dc_rx_steal_cnt[6];

/*
 * 自定义串口 TX 状态 (与 bsp_rs485.h 中的 UART_CustomTxState_t 完全一致).
 * 请求帧由 TXE 中断逐字节发出, TC 中断负责拉低 DE 并开启 RXNE 接收窗口.
 * 与 UART_RxRing_t 一致: 移植层自包含, 通过 extern 共享底层数组, 不 include
 * bsp_rs485.h (避免引入其整条 include 链, 也便于本文件独立编译).
 * 注意: 修改此处时必须同步更新 bsp_rs485.h.
 */
typedef struct {
    const uint8_t   *tx_buf;       /* 待发送数据指针 (来自调用方)   */
    uint16_t         tx_len;       /* 待发送字节数                  */
    uint16_t         tx_idx;       /* 下一个待发送字节下标 (ISR 自增) */
    volatile uint8_t tx_active;    /* 1 = TXE 中断正在发送帧        */
    volatile uint8_t tx_done;      /* 1 = TC 完成, DE 已切回接收    */
    GPIO_TypeDef    *de_port;      /* DE 引脚端口 (TC 完成时拉低)   */
    uint16_t         de_pin;       /* DE 引脚号                     */
} UART_CustomTxState_t;
extern UART_CustomTxState_t uart_custom_tx[6];


typedef struct
{
    UCHAR                   ubIdx;
    UART_HandleTypeDef      huart;
    GPIO_TypeDef           *pxRxPort;
    uint16_t                usRxPin;
    uint8_t                 ucRxAF;
    GPIO_TypeDef           *pxTxPort;
    uint16_t                usTxPin;
    uint8_t                 ucTxAF;
    GPIO_TypeDef           *pxDePort;
    uint16_t                usDePin;
    IRQn_Type               eIRQn;
    xMBHandle               xMBHdl;
    pbMBPSerialTransmitterEmptyCB pbTXFN;
    pvMBPSerialReceiverCB   pvRXFN;
    BOOL                    bTxEnabled;
    BOOL                    bRxEnabled;
} xSerialPortInternal;

static const struct
{
    USART_TypeDef          *Instance;
    GPIO_TypeDef           *RxPort;
    uint16_t                RxPin;
    uint8_t                 RxAF;
    GPIO_TypeDef           *TxPort;
    uint16_t                TxPin;
    uint8_t                 TxAF;
    GPIO_TypeDef           *DePort;
    uint16_t                DePin;
    IRQn_Type               IRQn;
} arxSerialPortConfig[MBP_SERIAL_PORTS] =
{
    { UART7, GPIOF, GPIO_PIN_6,  GPIO_AF8_UART7, GPIOF, GPIO_PIN_7, GPIO_AF8_UART7, GPIOF, GPIO_PIN_8,  UART7_IRQn  },//485-1
    { USART2, GPIOD, GPIO_PIN_6,  GPIO_AF7_USART2, GPIOD, GPIO_PIN_5, GPIO_AF7_USART2, GPIOD, GPIO_PIN_4,  USART2_IRQn },//485-2
    { UART5, GPIOD, GPIO_PIN_2,  GPIO_AF8_UART5, GPIOC, GPIO_PIN_12, GPIO_AF8_UART5, GPIOD, GPIO_PIN_3, UART5_IRQn  },//485-3
    { UART4, GPIOC, GPIO_PIN_11, GPIO_AF8_UART4, GPIOC, GPIO_PIN_10, GPIO_AF8_UART4, GPIOA, GPIO_PIN_15, UART4_IRQn  },//485-4
    { USART1, GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, GPIOA, GPIO_PIN_9, GPIO_AF7_USART1, GPIOA, GPIO_PIN_11, USART1_IRQn },//485-5
    { USART6,GPIOC, GPIO_PIN_7,  GPIO_AF8_USART6,GPIOC, GPIO_PIN_6,  GPIO_AF8_USART6,GPIOC, GPIO_PIN_8,  USART6_IRQn },//485-6
    { USART3, GPIOB, GPIO_PIN_11, GPIO_AF7_USART3, GPIOB, GPIO_PIN_10, GPIO_AF7_USART3, GPIOH, GPIO_PIN_6,  USART3_IRQn }//485-7 
};

static xSerialPortInternal arxSerialPorts[MBP_SERIAL_PORTS];
static BOOL bIsInitialized = FALSE;


static void prvConfigureUARTPins( const xSerialPortInternal *pxSerial );
static void prvInitDirectionPin( xSerialPortInternal *pxSerial );
static void prvInitPortClocks( void );
static xSerialPortInternal *prvGetSerialPort( UCHAR ucPort );
static void prvSetDE( xSerialPortInternal *pxSerial, BOOL bEnable );
static void prvUARTTxISR( xSerialPortInternal *pxSerial );
static void prvUARTRxISR( xSerialPortInternal *pxSerial );

static void prvEnableGPIOClock( GPIO_TypeDef *port )
{
    if( port == GPIOA )
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if( port == GPIOB )
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if( port == GPIOC )
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if( port == GPIOD )
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if( port == GPIOF )
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if( port == GPIOH )
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
}

static void prvInitPortClocks( void )
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
}

static void prvConfigureUARTPins( const xSerialPortInternal *pxSerial )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    prvEnableGPIOClock( pxSerial->pxTxPort );
    prvEnableGPIOClock( pxSerial->pxRxPort );

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Pin = pxSerial->usTxPin;
    GPIO_InitStruct.Alternate = pxSerial->ucTxAF;
    HAL_GPIO_Init( pxSerial->pxTxPort, &GPIO_InitStruct );

    GPIO_InitStruct.Pin = pxSerial->usRxPin;
    GPIO_InitStruct.Alternate = pxSerial->ucRxAF;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //test: 测试增加的代码
    HAL_GPIO_Init( pxSerial->pxRxPort, &GPIO_InitStruct );
}

static void prvInitDirectionPin( xSerialPortInternal *pxSerial )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if( pxSerial == NULL || pxSerial->pxDePort == NULL )
    {
        return;
    }

    prvEnableGPIOClock( pxSerial->pxDePort );

    GPIO_InitStruct.Pin = pxSerial->usDePin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( pxSerial->pxDePort, &GPIO_InitStruct );
    HAL_GPIO_WritePin( pxSerial->pxDePort, pxSerial->usDePin, GPIO_PIN_RESET );
}

static xSerialPortInternal *prvGetSerialPort( UCHAR ucPort )
{
    if( ucPort >= MBP_SERIAL_PORTS )
    {
        return NULL;
    }
    if( arxSerialPorts[ucPort].ubIdx != ucPort )
    {
        return NULL;
    }
    return &arxSerialPorts[ucPort];
}

static void prvSetDE( xSerialPortInternal *pxSerial, BOOL bEnable )
{
    if( pxSerial == NULL || pxSerial->pxDePort == NULL )
    {
        return;
    }
    HAL_GPIO_WritePin( pxSerial->pxDePort, pxSerial->usDePin, bEnable ? GPIO_PIN_SET : GPIO_PIN_RESET ); //Receiver Output Enable Active Low
}

static void prvUARTTxISR( xSerialPortInternal *pxSerial )
{
    BOOL bTxMore = FALSE;
    UBYTE ubValue = 0;

    if( pxSerial == NULL || pxSerial->pbTXFN == NULL )
    {
        return;
    }

#if MBS_SERIAL_API_VERSION == 2
    USHORT usBufferWritten = 0;
    bTxMore = ( ( pbMBPSerialTransmitterEmptyAPIV2CB )pxSerial->pbTXFN )( pxSerial->xMBHdl,
                                                                     &ubValue,
                                                                     1,
                                                                     &usBufferWritten );
    if( usBufferWritten > 0 )
    {
        pxSerial->huart.Instance->DR = ubValue;
    }
    if( !bTxMore || usBufferWritten == 0 )
#else
    bTxMore = ( ( pbMBPSerialTransmitterEmptyAPIV1CB )pxSerial->pbTXFN )( pxSerial->xMBHdl, &ubValue );
    if( bTxMore )
    {
        pxSerial->huart.Instance->DR = ubValue;
    }
    if( !bTxMore )
#endif
    {
        __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TXE );
        __HAL_UART_ENABLE_IT( &pxSerial->huart, UART_IT_TC );
        pxSerial->pbTXFN = NULL;
    }
}

static void prvUARTRxISR( xSerialPortInternal *pxSerial )
{
    UBYTE ubValue;

    if( pxSerial == NULL || pxSerial->pvRXFN == NULL )
    {
        return;
    }

    ubValue = ( UBYTE )( pxSerial->huart.Instance->DR & 0xFFU );

#if MBS_SERIAL_API_VERSION == 2
    ( ( pvMBPSerialReceiverAPIV2CB )pxSerial->pvRXFN )( pxSerial->xMBHdl, &ubValue, 1 );
#else
    ( ( pvMBPSerialReceiverAPIV1CB )pxSerial->pvRXFN )( pxSerial->xMBHdl, ubValue );
#endif
}

static void prvUARTTcISR( xSerialPortInternal *pxSerial )
{
    if( pxSerial == NULL )
    {
        return;
    }

    __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TC );
    __HAL_UART_CLEAR_FLAG( &pxSerial->huart, UART_FLAG_TC );
    prvSetDE( pxSerial, FALSE );// RX
}

// static void prvUARTIRQHandler( xSerialPortInternal *pxSerial )
// {
//     volatile ULONG tmp;
//     // ULONG sr = pxSerial->huart.Instance->SR;

//     if( pxSerial == NULL )
//     {
//         return;
//     }

//     // printf("SR=%08lx\r\n", sr);

//     /* RXNE */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_RXNE ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_RXNE ) != RESET ) )
//     {
//         prvUARTRxISR( pxSerial );
//     }

//     /* TXE */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_TXE ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_TXE ) != RESET ) )
//     {
//         prvUARTTxISR( pxSerial );
//     }

//     /* TC */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_TC ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_TC ) != RESET ) )
//     {
//         prvUARTTcISR( pxSerial );
//     }

//     /* ERROR FLAGS */
//     if( __HAL_USART_GET_FLAG( &pxSerial->huart, USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE) != RESET )
//     {
//         //  __HAL_UART_CLEAR_FLAG(&pxSerial->huart, (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

//         // __HAL_UART_CLEAR_OREFLAG( &pxSerial->huart );
//         // __HAL_UART_CLEAR_NEFLAG( &pxSerial->huart );
//         // __HAL_UART_CLEAR_FEFLAG( &pxSerial->huart );
//         tmp = pxSerial->huart.Instance->SR;
//         tmp = pxSerial->huart.Instance->DR;
//         (void)tmp;
//     }
// }

/*
 * 为自定义串口端口 (PortType 0x07/0x08) 注入 UART 实例指针.
 *
 * 自定义端口通过 uart_init_custom_raw() 初始化自己的 HAL 句柄,
 * 但 mbport 层的 arxSerialPorts[] 数组仅由 eMBPSerialInit() 填充
 * (该函数仅被 Modbus 端口调用).  对于自定义端口,
 * arxSerialPorts[port_idx].huart.Instance 保持 BSS 零值 (NULL).
 *
 * prvUARTIRQHandler 总是通过 arxSerialPorts[] 访问 UART 寄存器:
 *   sr = pxSerial->huart.Instance->SR;
 * 当 Instance==NULL 时, 读取的是 Flash 地址 0x00 的垃圾数据,
 * 而不是真实 UART 的 SR 寄存器 → 中断标志永远不会被清除 →
 * CPU 反复进入 ISR → 中断风暴 → 看门狗复位.
 *
 * 本函数由 uart_init_custom_raw 在 HAL_UART_Init 成功后调用,
 * 通过 Instance 指针在 arxSerialPortConfig[] 中查找匹配项,
 * 并将 Instance 写入对应的 arxSerialPorts[].  (O(1) 查找, 仅 7 项.)
 */
void mbport_set_custom_uart_instance( void *instance )
{
    int i;
    if( instance == NULL )
    {
        return;
    }
    for( i = 0; i < MBP_SERIAL_PORTS; i++ )
    {
        if( arxSerialPortConfig[i].Instance == ( USART_TypeDef * )instance )
        {
            arxSerialPorts[i].huart.Instance = ( USART_TypeDef * )instance;
            return;
        }
    }
}

static void prvUARTIRQHandler( xSerialPortInternal *pxSerial )
{
    ULONG sr;
    ULONG cr1;
    volatile ULONG tmp;
    BOOL   bRxDisabled = FALSE;

    if( pxSerial == NULL )
    {
        return;
    }

    /*
     * ── 自定义串口模式 (非 Modbus) 中断处理 ──
     * 通过 (pxSerial - arxSerialPorts) 反推端口索引, O(1) 直接判断.
     *
     * 自定义端口的中断必须始终由我们接管, 不能落入 FreeModbus 分支:
     *   FreeModbus 的 prvUARTRxISR 发现 pvRXFN==NULL 会直接 return,
     *   不读 DR → RXNE 不清除 → CPU 反复进入 ISR → 中断风暴 → 看门狗复位.
     *
     * rx_active=1: 正在接收 → 字节填入环形缓冲区
     * rx_active=0: 未在接收 → 仅读 DR 清除标志位, 防止中断风暴
     */
    {
        int _port = ( int )( pxSerial - arxSerialPorts );
        if( _port >= 0 && _port < 6 && UART_Custom[_port] )
        {
            sr  = pxSerial->huart.Instance->SR;
            cr1 = pxSerial->huart.Instance->CR1;

            /* 诊断: ORE 置位 = 上一字节还在 DR 里时又到一个字节, 硬件把**新到的**
             * 那个丢了 (STM32 溢出语义: DR 保留旧字节). 逐次统计, 任务侧清零看差值. */
            if( sr & USART_SR_ORE )
            {
                dc_rx_ore_cnt[_port]++;
            }

            /* ── TXE: 中断驱动发送请求帧 (与 Modbus prvUARTTxISR 同思路) ──
             * 请求帧字节由 ISR 按波特率连续移出, 与任务调度无关, 免疫抢占拆帧.
             *
             * 每个 TXE 中断移出一个字节: 写 DR 后 TXE 清, 硬件把字节移入移位
             * 寄存器后 TXE 重新置位 → 触发下一次 TXE 中断. 字节因此按波特率
             * 等间隔连续发出 (9600 时约 1.04ms/字节), 帧内间隙恒小于 T3.5.
             *
             * tx_idx 走满 tx_len (末字节已写入 DR) 后关 TXE、开 TC: 此时末字节
             * 仍在移位寄存器中, 必须等 TC (移位完成) 才能切回接收, 否则提前
             * 拉低 DE 会把末字节截断, 从机同样判帧无效.
             *
             * 安全前提: 任务在 uart_custom_exchange 中阻塞等待 tx_done, 期间
             * tx_buf (调用方局部数组) 保持有效, ISR 读写它是安全的. */
            if( ( sr & USART_SR_TXE ) && ( cr1 & USART_CR1_TXEIE ) )
            {
                UART_CustomTxState_t *_tx = &uart_custom_tx[_port];

                if( ( _tx->tx_active != 0U ) && ( _tx->tx_idx < _tx->tx_len ) )
                {
                    /* 逐字节写入 DR, 由硬件按波特率移出 */
                    pxSerial->huart.Instance->DR = _tx->tx_buf[_tx->tx_idx++];
                }
                else
                {
                    /* 全部字节已写入 → 关闭 TXE, 打开 TC, 等最后一字节移出 */
                    __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TXE );
                    __HAL_UART_ENABLE_IT( &pxSerial->huart, UART_IT_TC );
                }
            }

            /* ── TC: 最后一字节移出 → DE 切回接收, 立即开启 RXNE 窗口 ──
             * 在 ISR 内直接打开接收窗口, 消除 DE→RX 切换间隙:
             * 即使任务尚未从 vTaskDelay 醒来, 快速应答字节也会被收入 ring.
             *
             * 顺序固定: 先关 TCIE → 清 TC → 拉低 DE → 复位 ring → 开 RXNE.
             * TC 标志在移位完成后一直保持, 这里用写 SR 方式清除 (清 TC), 保持
             * 标志语义干净, 防止同一 ISR 再次进入本分支 (尽管 TCIE 已关不会触发
             * 新中断, 但重复进入会重复复位 ring, 把刚收到的字节清掉).
             *
             * ring 复位与 RXNE 使能放在 ISR 而非任务, 目的有二:
             *   1) 与拉低 DE 零间隙衔接, 快速应答不丢首字节;
             *   2) 任务侧不再重复初始化 ring, 避免与 ISR 竞争清掉已收字节.
             * 之后任务按 ring 的 head/tail 差判断有没有新字节. */
            if( ( sr & USART_SR_TC ) && ( cr1 & USART_CR1_TCIE ) )
            {
                __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TC );
                __HAL_UART_CLEAR_FLAG( &pxSerial->huart, UART_FLAG_TC );
                HAL_GPIO_WritePin( uart_custom_tx[_port].de_port,
                                   uart_custom_tx[_port].de_pin, GPIO_PIN_RESET );

                /* ── 先冲掉"发送期间"落进 DR 的残字节, 再开接收窗口 ──
                 * 发送期间 rx_active==0, 到达的字节不进 ring, 但会留在 DR 并置 RXNE.
                 * 末字节发送期间 (9600 下约 1ms) 到达的字节最致命: 此后不再有 TXE
                 * 中断去读 DR, 它会一直挂到 TC; 而下面的 RXNE 分支用的是 ISR 入口的
                 * sr 快照, 会把这个残字节当成应答的"首字节"存进刚复位的 ring →
                 * 整帧前移一字节 → 末两字节不再是 CRC → 校验必然失败 → 该从机被判
                 * 离线. 现场表现: 随机 1~2 台掉线, 与首字节超时/静默窗口都无关
                 * (调这两个参数没有任何改善).
                 * 此刻盒子不可能已经应答 (它要先收完整帧再启动发送, 至少数百 us),
                 * 故冲掉的必是自发自收的回声 / 上一台的迟到应答 / 总线噪声.
                 * 读 SR 再读 DR 同时清掉 RXNE/ORE/FE/NE. */
                if( pxSerial->huart.Instance->SR & ( USART_SR_RXNE | USART_SR_ORE ) )
                {
                    uart_custom_flush_cnt[_port]++;   /* 诊断: TX 窗口内曾有字节到达 */
                    tmp = pxSerial->huart.Instance->SR;
                    tmp = pxSerial->huart.Instance->DR;
                    ( void )tmp;
                }
                /* sr 是 ISR 入口的快照, 上面读 DR 只清了硬件标志. 若不清快照,
                 * 下面 RXNE 分支仍会据此再读一次"已经空了"的 DR, 把残留值当作
                 * 应答首字节存进 ring —— 与本修复目的相反. 清掉收方向标志后,
                 * 之后到达的字节会重新置 RXNE 并触发一次新的中断 (RXNEIE 已开),
                 * 用新的 sr 正常入 ring. 此刻 DE 刚拉低 ~2us, 而盒子最快也要
                 * 收完末字节再移出完整一帧首字节 (≥950us) 才可能让本机 RXNE 置位,
                 * 有 500 倍余量, 不会误丢真正的首字节. */
                sr &= ( ULONG )( ~( USART_SR_RXNE | USART_SR_ORE |
                                    USART_SR_FE   | USART_SR_NE  |
                                    USART_SR_PE ) );

                /* 直接进入接收窗口: 复位 ring 并开启 RXNE */
                {
                    UART_RxRing_t *_ring = &uart_rx_ring[_port];
                    _ring->head   = 0;
                    _ring->tail   = 0;
                    _ring->rx_active = 1;
                }
                __HAL_UART_ENABLE_IT( &pxSerial->huart, UART_IT_RXNE );

                uart_custom_tx[_port].tx_active = 0;
                uart_custom_tx[_port].tx_done   = 1;
            }

            /* ── RXNE: 正在接收 → 字节填入环形缓冲区 ── */
            if( uart_rx_ring[_port].rx_active )
            {
                if( sr & USART_SR_RXNE )
                {
                    uint8_t _byte = ( uint8_t )( pxSerial->huart.Instance->DR & 0xFFU );
                    UART_RxRing_t *_ring = &uart_rx_ring[_port];
                    /* 满判据: 留一个空槽 —— (head+1)==tail 即满 (容量 511 字节).
                     * ring 由任务侧流式排空, 正常情况绝不会满; 走到 else 说明任务
                     * 被抢占的时长超过了 ring 能吸收的字节数 (511B @19200 ≈ 266ms),
                     * 该字节连同后续字节丢失 → 帧 CRC 失败. 属异常调度, 需查任务
                     * 优先级/长时间阻塞点, 不要靠加大 ring 掩盖. */
                    if( ( uint16_t )( ( _ring->head + 1U ) & UART_RX_RING_MASK ) != _ring->tail )
                    {
                        _ring->buffer[_ring->head] = _byte; /* 先写数据, 后发布 head */
                        _ring->head = ( uint16_t )( ( _ring->head + 1U ) & UART_RX_RING_MASK );
                        dc_rx_store_cnt[_port]++;    /* 诊断: 本字节已入 ring */
                    }
                }
            }
            else
            {
                /* ── 未在接收: 仅排空中断标志, 防止风暴 ── */
                if( sr & USART_SR_RXNE )
                {
                    tmp = pxSerial->huart.Instance->DR;  /* 读 DR 清除 RXNE */
                }
            }

            /* 错误标志: 读 SR+DR 清除 (无论 rx_active 状态)
             * [2026-09-28 诊断] 原实现"无脑读 DR"有个隐患: 若此刻恰好又到一个
             *   字节 (RXNE 仍置位), 这一读会把它**吃掉**且不留痕迹 —— 正好能解释
             *   "应答恒定少最后 1 字节、切帧后再等 300ms 也没有". 现场调试助手在
             *   同一条总线上抓到盒子回的完整 143B 帧 (CRC 自洽), 故末字节确在线上,
             *   丢在主站侧. 本处先在**重新读的 SR** 上判断是否有待取字节:
             *     有 → 说明刚到一个真字节: 统计 dc_rx_steal_cnt 并**存进 ring**
             *          (修复, 不再丢弃; 帧尾那 50ms 内任务仍在排空 ring, 会取走);
             *     无 → 保持原样, 读 SR+DR 清错误标志.
             * 读 SR→读 DR 同时清 ORE/FE/NE/PE, 与 RXNE (上面已读走的那次同序列). */
            if( sr & ( USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE ) )
            {
                if( ( pxSerial->huart.Instance->SR & USART_SR_RXNE )
                    && uart_rx_ring[_port].rx_active
                    && ( ( uint16_t )( ( uart_rx_ring[_port].head + 1U ) & UART_RX_RING_MASK )
                         != uart_rx_ring[_port].tail ) )
                {
                    UART_RxRing_t *_r = &uart_rx_ring[_port];

                    dc_rx_steal_cnt[_port]++;
                    _r->buffer[_r->head] = ( uint8_t )( pxSerial->huart.Instance->DR & 0xFFU );
                    _r->head = ( uint16_t )( ( _r->head + 1U ) & UART_RX_RING_MASK );
                    dc_rx_store_cnt[_port]++;
                }
                else
                {
                    tmp = pxSerial->huart.Instance->SR;
                    tmp = pxSerial->huart.Instance->DR;
                    ( void )tmp;
                }
            }

            return;  /* 已处理, 不执行下方 FreeModbus 逻辑 */
        }
    }

    sr  = pxSerial->huart.Instance->SR;
    cr1 = pxSerial->huart.Instance->CR1;

    /* ERROR — clear error flags by reading SR then DR.
     * If the receiver is enabled, also disable RXNE interrupt to prevent
     * an ISR storm when the 485 bus is disconnected or noisy.
     * Without this, continuous framing errors starve the TIM6 timer ISR,
     * which prevents Modbus response timeouts from firing, permanently
     * hanging eMBMReadInputRegisters().
     * RXNE will be re-enabled by the Modbus state machine on the next
     * transaction (via eMBPSerialRxEnable). */
    if( sr & (USART_SR_ORE |
              USART_SR_NE  |
              USART_SR_FE  |
              USART_SR_PE) )
    {
        tmp = pxSerial->huart.Instance->SR;
        tmp = pxSerial->huart.Instance->DR;

        (void)tmp;

        /* Disable RXNE to break the interrupt storm.
         * Don't return — fall through to handle TXE/TC if they are pending,
         * which is critical during transmission phases. */
        if( pxSerial->bRxEnabled )
        {
            __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_RXNE );
            pxSerial->bRxEnabled = FALSE;
            bRxDisabled = TRUE;
        }

        /* Re-read SR after error clearing, as the flags may have changed. */
        if( bRxDisabled )
        {
            sr  = pxSerial->huart.Instance->SR;
            cr1 = pxSerial->huart.Instance->CR1;
        }
    }

    /* RXNE — processed only if no error occurred (otherwise the byte was
     * already consumed by the error-clearing DR read above). */
    if( (sr & USART_SR_RXNE) &&
        (cr1 & USART_CR1_RXNEIE) )
    {
        prvUARTRxISR( pxSerial );
    }

    /* TXE */
    if( (sr & USART_SR_TXE) &&
        (cr1 & USART_CR1_TXEIE) )
    {
        prvUARTTxISR( pxSerial );
    }

    /* TC */
    if( (sr & USART_SR_TC) &&
        (cr1 & USART_CR1_TCIE) )
    {
        prvUARTTcISR( pxSerial );
    }
}

void USART1_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[4] );
}

void USART2_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[1] );
}

void USART3_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[6] );
}

void UART4_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[3] );
}

void UART5_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[2] );
}

void USART6_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[5] );
}

void UART7_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[0] );
}

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                   UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBHdl )
{
    eMBErrorCode eStatus = MB_EPORTERR;
    xSerialPortInternal *pxSerialPort;
    ULONG wordLength;
    ULONG stopBits;
    ULONG parity;

    if( ( pxSerialHdl == NULL ) || ( xMBHdl == MB_HDL_INVALID ) || ( ucPort >= MBP_SERIAL_PORTS ) ||
        ( ulBaudRate == 0 ) || ( ( ucStopBits != 1 ) && ( ucStopBits != 2 ) ) )
    {
        return MB_EINVAL;
    }

    MBP_ENTER_CRITICAL_SECTION();
    if( !bIsInitialized )
    {
        for( UCHAR ubIdx = 0; ubIdx < MBP_SERIAL_PORTS; ubIdx++ )
        {
            arxSerialPorts[ubIdx].ubIdx = IDX_INVALID;
            arxSerialPorts[ubIdx].pbTXFN = NULL;
            arxSerialPorts[ubIdx].pvRXFN = NULL;
            arxSerialPorts[ubIdx].bTxEnabled = FALSE;
            arxSerialPorts[ubIdx].bRxEnabled = FALSE;
        }

        prvInitPortClocks();
        bIsInitialized = TRUE;
    }

    pxSerialPort = prvGetSerialPort( ucPort );
    if( pxSerialPort != NULL )
    {
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EAGAIN;
    }

    pxSerialPort = &arxSerialPorts[ucPort];
    pxSerialPort->ubIdx = ucPort;
    pxSerialPort->xMBHdl = xMBHdl;
    pxSerialPort->pbTXFN = NULL;
    pxSerialPort->pvRXFN = NULL;
    pxSerialPort->bTxEnabled = FALSE;
    pxSerialPort->bRxEnabled = FALSE;

    pxSerialPort->pxRxPort = arxSerialPortConfig[ucPort].RxPort;
    pxSerialPort->usRxPin = arxSerialPortConfig[ucPort].RxPin;
    pxSerialPort->ucRxAF = arxSerialPortConfig[ucPort].RxAF;
    pxSerialPort->pxTxPort = arxSerialPortConfig[ucPort].TxPort;
    pxSerialPort->usTxPin = arxSerialPortConfig[ucPort].TxPin;
    pxSerialPort->ucTxAF = arxSerialPortConfig[ucPort].TxAF;
    pxSerialPort->pxDePort = arxSerialPortConfig[ucPort].DePort;
    pxSerialPort->usDePin = arxSerialPortConfig[ucPort].DePin;
    pxSerialPort->eIRQn = arxSerialPortConfig[ucPort].IRQn;
    pxSerialPort->huart.Instance = arxSerialPortConfig[ucPort].Instance;

    if( eParity == MB_PAR_NONE )
    {
        wordLength = UART_WORDLENGTH_8B;
        parity = UART_PARITY_NONE;
    }
    else if( eParity == MB_PAR_EVEN )
    {
        wordLength = UART_WORDLENGTH_9B;
        parity = UART_PARITY_EVEN;
    }
    else if( eParity == MB_PAR_ODD )
    {
        wordLength = UART_WORDLENGTH_9B;
        parity = UART_PARITY_ODD;
    }
    else
    {
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EINVAL;
    }

    stopBits = ( ucStopBits == 2 ) ? UART_STOPBITS_2 : UART_STOPBITS_1;

    pxSerialPort->huart.Init.BaudRate = ulBaudRate;
    pxSerialPort->huart.Init.WordLength = ( ULONG )wordLength;
    pxSerialPort->huart.Init.StopBits = stopBits;
    pxSerialPort->huart.Init.Parity = parity;
    pxSerialPort->huart.Init.Mode = UART_MODE_TX_RX;
    pxSerialPort->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    pxSerialPort->huart.Init.OverSampling = UART_OVERSAMPLING_16;
    // pxSerialPort->huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // pxSerialPort->huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    prvConfigureUARTPins( pxSerialPort );
    prvInitDirectionPin( pxSerialPort );

    if( HAL_UART_Init( &pxSerialPort->huart ) != HAL_OK )
    {
        pxSerialPort->ubIdx = IDX_INVALID;
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EPORTERR;
    }

    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );

    /* UART IRQ priority = 2, higher than TIM6 (priority 3).
     * This ensures UART can preempt TIM6, which is critical for
     * accurate T3.5 character-timeout detection in Modbus RTU.
     * If TIM6 could preempt UART, the timer ISR might fire while
     * a character is being received, causing premature frame timeout. */
    HAL_NVIC_SetPriority( pxSerialPort->eIRQn, 2, 0 );
    HAL_NVIC_EnableIRQ( pxSerialPort->eIRQn );

    *pxSerialHdl = pxSerialPort;
    eStatus = MB_ENOERR;
    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}


eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    if( pxSerialPort != NULL )
    {
        if( pxSerialPort->ubIdx != IDX_INVALID )
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );
            HAL_UART_DeInit( &pxSerialPort->huart );
            pxSerialPort->ubIdx = IDX_INVALID;
            pxSerialPort->xMBHdl = MB_HDL_INVALID;
            pxSerialPort->pbTXFN = NULL;
            pxSerialPort->pvRXFN = NULL;
            pxSerialPort->bTxEnabled = FALSE;
            pxSerialPort->bRxEnabled = FALSE;
            eStatus = MB_ENOERR;
        }
    }
    return eStatus;
}


eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl,
                        pbMBPSerialTransmitterEmptyCB pbMBPTransmitterEmptyFN ) //mark:eMBPSerialTxEnable
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( pxSerialPort != NULL )
    {
        pxSerialPort->pbTXFN = pbMBPTransmitterEmptyFN;
        if( pbMBPTransmitterEmptyFN != NULL )
        {
            prvSetDE( pxSerialPort, TRUE ); //TX
            __HAL_UART_ENABLE_IT( &pxSerialPort->huart, UART_IT_TXE ); //上电时 TXE 默认为 1（空）,一开 TXE 中断，CPU 立刻进入一次 prvUARTTxISR
            pxSerialPort->bTxEnabled = TRUE;
            
        }
        else
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );
            prvSetDE( pxSerialPort, FALSE ); //RX
            pxSerialPort->bTxEnabled = FALSE;
        }
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION()
    return eStatus;
}


eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBPReceiveFN ) //mark:eMBPSerialRxEnable
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    if( pxSerialPort != NULL )
    {
        pxSerialPort->pvRXFN = ( pvMBPSerialReceiverAPIV1CB ) pvMBPReceiveFN;
        if( pvMBPReceiveFN != NULL )
        {
            // prvSetDE( pxSerialPort, FALSE ); //RX, 在TC中断里开比较好，等最后一个字节发送完成后再开RX，避免发送过程中被打断
            __HAL_UART_CLEAR_PEFLAG(&pxSerialPort->huart);
            __HAL_UART_ENABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            pxSerialPort->bRxEnabled = TRUE;
            
        }
        else
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            pxSerialPort->bRxEnabled = FALSE;
        }
        eStatus = MB_ENOERR;
    }
    return eStatus;
}
