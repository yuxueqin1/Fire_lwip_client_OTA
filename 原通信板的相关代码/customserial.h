/*************************************************************************************
 Copyright(c) 2012, Shanghai TIEDA Telecommunications Equipment Co., Ltd
 Filename      : customserial.h
 Version       : V1.00
 Programmer(s) : Yangzq	& Chenchang
 Last revised  : 2014-05-15
*************************************************************************************/
#ifndef  CUSTOMSERIAL_PRESENT
#define  CUSTOMSERIAL_PRESENT

typedef struct
{
	CPU_INT08U PortNum;
	CPU_INT08U PortType;
	CPU_INT08U BautRate;
	CPU_INT08U DataBit;
	CPU_INT08U StopBit;
	CPU_INT08U Parity;
	CPU_INT08U MainProType;
	CPU_INT08U SubProType;
	CPU_INT08U FenjiNum;
	CPU_INT08U QueryPeriod;
	CPU_INT08U SlaveModuelNum;
	CPU_INT08U SlaveModuelAddress[32];
	CPU_INT08U SlaveModuelType[32];
} BoardConfig;

typedef struct
{
	CPU_INT08U *send_buf;
	CPU_INT16U  send_len;
	CPU_INT08U *recv_buf;
	CPU_INT16U  recv_len;
	CPU_INT32S  recv_timeleft;
} CustomUartHandle;

/*************************************************************************************
									CONSTANTS
*************************************************************************************/ 
#define  CUSTOM_UART_RECV_TIMEOUT		10u				//10ms
#define  CUSTOM_VAL_TIMER1_FREQ			1000u			//1000Hz


#define  PARITY_NONE					0
#define  PARITY_ODD						1
#define  PARITY_EVEN					2
#define  PARITY_MARK					3
#define  PARITY_SPACE					4


/*************************************************************************************
								FUNCTION PROTOTYPES
*************************************************************************************/
 void Custom_Uart0Rx_Handler(void);
 void Custom_Uart0Tx_Handler(void);
 void Custom_Uart1Rx_Handler(void);
 void Custom_Uart1Tx_Handler(void);

 void Custom_Uart2Rx_Handler(void);
 void Custom_Uart2Tx_Handler(void);

 void Custom_Uart3Rx_Handler(void);
 void Custom_Uart3Tx_Handler(void);

CPU_BOOLEAN CustomSerialInit(CPU_INT08U Port, CPU_INT32U BaudRate, CPU_INT08U DataBits,
							 CPU_INT08U Parity, CPU_INT08U StopBits);

void BSP_TIMER1_Init(void);   

CPU_BOOLEAN Custom_Uart_Write(CPU_INT08U port, CPU_INT08U *ptr, CPU_INT16U len);
CPU_BOOLEAN Custom_Uart_Read(CPU_INT08U port, CPU_INT08U **ptr, CPU_INT16U *len);

#endif

    
