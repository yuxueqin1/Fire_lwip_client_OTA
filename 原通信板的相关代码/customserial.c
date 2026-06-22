/*************************************************************************************
 Copyright(c) 2012, Shanghai TIEDA Telecommunications Equipment Co., Ltd
 Filename      : customserial.c
 Version       : V1.00
 Programmer(s) : Yangzq	& Chenchang
 Last revised  : 2014-05-15
*************************************************************************************/

/*************************************************************************************
									Custom Serial
						NXP LPC1768 on the TJWX2014-COMM Board
*************************************************************************************/
#include  <includes.h>
#include "cmsis_os.h"                   /* CMSIS RTOS definitions             */
/*************************************************************************************
									CONSTANTS
*************************************************************************************/
#define UART_BAUDRATE_MIN		300
#define UART_BAUDRATE_MAX		230400

#define  DEF_FAIL             0u
#define  DEF_OK               1u

#define UART_0_PORT             0
#define UART_1_PORT             1
#define UART_2_PORT             2
#define UART_3_PORT             3

#define RS485_UART_0_DIR_INIT()				\
do {                        \
	  GPIO_SetDir(0, 27,1);	\
} while( 0 )
#define RS485_UART_0_DIR_TX()				\
do {										\
	GPIO_PinWrite (0, 27, 1); \
} while( 0 )
#define RS485_UART_0_DIR_RX()				\
do {										\
	 GPIO_PinWrite (0, 27, 0); \
} while( 0 )

#define RS485_UART_1_DIR_INIT()			GPIO_SetDir(0, 28,1)    							/*PINSEL_ConfigPin(2,2,0)	*/	

#define RS485_UART_1_DIR_TX()			  GPIO_PinWrite (0, 28, 1)

#define RS485_UART_1_DIR_RX()			  GPIO_PinWrite (0, 28, 0)

#define RS485_UART_2_DIR_INIT()				\
do {										\
	 GPIO_SetDir(0, 29,1);	\
} while( 0 )
#define RS485_UART_2_DIR_TX()				\
do {										\
	 GPIO_PinWrite (0, 29, 1); \
} while( 0 )
#define RS485_UART_2_DIR_RX()				\
do {										   \
	 GPIO_PinWrite (0, 29, 0); \
} while( 0 )

#define RS485_UART_3_DIR_INIT()				\
do {										\
	 GPIO_SetDir(0, 30,1); \
} while( 0 )
#define RS485_UART_3_DIR_TX()				\
do {										\
	 GPIO_PinWrite (0, 30, 1); \
} while( 0 )
#define RS485_UART_3_DIR_RX()				\
do {										\
	 GPIO_PinWrite (0, 30, 0); \
} while( 0 )

extern BoardConfig	BoardCfgTab[4];  

/*************************************************************************************
								TYPE DEFINITIONS
*************************************************************************************/

/*************************************************************************************
									VARIABLES
*************************************************************************************/
extern CPU_INT08U SerialDataBuf[4][2*1024];

extern	CPU_INT32U	Indicator;

//extern	OS_FLAG_GRP	*AppCustomUartFlag;

CustomUartHandle CustomUartHdls[4];


/*************************************************************************************
								FUNCTION PROTOTYPES
*************************************************************************************/

CPU_INT16U System_number=0;
CPU_INT08U APP_CUSTOM_UART0_RECV_START=0;
CPU_INT08U APP_CUSTOM_UART1_RECV_START=0;
CPU_INT08U APP_CUSTOM_UART2_RECV_START=0;
CPU_INT08U APP_CUSTOM_UART3_RECV_START=0;

extern CPU_INT32U	Systemtime;
extern osThreadId App_Taskserial_0_id;
extern osThreadId App_Taskserial_1_id;
extern osThreadId App_Taskserial_2_id;
extern osThreadId App_Taskserial_slave_id;

/*************************************************************************************
								CustomSerialInit
 Description : Initialize the uart clks & interrupt, etc.
 Argument(s) : none.
 Return(s)   : status.
 Caller(s)   : Application.
 Note(s)     : none.
*************************************************************************************/
CPU_BOOLEAN CustomSerialInit(CPU_INT08U Port, CPU_INT32U BaudRate, CPU_INT08U DataBits,
														 CPU_INT08U Parity, CPU_INT08U StopBits)
{
    CPU_INT08U	Status = DEF_OK;
    CPU_INT32U	UARTxConfig = 0;
    CPU_INT16U	div;				// Baud rate divisor
    CPU_INT08U	divlo;
    CPU_INT08U	divhi;
    CPU_INT32U	pclk_freq;
	volatile CPU_INT08U	dummy;

    /* Setup baudrate */
    if ((BaudRate < UART_BAUDRATE_MIN) || (BaudRate > UART_BAUDRATE_MAX))
	    {
		   Status = DEF_FAIL;
	    }

    /* Setup stopbits */
    switch (StopBits)
	    {
	     case 1:
	        UARTxConfig |= 0x00000000;
	        break;
	     case 2:
	        UARTxConfig |= 0x00000004;
	        break;
	     default:
	        Status = DEF_FAIL;
	        break;
	    }

    /* Setup parity */
    switch (Parity)
	    {
	     case PARITY_NONE:
	        UARTxConfig |= 0x00000000;
	        break;
	     case PARITY_ODD:
	        UARTxConfig |= 0x00000008;
	        break;
	     case PARITY_EVEN:
	        UARTxConfig |= 0x00000018;
	        break;
		 case PARITY_MARK:
	        UARTxConfig |= 0x00000028;
	        break;
		 case PARITY_SPACE:
	        UARTxConfig |= 0x00000038;
	        break;
	     default:
	        Status = DEF_FAIL;
	        break;
	    }

	/* Setup databits */
    switch (DataBits)
	    {
	 	 case 5:
	    	UARTxConfig |= 0x00000000;
	    	break;
	 	 case 6:
	    	UARTxConfig |= 0x00000001;
	    	break;
	 	 case 7:
	    	UARTxConfig |= 0x00000002;
	    	break;
	 	 case 8:
	    	UARTxConfig |= 0x00000003;
	    	break;
	     default:
	        Status = DEF_FAIL;
	        break;
	    }

    if (Status != DEF_FAIL)
		{
         switch (Port)
            {
             case UART_0_PORT:
							// Initialization UART0 RXD TXD pin
								PINSEL_ConfigPin(0,0,4);				
								PINSEL_ConfigPin(0,1,4);

								// Initialization UART0 RS485 direction pin
								RS485_UART_0_DIR_INIT();
								RS485_UART_0_DIR_RX();

								// Enable UART0 peripheral clock.
								CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0, ENABLE);

								/* Enable Interrupt for UART0 channel */
									NVIC_EnableIRQ(UART0_IRQn);
						 
								// Compute divisor for desired baud rate
								pclk_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_PER);
								div       = (CPU_INT16U)(((2 * pclk_freq / 16 / BaudRate) + 1) / 2);
								divlo     = div & 0x00FF;
								divhi     = (div >> 8) & 0x00FF;

								LPC_UART0->LCR	= UARTxConfig;    // Configure Data Bits and Parity 

								LPC_UART0->IER 	= 0;              // Disable UART0 Interrupts 

								LPC_UART0->LCR |= 0x80;           // Set divisor access bit 
								LPC_UART0->DLL	= divlo;          // Load divisor
								LPC_UART0->DLM	= divhi;
								LPC_UART0->LCR &= ~0x80;          // disable divisor access bit

								LPC_UART0->FCR	= 0x01;           // Enable FIFO, flush Rx & Tx 

								dummy			= LPC_UART0->IIR; // Required to Get Interrupts Started

								LPC_UART0->IER |= 0x03;           // Enable UART0 Rx & Tx Interrupts 			

                break;

             case UART_1_PORT:
								// Initialization UART1 RXD TXD pin
								PINSEL_ConfigPin(3,16,3);				
								PINSEL_ConfigPin(3,17,3);        
					      // Initialization UART1 RS485 direction pin
								RS485_UART_1_DIR_INIT();
								RS485_UART_1_DIR_RX();

								// Enable UART1 peripheral clock.
								CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCUART1, ENABLE);

								/* Enable Interrupt for UART1 channel */
								NVIC_EnableIRQ(UART1_IRQn);
												
								// Compute divisor for desired baud rate
								pclk_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_PER);
								div       = (CPU_INT16U)(((2 * pclk_freq / 16 / BaudRate) + 1) / 2);
								divlo     = div & 0x00FF;
								divhi     = (div >> 8) & 0x00FF;

								LPC_UART1->LCR	= UARTxConfig;    // Configure Data Bits and Parity 

								LPC_UART1->IER 	= 0;              // Disable UART1 Interrupts 

								LPC_UART1->LCR |= 0x80;           // Set divisor access bit 
								LPC_UART1->DLL	= divlo;          // Load divisor
								LPC_UART1->DLM	= divhi;
								LPC_UART1->LCR &= ~0x80;          // disable divisor access bit

								LPC_UART1->FCR	= 0x01;           // Enable FIFO, flush Rx & Tx 

								dummy			= LPC_UART1->IIR; // Required to Get Interrupts Started

								LPC_UART1->IER |= 0x03;           // Enable UART1 Rx & Tx Interrupts 

                break;

             case UART_2_PORT:
								// Initialization UART2 RXD TXD pin
					      // Not use UART2 in this App
								PINSEL_ConfigPin(0,10,1);				
								PINSEL_ConfigPin(0,11,1); 
						    // Initialization UART2 RS485 direction pin
								RS485_UART_2_DIR_INIT();
								RS485_UART_3_DIR_INIT();
								RS485_UART_2_DIR_RX();

							  // Enable UART0 peripheral clock.
								CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCUART2, ENABLE);

							  /* Enable Interrupt for UART2 channel */
								NVIC_EnableIRQ(UART2_IRQn);
										
						    // Compute divisor for desired baud rate
								pclk_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_PER);
								div       = (CPU_INT16U)(((2 * pclk_freq / 16 / BaudRate) + 1) / 2);
								divlo     = div & 0x00FF;
								divhi     = (div >> 8) & 0x00FF;

								LPC_UART2->LCR	= UARTxConfig;    // Configure Data Bits and Parity 

									LPC_UART2->IER 	= 0;              // Disable UART2 Interrupts 

									LPC_UART2->LCR |= 0x80;           // Set divisor access bit 
								LPC_UART2->DLL	= divlo;          // Load divisor
								LPC_UART2->DLM	= divhi;
									LPC_UART2->LCR &= ~0x80;          // disable divisor access bit

								LPC_UART2->FCR	= 0x01;           // Enable FIFO, flush Rx & Tx 

									dummy			= LPC_UART2->IIR; // Required to Get Interrupts Started

								LPC_UART2->IER |= 0x03;           // Enable UART2 Rx & Tx Interrupts 

                break;

             case UART_3_PORT:
								// Initialization UART3 RXD TXD pin
		             PINSEL_ConfigPin(0,2,2);				
								 PINSEL_ConfigPin(0,3,2);
					     // Initialization UART3 RS485 direction pin
								RS485_UART_3_DIR_INIT();
								RS485_UART_2_DIR_INIT();
								RS485_UART_3_DIR_RX();

					     // Enable UART3 peripheral clock.
					      CLKPWR_ConfigPPWR (CLKPWR_PCONP_PCUART3, ENABLE);

				      /* Enable Interrupt for UART3 channel */
					      NVIC_EnableIRQ(UART3_IRQn);
									
					    // Compute divisor for desired baud rate
								pclk_freq = CLKPWR_GetCLK(CLKPWR_CLKTYPE_PER);
								div       = (CPU_INT16U)(((2 * pclk_freq / 16 / BaudRate) + 1) / 2);
								divlo     = div & 0x00FF;
								divhi     = (div >> 8) & 0x00FF;

								LPC_UART3->LCR	= UARTxConfig;    // Configure Data Bits and Parity 

								LPC_UART3->IER 	= 0;              // Disable UART3 Interrupts 

								LPC_UART3->LCR |= 0x80;           // Set divisor access bit 
								LPC_UART3->DLL	= divlo;          // Load divisor
								LPC_UART3->DLM	= divhi;
								LPC_UART3->LCR &= ~0x80;          // disable divisor access bit

								LPC_UART3->FCR	= 0x01;           // Enable FIFO, flush Rx & Tx 

								dummy			= LPC_UART3->IIR; // Required to Get Interrupts Started

								LPC_UART3->IER |= 0x03;           // Enable UART3 Rx & Tx Interrupts 

                break;

             default:
								Status = DEF_FAIL;
                break;
            }
        }

    return (Status);
}


/*************************************************************************************
								Custom_Uart0Tx_Handler
 Description : Handle the Custom Uart0 Transmitted character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart0ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart0Tx_Handler(void)
{
	if (CustomUartHdls[0].send_len != 0)
		{
		 LPC_UART0->THR	= *(CustomUartHdls[0].send_buf++);
		 CustomUartHdls[0].send_len--;
		}
	else
		{
     RS485_UART_0_DIR_RX();			
		 osSignalSet(App_Taskserial_1_id, 0x00000002); 		 
		}	
}


/*************************************************************************************
								Custom_Uart0Rx_Handler
 Description : Handle the Custom Uart0 Received a character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart0ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart0Rx_Handler(void)
{

		if ( APP_CUSTOM_UART0_RECV_START== 0)
		{	
			APP_CUSTOM_UART0_RECV_START=1;
     if(BoardCfgTab[0].PortType==2)			 
		 CustomUartHdls[0].recv_buf = &SerialDataBuf[1][10];
		 else
		 CustomUartHdls[0].recv_buf = &SerialDataBuf[1][11];
		 
		 CustomUartHdls[0].recv_len = 0;
	 }

	*(CustomUartHdls[0].recv_buf++) = LPC_UART0->RBR;
	CustomUartHdls[0].recv_len++;
	CustomUartHdls[0].recv_timeleft = CUSTOM_UART_RECV_TIMEOUT;
}


/*************************************************************************************
								Custom_Uart1Tx_Handler
 Description : Handle the Custom Uart1 Transmitted character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart1ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
void	Custom_Uart1Tx_Handler(void)
{

	if (CustomUartHdls[1].send_len != 0)
		{
		 LPC_UART1->THR	= *(CustomUartHdls[1].send_buf++);
		 CustomUartHdls[1].send_len--;
		}
	else
		{
     RS485_UART_1_DIR_RX();			
		 osSignalSet(App_Taskserial_0_id, 0x00000002); 		
		}
}


/*************************************************************************************
								Custom_Uart1Rx_Handler
 Description : Handle the Custom Uart1 Received a character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart1ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart1Rx_Handler(void)
{
	if ( APP_CUSTOM_UART1_RECV_START== 0)
		{	
			APP_CUSTOM_UART1_RECV_START=1;
		
		 if(BoardCfgTab[1].PortType==2)			                     // 判断是否为外电网
		 CustomUartHdls[1].recv_buf = &SerialDataBuf[0][10];    
		 else
		 CustomUartHdls[1].recv_buf = &SerialDataBuf[0][11];
		 
		 CustomUartHdls[1].recv_len = 0;			 
	}

	*(CustomUartHdls[1].recv_buf++) = LPC_UART1->RBR;
	CustomUartHdls[1].recv_len++;
	CustomUartHdls[1].recv_timeleft = CUSTOM_UART_RECV_TIMEOUT;
}

/*************************************************************************************
								Custom_Uart2Tx_Handler
 Description : Handle the Custom Uart2 Transmitted character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart2ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart2Tx_Handler(void)
{

	if (CustomUartHdls[2].send_len != 0)
		{
		 LPC_UART2->THR	= *(CustomUartHdls[2].send_buf++);
		 CustomUartHdls[2].send_len--;
		}
	else
		{
		RS485_UART_2_DIR_RX();	
		osSignalSet(App_Taskserial_slave_id, 0x00000002); 
	
		}
}


/*************************************************************************************
								Custom_Uart2Rx_Handler
 Description : Handle the Custom Uart2 Received a character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart2ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart2Rx_Handler(void)
{
	if ( APP_CUSTOM_UART2_RECV_START== 0)
		{	
			APP_CUSTOM_UART2_RECV_START=1;

		  if(BoardCfgTab[2].PortType==2)			 
				 CustomUartHdls[2].recv_buf = &SerialDataBuf[3][10];
				else
				 CustomUartHdls[2].recv_buf = &SerialDataBuf[3][11];
		 CustomUartHdls[2].recv_len = 0;
				
		}

	*(CustomUartHdls[2].recv_buf++) = LPC_UART2->RBR;
	CustomUartHdls[2].recv_len++;
	CustomUartHdls[2].recv_timeleft = CUSTOM_UART_RECV_TIMEOUT;
}


/*************************************************************************************
								Custom_Uart3Tx_Handler
 Description : Handle the Custom Uart3 Transmitted character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart3ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
void	Custom_Uart3Tx_Handler(void)
{
	if (CustomUartHdls[3].send_len != 0)
		{
		 LPC_UART3->THR	= *(CustomUartHdls[3].send_buf++);
		 CustomUartHdls[3].send_len--;
		}
	else
		{
		RS485_UART_3_DIR_RX();	
		osSignalSet(App_Taskserial_2_id, 0x00000002); 		 
		}
}


/*************************************************************************************
								Custom_Uart3Rx_Handler
 Description : Handle the Custom Uart3 Received a character
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : Custom_Uart3ISR_Handler()
 Note(s)     : none.
*************************************************************************************/
 void	Custom_Uart3Rx_Handler(void)
{
if ( APP_CUSTOM_UART3_RECV_START== 0)
		{	
			APP_CUSTOM_UART3_RECV_START=1;

		  if(BoardCfgTab[3].PortType==2)			 
		 CustomUartHdls[3].recv_buf = &SerialDataBuf[2][10];
		 else
		 CustomUartHdls[3].recv_buf = &SerialDataBuf[2][11];
		 CustomUartHdls[3].recv_len = 0;
		 
		}

	*(CustomUartHdls[3].recv_buf++) = LPC_UART3->RBR;
	CustomUartHdls[3].recv_len++;
	CustomUartHdls[3].recv_timeleft = CUSTOM_UART_RECV_TIMEOUT;
}

/*************************************************************************************
									BSP_TIMER1_Init
 Description : Initialize the TIMER1 clks & interrupt, etc.
 Argument(s) : none.
 Return(s)   : status.
 Caller(s)   : BSP_PostInit()
 Note(s)     : none.
*************************************************************************************/
 void BSP_TIMER1_Init(void)
{
       TIM_MATCHCFG_Type MatchConfigStruct;
		// Configure Timer #1	
	      MatchConfigStruct.MatchChannel = 0;
				MatchConfigStruct.IntOnMatch = ENABLE;
				MatchConfigStruct.ResetOnMatch = ENABLE;
				MatchConfigStruct.StopOnMatch = DISABLE;
				MatchConfigStruct.ExtMatchOutputType = 0;
				MatchConfigStruct.MatchValue = CUSTOM_VAL_TIMER1_FREQ;             //1MS

				TIM_ConfigMatch(LPC_TIM1, &MatchConfigStruct);

	   /* Enable Interrupt for TIMER1 channel */
				NVIC_ClearPendingIRQ(TIMER1_IRQn);
		    NVIC_EnableIRQ(TIMER1_IRQn);
	      TIM_Cmd(LPC_TIM1,ENABLE);
}

/*************************************************************************************
								TIMER1_IRQHandler
 Description : Handle the Timer1 interrupt
 Argument(s) : none.
 Return(s)   : none.
 Caller(s)   : interrupt handle
 Note(s)     : none.
*************************************************************************************/
void TIMER1_IRQHandler(void)
{
//	CPU_INT08U	os_err;
// Clear Timer #1 interrupt.	
   TIM_ClearIntPending(LPC_TIM1, TIM_MR0_INT);
	 if(System_number++ > 999)
	 {
	  System_number=0;
	  Systemtime++;
	  }

	if (APP_CUSTOM_UART0_RECV_START==1)
		{
		 if (--CustomUartHdls[0].recv_timeleft < 0)
		 	{			
				APP_CUSTOM_UART0_RECV_START=0;				
			  osSignalSet(App_Taskserial_1_id, 0x00000001); 				
							
			}
		}

	if (APP_CUSTOM_UART1_RECV_START==1)
		{
		 if (--CustomUartHdls[1].recv_timeleft < 0)
		 	{
			 APP_CUSTOM_UART1_RECV_START=0;				
			 osSignalSet(App_Taskserial_0_id, 0x00000001); 				
							
			}
		}

		if (APP_CUSTOM_UART2_RECV_START==1)
		{
		 if (--CustomUartHdls[2].recv_timeleft < 0)
		 	{
			  APP_CUSTOM_UART2_RECV_START=0;
				osSignalSet(App_Taskserial_slave_id, 0x00000001); 
											
			}
		}

		if (APP_CUSTOM_UART3_RECV_START==1)
		{
		 if (--CustomUartHdls[3].recv_timeleft < 0)
		 	{
				
			APP_CUSTOM_UART3_RECV_START=0;
			osSignalSet(App_Taskserial_2_id, 0x00000001); 	
							
			}
		}
}


/*************************************************************************************
									Custom_Uart_Write
 Description : Uart write operate
 Argument(s) : port		uart port
 			  *ptr		point to the data buffer
			   len		length of the data
 Return(s)   : status.
 Caller(s)   : Application
 Note(s)     : none.
*************************************************************************************/
CPU_BOOLEAN Custom_Uart_Write(CPU_INT08U port, CPU_INT08U *ptr, CPU_INT16U len)
{
//	CPU_INT08U	os_err;

	switch (port)
		{
		 case UART_0_PORT:	
							CustomUartHdls[0].send_buf = ptr;
							CustomUartHdls[0].send_len = len;
							RS485_UART_0_DIR_TX();
							Custom_Uart0Tx_Handler();
							break;

		 case UART_1_PORT:					   	
						CustomUartHdls[1].send_buf = ptr;
						CustomUartHdls[1].send_len = len;
						RS485_UART_1_DIR_TX();
						Custom_Uart1Tx_Handler();
						break;

		 case UART_2_PORT:				   	
						CustomUartHdls[2].send_buf = ptr;
						CustomUartHdls[2].send_len = len;
						RS485_UART_2_DIR_TX();
						Custom_Uart2Tx_Handler();
						break;

		 case UART_3_PORT:					   	
					CustomUartHdls[3].send_buf = ptr;
					CustomUartHdls[3].send_len = len;
					RS485_UART_3_DIR_TX();
					Custom_Uart3Tx_Handler();
					break;

		 default:
					return (DEF_FAIL);
		}
	
	return (DEF_OK);
}


/*************************************************************************************
									Custom_Uart_Read
 Description : Uart Read operate
 Argument(s) : port		uart port
 			  *ptr		point to the data buffer
			   len		length of the data
 Return(s)   : status.
 Caller(s)   : Application
 Note(s)     : none.
*************************************************************************************/
CPU_BOOLEAN Custom_Uart_Read(CPU_INT08U port, CPU_INT08U **ptr, CPU_INT16U *len)
{
	switch (port)
		{
		 case UART_0_PORT:
  		 	*ptr = &SerialDataBuf[1][0];
			  *len = CustomUartHdls[0].recv_len;
		 	break;
			
		 case UART_1_PORT:
  		 	*ptr = &SerialDataBuf[0][0];
			*len = CustomUartHdls[1].recv_len;
		 	break;		

		 case UART_2_PORT:
  		 	*ptr = &SerialDataBuf[3][0];
			*len = CustomUartHdls[2].recv_len;
		 	break;

		 case UART_3_PORT:
  		 	*ptr = &SerialDataBuf[2][0];
			*len = CustomUartHdls[3].recv_len;
		 	break;

		 default:
		 	return (DEF_FAIL);
		}

	return (DEF_OK);
}






