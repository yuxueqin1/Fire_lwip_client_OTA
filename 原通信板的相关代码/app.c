/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::Network
 * Copyright (c) 2004-2014 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    BSD_Client.c
 * Purpose: LED Client example using BSD sockets
 *----------------------------------------------------------------------------*/
/*------------------------------------------
 * 2021/7/21日，版本号为V3.3，zld增加以下处理：
 * 1.网络发送增加超时判断，达到TCPSENDCOUNT次数后，复位板子
 * 2.单个通道故障次数超过4次，复位板子。
 * 3.修改喂狗操作改&为&&。
 * 4.增加日志信息，上传站机，协议为：端口号[1],主类型=0x50, 子类型=0x01, 分机号=0xff, 数据类型=0xff, 数据长度低位=0x02,数据长度高位=0x00,日志代码[1]，日志代码[2]
 *       日志代码：
 *                1= 启动
 *                2= 网络连接成功
 *                3= 看门狗复位
 *                4= 端口模块全故障
 * 5.修改电流曲线发送，在收到结束帧发送，原来在收到清除返回后发送，会存在一定风险丢曲线。
*/

#include  <includes.h>
#include "rl_net.h"                     /*Network definitions*/
#include <rl_net_lib.h>
#include  <app_cfg.h>

extern ETH_CFG eth0_config;
extern LOCALM  nlocalm;
uint8_t Adress_ip[4] = { 192,168,1,100 };

uint8_t MYMAC[6] = { 0xE4,0x95,0x6e,0XD3,00,00 };

extern int  memcpy(void *dp, const void *sp, int len);

extern int  memcmp(const void *dp, const void *sp, int len);

//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------

//   <h>Remote IP Address
//   ====================
//
//     <o>IP1: Address byte 1 <0-255>
//     <i> Default: 192
#define IP1            192

//     <o>IP2: Address byte 2 <0-255>
//     <i> Default: 168
#define IP2            168

//     <o>IP3: Address byte 3 <0-255>
//     <i> Default: 0
#define IP3            1

//     <o>IP4: Address byte 4 <0-255>
//     <i> Default: 100
#define IP4            88

//   </h>

//   <o>Remote Port <1-65535>
//   <i> Do not set number of port too small,
//   <i> maybe it is already used.
//   <i> Default: 1001
#define PORT_NUM       502

//   <o>Communication Protocol <0=> Stream (TCP) <1=> Datagram (UDP)
//   <i> Selecet a protocol for sending data.
#define PROTOCOL       0

#define TCPSENDCOUNT   20		//tcp发送次数，超次数后复位板子

//------------- <<< end of configuration section >>> -----------------------

#define BLINKLED 0x01                   /* Command for blink the leds        */
#if (PROTOCOL == 0)
 #define SOCKTYPE   SOCK_STREAM
#else
 #define SOCKTYPE   SOCK_DGRAM
#endif

typedef struct
{
	CPU_INT08U Curve_number[20];
	CPU_INT08U Frame_mark[20];
	CPU_INT08U Frame_sequence[20];
	CPU_INT16U Points_number[20];	
} AC_module_mrammanage;

extern	CustomUartHandle CustomUartHdls[4];

osThreadId App_TaskClien_id;
osThreadId App_TaskStart_id;
osThreadId App_TaskSave_id; 
osThreadId App_Taskserial_0_id;
osThreadId App_Taskserial_1_id;
osThreadId App_Taskserial_2_id;
osThreadId App_Taskserial_slave_id;
osThreadId App_Task_TcpSend_id;

static void App_TaskClient  (void const *arg);
static void App_TaskStart   (void const *arg);
static void App_TaskSave    (void const *arg);
static void App_Taskserial_0(void const *arg);
static void App_Taskserial_1 (void const *arg);
static void App_Taskserial_2(void const *arg);
static void App_Taskserial_slave(void const *arg);
static void App_Task_TcpSend(void const *arg);

osThreadDef(App_TaskClient, 		osPriorityNormal, 1, 0);
osThreadDef(App_TaskStart,  		osPriorityNormal, 1, 0);
osThreadDef(App_TaskSave,   		osPriorityNormal, 1, 0);
osThreadDef(App_Taskserial_0, 		osPriorityNormal, 1, 0);
osThreadDef(App_Taskserial_1, 		osPriorityNormal, 1, 0);
osThreadDef(App_Taskserial_2, 		osPriorityNormal, 1, 0);
osThreadDef(App_Taskserial_slave, 	osPriorityNormal, 1, 0);
osThreadDef(App_Task_TcpSend, 		osPriorityNormal, 1, 0);

CPU_INT16U   WDTUART0FLAG=0,WDTUART1FLAG=0,WDTUART2FLAG=0,WDTUART3FLAG=0,connect_time=0,heartbeat=0;
CPU_INT16U   WDTTCPSENDFLAG = 0;
CPU_INT08U   WTD_RESET=0,download_flag=0;

CPU_INT32U  		Systemtime=0; 
CPU_INT08U			changetime[4]={0,0,0,0};
CPU_INT08U			Task0AdjustNum = 0;
CPU_INT08U			Task1AdjustNum = 0;
CPU_INT08U			Task2AdjustNum = 0;
CPU_INT08U			Task3AdjustNum = 0;
BoardConfig			BoardCfgTab[4];
AC_module_mrammanage  AC_module_infor[4]={0};
CPU_INT16U  		Weibo_number[4][20];
CPU_INT08U 			CmdBuf0[15];
CPU_INT08U 	    CmdBuf1[15];
CPU_INT08U 			CmdBuf2[15];
CPU_INT08U 			CmdBuf3[15];


CPU_INT08U	EthernetDataBuf[308];
CPU_INT08U  SerialDataBuf[4][4*1024]={0};    // 修改20241224
CPU_INT08U	MramCfgDataBuf[300];

const	CPU_INT08U	RequestConfig[11] = {0x00, 0x07, 0x01, 0xff, 0xff, 0x04, 0x00, 0xff, 0xff, 0xff, 0xff};   //通信板请求连接

 CPU_INT16U   UART_delay[4]={0,0,0,0};
 CPU_INT08U   UART_Custom[4]={0,0,0,0};
 CPU_INT32U   ALARM_TEMP[4]={0,0,0,0}; 
 CPU_INT08U   ALARM=0; 
 //日志信息定义
 CPU_INT08U	  pStart = 0;    //程序启动标记1
 CPU_INT08U   netReset = 0;  //网络重连标记2
 CPU_INT08U	  logWTD_RESET = 0;	//复位3
 CPU_INT08U	  port1Error = 0;   //端口1数据错误4
 CPU_INT08U	  port2Error = 0;   //端口2数据错误4
 CPU_INT08U	  port3Error = 0;   //端口3数据错误4
 CPU_INT08U	  port4Error = 0;   //端口4数据错误4
 CPU_INT08U   port1DLQX = 0;
 CPU_INT08U   port2DLQX = 0;
 CPU_INT08U   port3DLQX = 0;
 CPU_INT08U   port4DLQX = 0;
 CPU_INT08U	  jiaoshi = 0;	//复位3
 
 int res;
 int sock;

static  USHORT	MBRegHoldingBuf[MB_REG_HOLDING_NREGS]={0};
		USHORT	MBRegInputBuf[MB_REG_INPUT_NREGS]={0};

static eMBException eMBSRegHoldingCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs, eMBSRegisterMode eRegMode);
static eMBException eMBSRegInputCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs);

typedef struct
{
	CPU_INT08U Ready;		//数据准备标志
	CPU_INT08U portFlag;
	CPU_INT08U WTD_RESET_flag;
	CPU_INT16U len;
	char sendBuf[1024 * 4];         // 20241224
}TcpDataStruct;

//TCP数据发送缓冲区，0-3为端口发送缓冲，4为其它发送缓冲，如日志等数据发送
TcpDataStruct TcpWaitingSendBuf[5];

static void WaitForTcpSendBufNull(CPU_INT08U portFlag)
{
	while (TcpWaitingSendBuf[portFlag].Ready > 0)
	{
		if (portFlag == 0) WDTUART0FLAG = 0;
		else if (portFlag == 1) WDTUART1FLAG = 0;
		else if (portFlag == 2) WDTUART2FLAG = 0;
		else if (portFlag == 3) WDTUART3FLAG = 0;
		osDelay(10);
	}
}

static void AddToTcpSendBuf(char *buf, CPU_INT16U len, CPU_INT08U WTD_RESET_flag, CPU_INT08U portFlag)
{
	WaitForTcpSendBufNull(portFlag);	//等待发送缓冲区为空

	TcpWaitingSendBuf[portFlag].portFlag = portFlag;
	TcpWaitingSendBuf[portFlag].WTD_RESET_flag = WTD_RESET_flag;
	memcpy(TcpWaitingSendBuf[portFlag].sendBuf, buf, len);
	TcpWaitingSendBuf[portFlag].len = len;
	TcpWaitingSendBuf[portFlag].Ready = 1;	

	WaitForTcpSendBufNull(portFlag);	//等待发送缓冲区为空

	/*CPU_INT08U timeCount = 10;
	while (send(sock, buf, len, 0) < 0)
	{
		if (portFlag == 0) WDTUART0FLAG = 0;
		else if (portFlag == 1) WDTUART1FLAG = 0;
		else if (portFlag == 2) WDTUART2FLAG = 0;
		else if (portFlag == 3) WDTUART3FLAG = 0;

		osDelay(10);
		if (timeCount>0) timeCount--;
		else WTD_RESET = flag;
	}*/
}

static void SendAdjustTimeToModue(CPU_INT08U TASKUART_PORT, CPU_INT08U com_port, CPU_INT08U m)
{
	osEvent         evt;
	// 主机发送命令格式：地址,10, 00, 00,00,02,04, 4E, 4E, 2D, 18, CRCH,CRCL
	// 从机返回命令格式：地址,10, 00, 00,00,02, CRCH,CRCL
	CmdBuf0[0] = BoardCfgTab[TASKUART_PORT].SlaveModuelAddress[m];
	CmdBuf0[1] = 0x10;
	CmdBuf0[2] = 0x00;
	CmdBuf0[3] = 0x00;
	CmdBuf0[4] = 0x00;
	CmdBuf0[5] = 0x02;
	CmdBuf0[6] = 0x04;
	CmdBuf0[7] = (Systemtime >> 24) & 0xff;
	CmdBuf0[8] = (Systemtime >> 16) & 0xff;
	CmdBuf0[9] = (Systemtime >> 8) & 0xff;
	CmdBuf0[10] = Systemtime & 0xff;
	*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

	Custom_Uart_Write(com_port, CmdBuf0, 13);
	evt = osSignalWait(0x02, 100); 									//发送超时								
	evt = osSignalWait(0x01, 300);  								//接收超时,没有对返回的数据进行判断
	if (TASKUART_PORT == TASKUART_0_PORT)WDTUART0FLAG = 0;
	else if (TASKUART_PORT == TASKUART_1_PORT)WDTUART1FLAG = 0;
	else if (TASKUART_PORT == TASKUART_2_PORT)WDTUART2FLAG = 0;
	else if (TASKUART_PORT == TASKUART_3_PORT)WDTUART3FLAG = 0;
}

static void SendLogToServer(CPU_INT08U port,CPU_INT08U logID)
{
	uint8_t m=0,timeCount=0;
	char sBuf[32] = {0};	
	//端口号[1],主类型=0x50, 子类型=0x01, 分机号=0xff, 数据类型=0xff, 数据长度低位=0x01,数据长度高位=0x00,日志代码[1]
	sBuf[m++] = port;    //端口号
	sBuf[m++] = 0x50;	 
	sBuf[m++] = 0x01;	 
	sBuf[m++] = 0xff;	 
	sBuf[m++] = 0xff;  	 
	sBuf[m++] = 0x02;	 
	sBuf[m++] = 0x00;	
	sBuf[m++] = logID;
	if(logID==3)	//复位
	{
		sBuf[m++] = WTD_RESET;
	}
	else sBuf[m++] = 0;
	AddToTcpSendBuf(sBuf, m, 26, 4);
}
static void checkSendLog()
{
	if (pStart == 1)
	{
		SendLogToServer(0, 1);   //发送程序启动日志
		pStart = 0;
	}
	if (netReset == 1)
	{
		SendLogToServer(0, 2);  //发送网络链接日志
		netReset = 0;
	}
	if (logWTD_RESET == 1)
	{
		SendLogToServer(0, 3);  //复位板子
		logWTD_RESET = 2;
	}
	if (port1Error == 1 || port1Error == 2)
	{
		SendLogToServer(1, 4);  //端口数据错误
		if (port1Error == 1)port1Error = 0;
		else port1Error = 0xff;
	}
	if (port2Error == 1 || port2Error == 2)
	{
		SendLogToServer(2, 4);  //端口数据错误
		if (port2Error == 1)port2Error = 0;
		else port2Error = 0xff;
	}
	if (port3Error == 1 || port3Error == 2)
	{
		SendLogToServer(3, 4);  //端口数据错误
		if (port3Error == 1)port3Error = 0;
		else port3Error = 0xff;
	}
	if (port4Error == 1 || port4Error == 2)
	{
		SendLogToServer(4, 4);  //端口数据错误
		if (port4Error == 1)port4Error = 0;
		else port4Error = 0xff;
	}
	if(port1DLQX !=0)
	{
		SendLogToServer(1, 5);  //端口1发送电流曲线
		port1DLQX = 0;
	}
	if(port2DLQX !=0)
	{
		SendLogToServer(2, 5);  //端口2发送电流曲线
		port2DLQX = 0;
	}
	if(port3DLQX !=0)
	{
		SendLogToServer(3, 5);  //端口3发送电流曲线
		port3DLQX = 0;
	}
	if(port4DLQX !=0)
	{
		SendLogToServer(4, 5);  //端口4发送电流曲线
		port4DLQX = 0;
	}
	if(jiaoshi != 0)
	{
		SendLogToServer(0, 6);  // 校时
		jiaoshi = 0;
	}
}


/*----------------------------------------------------------------------------
  Thread 'Client': BSD Client socket process
 *---------------------------------------------------------------------------*/
static void App_TaskClient(void const *arg)
{
	CPU_INT08U revErrCount = 0;
	CPU_INT08U reconnect = 0;

	SOCKADDR_IN     addr;
connectto:					
	revErrCount = 0;
	sock = socket(AF_INET, SOCKTYPE, 0);
	addr.sin_port = htons(PORT_NUM);
	addr.sin_family = PF_INET;
	addr.sin_addr.s_b1 = IP1;
	addr.sin_addr.s_b2 = IP2;
	addr.sin_addr.s_b3 = IP3;
	addr.sin_addr.s_b4 = IP4;

	res = connect(sock, (SOCKADDR *)&addr, sizeof (addr));
	netReset = 1;
	if (res < 0) 
	{
		while (closesocket(sock) != 0);		// 关闭连接

		// test:通信板TCP连接状态
		reconnect++;
		MBRegHoldingBuf[1] = reconnect;		
		if(reconnect>=20)
		{
			reconnect = 0;
			WTD_RESET = 28; 
		}

		osDelay(100);
		goto connectto;			// 重新连接，如果一直ping不同，就是网络问题
	}

	AddToTcpSendBuf((CPU_INT08U*)RequestConfig, 11, 27, 4);		//通信板向上位机发送请求配置信息
	
	heartbeat = 0;
	while (1)
	{
		res = recv(sock, (char *)EthernetDataBuf, 308, 0); // 上位机发过来的信息
		// 如果上位机关闭了连接，recv 将返回0;
		// 如果发生错误，recv 将返回-1
		if ((res <= 0))  // BUG:20241230 res<=0 为错误
		{
			revErrCount++;
			osDelay(1000); 
			if(revErrCount>=5)
			{
				revErrCount = 0;
				closesocket(sock);
				goto connectto;
			}
		}
		else
		{
			checkSendLog();    //检查是否有日志信息需要发送
			switch (EthernetDataBuf[0])
			{
			case CONFIG_PORT:
				//download
				if (EthernetDataBuf[1] == 0xA1 && EthernetDataBuf[2] == 0x2c && EthernetDataBuf[3] == 0x01)
				{
					memcpy(BoardCfgTab, &EthernetDataBuf[4], 0x012c);
					if ((BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum <= 32)&(BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum <= 32)&(BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum <= 32)&(BoardCfgTab[TASKUART_3_PORT].SlaveModuelNum <= 32))
					{
						EEPROM_Read(0, 0, (void*)MramCfgDataBuf, MODE_8_BIT, 300);

						if (memcmp(MramCfgDataBuf, BoardCfgTab, 0x012c) == 0) //判断是否与存储配置一样，一样正常执行，否则重新启动
						{
							//	EEPROM_Write(0, 0, (void*)&BoardCfgTab[0].PortNum, MODE_8_BIT,300);     //启动采集
							//	EEPROM_Read(0, 5, (void*)&AC_module_infor[4].Curve_number[0], MODE_8_BIT, 100);
							//	if((AC_module_infor[4].Frame_mark[m]==0xFF)&(AC_module_infor[4].Curve_number[m]==1)&(AC_module_infor[4].Points_number[m]<2001))
							//	{
							//		EEPROM_Read(0, 5, (void*)&AC_module_infor[0].Curve_number[0], MODE_8_BIT, 100);
							//	}
							//													
							//	EEPROM_Read(0, 7, (void*)&AC_module_infor[4].Curve_number[0], MODE_8_BIT, 100);
							//	if((AC_module_infor[4].Frame_mark[m]==0xFF)&(AC_module_infor[4].Curve_number[m]==1)&(AC_module_infor[4].Points_number[m]<2001))
							//	{
							//		EEPROM_Read(0, 7, (void*)&AC_module_infor[1].Curve_number[0], MODE_8_BIT, 100);
							//	}
							//													
							//	EEPROM_Read(0, 9, (void*)&AC_module_infor[4].Curve_number[0], MODE_8_BIT, 100);
							//	if((AC_module_infor[4].Frame_mark[m]==0xFF)&(AC_module_infor[4].Curve_number[m]==1)&(AC_module_infor[4].Points_number[m]<2001))
							//	{
							//		EEPROM_Read(0, 9, (void*)&AC_module_infor[2].Curve_number[0], MODE_8_BIT, 100);
							//	}
							//													
							//	EEPROM_Read(0, 11, (void*)&AC_module_infor[4].Curve_number[0], MODE_8_BIT, 100);
							//	if((AC_module_infor[4].Frame_mark[m]==0xFF)&(AC_module_infor[4].Curve_number[m]==1)&(AC_module_infor[4].Points_number[m]<2001))
							//	{
							//		EEPROM_Read(0, 11, (void*)&AC_module_infor[3].Curve_number[0], MODE_8_BIT, 100);
							//	}
							download_flag = 1;
						}
						else
						{
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							EEPROM_Write(0, 11, (void*)&AC_module_infor[TASKUART_3_PORT].Curve_number[0], MODE_8_BIT, 100);
							EEPROM_Write(0, 0, (void*)&BoardCfgTab[0].PortNum, MODE_8_BIT, 300);
							WTD_RESET = 1; 		                                                        //置看门狗复位				 
						}
					}
				}

				//time calibration 道岔教时命令
				else if (EthernetDataBuf[1] == 0xA3 && EthernetDataBuf[2] == 0x04 && EthernetDataBuf[3] == 0x00)
				{
					Systemtime = *((CPU_INT32U*)&EthernetDataBuf[4]);
					changetime[0] = 1;
					changetime[1] = 1;
					changetime[2] = 1;
					changetime[3] = 1;
					Task0AdjustNum = 0;
					Task1AdjustNum = 0;
					Task2AdjustNum = 0;
					Task3AdjustNum = 0;
					jiaoshi = 1;	//校时日志标志位
				}
				else if (EthernetDataBuf[1] == 0xA2 && EthernetDataBuf[2] == 0x00 && EthernetDataBuf[3] == 0x00)  //软件复位命令
				{
					WTD_RESET = 2;
				}
				else if (EthernetDataBuf[1] == 0xA5 && EthernetDataBuf[2] == 0x00 && EthernetDataBuf[3] == 0x00)  //心跳包命令
				{
					heartbeat = 0;
				}
				break;

			default:
				break;
			}
		}
	}
}

static void App_Task_TcpSend(void const *arg)
{
	while (true)
	{
		CPU_INT08U i = 0;
		WDTTCPSENDFLAG = 0;
		for (i = 0; i < 5; i++)
		{
			if (TcpWaitingSendBuf[i].Ready >0)
			{
				CPU_INT08U timeCount = 10;
				while (send(sock, TcpWaitingSendBuf[i].sendBuf, TcpWaitingSendBuf[i].len, 0) < 0) // tcp发送失败时
				{
					osDelay(10);
					if (timeCount>0) timeCount--;	
					else WTD_RESET = TcpWaitingSendBuf[i].WTD_RESET_flag;  // 看门狗复位
					WDTTCPSENDFLAG = 0;   // 给 TCP发送任务 超时标志位清零
				}
				TcpWaitingSendBuf[i].len = 0;
				TcpWaitingSendBuf[i].Ready = 0; 		// 发送完成
				osDelay(10);
			}
			else osDelay(10);
		}
	}
}

/*----------------------------------------------------------------------------
  Thread 'Master': 
 *---------------------------------------------------------------------------*/
static void App_Taskserial_0(void const *arg)
{
	xMBHandle 			xMBMMaster0;
	CPU_INT08U			sendCountOut = 0;
	CPU_INT08U 			n, m;
	CPU_INT16U 			*temp, Uart_temp;
	CPU_INT16U 			len = 0, DC_address_temp = 0;
	CPU_INT08U 			Flash_time[6];
	CPU_INT08U  		Modual_ststus_temp[32];
	CPU_INT16U      Weibo_temp = 0;
	CPU_INT08U  		modbus_number = 1, erro_number;
	CPU_INT32U  		COM_BautRate = 19200;
	uint16_t       *DC_electricity_SSZ;
	uint16_t       *DC_electricity_QX;
	osEvent         evt;
	CPU_INT08U **ptr;

	while ((BoardCfgTab[TASKUART_0_PORT].PortType == 0) | (BoardCfgTab[TASKUART_0_PORT].PortType == 3) | (download_flag == 0))
	{
		WDTUART0FLAG = 0;
		osDelay(100);
	}



	switch (BoardCfgTab[TASKUART_0_PORT].BautRate)	//tag 修改后的波特率
	{
	case 1:
		COM_BautRate = 9600;
		break;

	case 2:
		COM_BautRate = 19200;
		break;

	case 3:
		COM_BautRate = 38400;
		break;

	case 4:
		COM_BautRate = 57600;
		break;

	case 5:
		COM_BautRate = 115200;
		break;
	
	case 6:
		COM_BautRate = 230400;
		break;

	default:
		COM_BautRate = 19200;
		break;
	}

	switch (BoardCfgTab[TASKUART_0_PORT].PortType)
	{
	case 4:
		UART_Custom[TASKUART_0_PORT] = 0;                      //?ж????????modbus?ж????????ж?
		eMBMSerialInit(&xMBMMaster0,                        //???modbus
			MB_RTU,
			1,
			COM_BautRate,
			MB_PAR_NONE);
		if(BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum > 15)
		{
		UART_delay[TASKUART_0_PORT] = 150;   
		}
		else
		{
		UART_delay[TASKUART_0_PORT] = 900 - BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum * 50;   
		}
		break;

	case 1:
		UART_Custom[TASKUART_0_PORT] = 1;
		CustomSerialInit(1,                                   //???????????
			COM_BautRate,
			BoardCfgTab[0].DataBit,
			BoardCfgTab[0].Parity,
			BoardCfgTab[0].StopBit);
		UART_delay[TASKUART_0_PORT] = 900 - BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum * 25;
		break;

	default:
		UART_delay[TASKUART_0_PORT] = 440;
		break;
	}

	SerialDataBuf[TASKUART_0_PORT][0] = BoardCfgTab[TASKUART_0_PORT].PortNum;
	SerialDataBuf[TASKUART_0_PORT][1] = BoardCfgTab[TASKUART_0_PORT].MainProType;
	SerialDataBuf[TASKUART_0_PORT][2] = BoardCfgTab[TASKUART_0_PORT].SubProType;
	SerialDataBuf[TASKUART_0_PORT][3] = 0x00;
	SerialDataBuf[TASKUART_0_PORT][4] = BoardCfgTab[TASKUART_0_PORT].FenjiNum;
	ALARM_TEMP[TASKUART_0_PORT] = 0xFFFF;
	DC_electricity_SSZ = (uint16_t *)(SDRAM_BASE_ADDR);
	DC_electricity_QX = (uint16_t *)(SDRAM_BASE_ADDR + 0x5000);
	while (1)
	{
		WDTUART0FLAG = 0;
		temp = (CPU_INT16U*)&SerialDataBuf[TASKUART_0_PORT][11];               //7???????4????????
		Uart_temp = 2;
		len = 0;
		DC_address_temp = 0;
		if(BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum==0)
		{
			osDelay(50);
			continue;
		}
		for (m = 0; m < BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum; m++)
		{
			if (changetime[0] != 0)Task0AdjustNum++;

			switch (BoardCfgTab[TASKUART_0_PORT].SlaveModuelType[m])
			{


/*信号机*************************************************************************************************************************/
			case FLFXG_XHJ:
			case FLFXG_XHJ_KGL:
				modbus_number = 13;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)   // 开关量状态
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);               					//1???????0?????
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					//if (*(temp + 1) == 0)
						//MBRegHoldingBuf[1] |= (1 << m);
					//else
						//MBRegHoldingBuf[1] &= ~(1 << m);

					//if (BoardCfgTab[TASKUART_0_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					//{
						//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 22);
						//Uart_temp += 11;
					//}
					temp = temp + modbus_number;

				}
				else
				{
					if (Modual_ststus_temp[m] > 2)  // 大于三次没有接收到正确数据
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);

						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[1] &= ~(1 << m);
					// if (BoardCfgTab[TASKUART_0_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	for (n = 0; n < (modbus_number - 2); n++)
					// 	{
					// 		//MBRegHoldingBuf[Uart_temp] = 0;
					// 		//Uart_temp++;
					// 	}
					// }

				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


/*高频**信号机*************************************************************************************************************************/
			case FLFXG_NEW_XHJ:
				modbus_number=50;
				if(MB_ENOERR ==eMBMReadInputRegisters( xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp ))
					{		     
						Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);               					//1???????0?????
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					// if (BoardCfgTab[TASKUART_0_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 96);
					// 	//Uart_temp += 48;
					// }
					temp = temp + modbus_number;																						
					}
				else
					{	
						if (Modual_ststus_temp[m] > 2)
						{
							Modual_ststus_temp[m] = 0;
							*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
							//ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);

							//MBRegHoldingBuf[0] &= ~(1 << m);
						}
						else
						{
							Modual_ststus_temp[m]++;
						}

						for (n = 0; n < modbus_number; n++)
						{
							*temp++ = 0xffff;
						}

						// //MBRegHoldingBuf[1] &= ~(1 << m);
						
						// for (n = 0; n < (modbus_number - 2); n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }
						
					}

				len +=(modbus_number*2);
				osDelay (5);
				break;


/*25HZ轨道接收*************************************************************************************************************************/
			case FLFXG_GD_25_J:
				modbus_number = 34;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 64);	   //20191125??? 
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);												 
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					// //MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



/*高频**25HZ轨道接收*************************************************************************************************************************/
			case FLFXG_GD_25_J_HIGH:
			    modbus_number = 133; 
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, 133, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);     //4字节的模块状态

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 262);	   //20191125???

					//Uart_temp += 131;

					temp = temp + 133;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						// ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);												 
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					// //MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}				
				len += (modbus_number * 2);
				osDelay(5);
				break;



/**************************************************************************************************************************/
			case FLFXG_DC_AC:
				modbus_number = 10;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 16);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



/*25HZ轨道发送*************************************************************************************************************************/
			case FLFXG_GD_25_F:
				modbus_number = 30;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 56);	      //20191125???
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



/*高频**25HZ轨道发送*************************************************************************************************************************/			
			case FLFXG_GD_25_F_HIGH:
				modbus_number = 117;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 224);	      //20191125???
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



/*6线3对*************************************************************************************************************************/
			case FLFXG_CL:
				modbus_number = 8;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 12);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



/*高频**6线3对***********************************************************************************************************************/
			case FLFXG_CL_HIGH:  
				modbus_number = 62;  
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster0, BoardCfgTab[0].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_0_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_0_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[0] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[1] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[1] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 120);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_0_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[1] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;
			


/*交流道岔************************************************************************************************************************/			
			case FLFXG_AC_ZZJ:
				if (changetime[0] == 1)                           				     //У?????
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//??????								
					//evt = osSignalWait(0x01, 300);  								//??????
					//WDTUART0FLAG = 0;
				}

				// 主机发送命令格式：地址,03, 00, 00, 00, 00, CRCH,CRCL
				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //????????1??

				evt = osSignalWait(0x02, 100);     // 发送超时
				evt = osSignalWait(0x01, 400);		// 接收超时
				if (evt.status == osEventSignal)
				{
					//if(SerialDataBuf[TASKUART_0_PORT][13]==(CustomUartHdls[1].recv_len-5))  //?ж????	
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 29);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 58);     //????????29??????????

						//MBRegHoldingBuf[0] |= (1 << m);

						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 54);	        //20191130???
						// Uart_temp += 27;

						DC_address_temp += 29;
						len += 58;
						switch (SerialDataBuf[TASKUART_0_PORT][73])
						{
						case 0:    //???????												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//????????
								//2021??7??23??????????????
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), 2); //???????????	
								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//	{
								//		WDTUART0FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 3;
								//	}
								//	port1DLQX = n+1;
								//}
								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//?????????								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][75];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][74] == 0)&(SerialDataBuf[TASKUART_0_PORT][75] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//?????								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][75];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][74] == 0)&(SerialDataBuf[TASKUART_0_PORT][75] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH	
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][282], 2);		//???????????												

							//Weibo_number[0][m] += 2;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							//20210723????????????????????????????????????
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), 2); //???????????	
								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 3, 0);
/*
								sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								{
									WDTUART0FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 3;
								}*/
//								port1DLQX = n + 1;
							}

							SendLogToServer(1, 5);  //???1???????????
							//------------------------
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 13; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 29; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 58);
						DC_address_temp += 29;
						len += 58;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 13; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 29; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 58);
					DC_address_temp += 29;
					len += 58;
				}
				osDelay(1);
				break;



/*高频**交流道岔**********************************************************************************************************************/
			case FLFXG_AC_ZZJ_HIGH:
				if (changetime[0] == 1)                           				     //У?????
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//??????								
					//evt = osSignalWait(0x01, 300);  								//??????
					//WDTUART0FLAG = 0;
				}

				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //????????1??

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 400);
				if (evt.status == osEventSignal)
				{
					//if(SerialDataBuf[TASKUART_0_PORT][13]==(CustomUartHdls[1].recv_len-5))  //?ж????	
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 29+39); //68
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 136);     //????????29??????????

						//MBRegHoldingBuf[0] |= (1 << m);

						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 132);	        //20191130???
						//Uart_temp += 66;

						DC_address_temp += 68;
						len += 136;
						switch (SerialDataBuf[TASKUART_0_PORT][151])
						{
						case 0:    //???????												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//????????
								//2021??7??23??????????????
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), 2); //???????????	
								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//	{
								//		WDTUART0FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 3;
								//	}
								//	port1DLQX = n+1;
								//}
								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//?????????								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][153];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][152] == 0)&(SerialDataBuf[TASKUART_0_PORT][153] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//								BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//?????								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][153];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][152] == 0)&(SerialDataBuf[TASKUART_0_PORT][153] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH	
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][360], 2);		//???????????												

							//Weibo_number[0][m] += 2;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							//20210723????????????????????????????????????
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[0][m] / 2), 2); //???????????	
								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 3, 0);
/*
								sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								{
									WDTUART0FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 3;
								}*/
//								port1DLQX = n + 1;
							}

							SendLogToServer(1, 5);  //???1???????????
							//------------------------
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 52; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 68; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 136);
						DC_address_temp += 68;
						len += 136;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 52; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 68; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 136);
					DC_address_temp += 68;
					len += 136;
				}
				osDelay(1);
				break;


/**4线直流道岔************************************************************************************************************************/
			case  FLFXG_DC_ZZJ4:
				if (changetime[0] == 1)                           				//У?????
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//??????								
					//evt = osSignalWait(0x01, 300);  								//??????
					//WDTUART0FLAG = 0;
				}

				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //????????1??

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((SerialDataBuf[TASKUART_0_PORT][13] == (CustomUartHdls[1].recv_len - 5))&(CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 8);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 16);     //4???????????8??????????

						//MBRegHoldingBuf[0] |= (1 << m);

						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 8;
						len += 16;
						switch (SerialDataBuf[TASKUART_0_PORT][31])
						{
						case 0:    //???????												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//????????
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
								//memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_0_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//{
								//	WDTUART0FLAG = 0;
								//	osDelay(5);
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 4;
								//}
								//port1DLQX = 1;

								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								//EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//?????????								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][33];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][32] == 0)&(SerialDataBuf[TASKUART_0_PORT][33] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH		
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][40], Weibo_temp);
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//?????								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][33];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][32] == 0)&(SerialDataBuf[TASKUART_0_PORT][33] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH		
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][40], Weibo_temp);
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][90], 2);  //???????????	                                                                          //??????????? 
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							//----------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
							memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_0_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
							// sendCountOut = TCPSENDCOUNT;	//Marked by yuxueqin 2024.12.31

							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 4, 0);
/*
							while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
							{
								WDTUART0FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 4;
							}*/
							//port1DLQX = 1;
							SendLogToServer(1, 5);  //???1???????????
							//-------------------------------------
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 8; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 16);
						DC_address_temp += 8;
						len += 16;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 8; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}

					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 16);
					DC_address_temp += 8;
					len += 16;
				}
				osDelay(1);
				break;


/**高频**4线直流道岔**********************************************************************************************************************/
			case  FLFXG_DC_ZZJ4_HIGH:
				if (changetime[0] == 1)                           				//У?????
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//??????								
					//evt = osSignalWait(0x01, 300);  								//??????
					//WDTUART0FLAG = 0;
				}

				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //????????1??

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((SerialDataBuf[TASKUART_0_PORT][13] == (CustomUartHdls[1].recv_len - 5))&(CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 23);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 46);     //4???????????8??????????

						//MBRegHoldingBuf[0] |= (1 << m);

						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 23;
						len += 46;
						switch (SerialDataBuf[TASKUART_0_PORT][61])
						{
						case 0:    //???????												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//????????
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
								//memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_0_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//{
								//	WDTUART0FLAG = 0;
								//	osDelay(5);
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 4;
								//}
								//port1DLQX = 1;

								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								//EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//?????????								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][63];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][62] == 0)&(SerialDataBuf[TASKUART_0_PORT][63] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH		
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][70], Weibo_temp);
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//?????								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][63];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][62] == 0)&(SerialDataBuf[TASKUART_0_PORT][63] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH		
								Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][70], Weibo_temp);
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][120], 2);  //???????????	                                                                          //??????????? 
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							//----------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;

							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
							memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_0_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
							// sendCountOut = TCPSENDCOUNT;  //Marked by yuxueqin 2024.12.31

							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 4, 0);
/*
							while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
							{
								WDTUART0FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 4;
							}*/
							//port1DLQX = 1;
							SendLogToServer(1, 5);  //???1???????????
							//-------------------------------------
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 23; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 46);
						DC_address_temp += 23;
						len += 46;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 23; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}

					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 46);
					DC_address_temp += 23;
					len += 46;
				}
				osDelay(1);
				break;


/**6线直流道岔************************************************************************************************************************/
			case	FLFXG_DC_ZZJ6:
				if (changetime[0] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART0FLAG = 0;
				}

				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((SerialDataBuf[TASKUART_0_PORT][13] == (CustomUartHdls[1].recv_len - 5))&(CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 11);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 22);     //6线制直流道岔11个寄存器实时值

						//MBRegHoldingBuf[0] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 11;
						len += 22;
						switch (SerialDataBuf[TASKUART_0_PORT][37])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_0_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//	{
								//		WDTUART0FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 5;
								//	}
								//}
								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][39];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_0_PORT][38] == 0)&(SerialDataBuf[TASKUART_0_PORT][39] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][39];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_0_PORT][38] == 0)&(SerialDataBuf[TASKUART_0_PORT][39] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][146], 2);			//增加转换方向	

							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];

							//-----------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_0_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 5, 0);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								{
									WDTUART0FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 5;
								}*/
							}
							SendLogToServer(1, 5);  //端口1发送电流曲线
							//--------------------------------

							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 11; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 22);
						DC_address_temp += 11;
						len += 22;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
					for (n = 0; n < 11; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 22);
					DC_address_temp += 11;
					len += 22;
				}
				osDelay(1);
				break;


/*高频**6线直流道岔**********************************************************************************************************************/
			case	FLFXG_DC_ZZJ6_HIGH:
				if (changetime[0] == 1)                           				//У?????
				{
					SendAdjustTimeToModue(TASKUART_0_PORT, 1, m);
					//CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
					//CmdBuf0[1] = 0x10;
					//CmdBuf0[2] = 0x00;
					//CmdBuf0[3] = 0x00;
					//CmdBuf0[4] = 0x00;
					//CmdBuf0[5] = 0x02;
					//CmdBuf0[6] = 0x04;
					//CmdBuf0[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf0[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf0[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf0[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf0[11]) = CRC16(CmdBuf0, 11);

					//Custom_Uart_Write(1, CmdBuf0, 13);
					//evt = osSignalWait(0x02, 100); 								//??????								
					//evt = osSignalWait(0x01, 300);  								//??????
					//WDTUART0FLAG = 0;
				}

				CmdBuf0[0] = BoardCfgTab[TASKUART_0_PORT].SlaveModuelAddress[m];
				CmdBuf0[1] = 0x03;
				CmdBuf0[2] = 0x00;
				CmdBuf0[3] = 0x00;
				CmdBuf0[4] = 0x00;
				CmdBuf0[5] = AC_module_infor[TASKUART_0_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(CmdBuf0, 6);
				Custom_Uart_Write(1, CmdBuf0, 8);                       //????????1??

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf0[6]) = CRC16(&SerialDataBuf[TASKUART_0_PORT][11], (CustomUartHdls[1].recv_len - 2));
					if ((SerialDataBuf[TASKUART_0_PORT][13] == (CustomUartHdls[1].recv_len - 5))&(CmdBuf0[6] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 2])&(CmdBuf0[7] == SerialDataBuf[TASKUART_0_PORT][CustomUartHdls[1].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14], 26);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 52);     //6???????????11??????????

						//MBRegHoldingBuf[0] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_0_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 26;
						len += 52;
						switch (SerialDataBuf[TASKUART_0_PORT][67])
						{
						case 0:    //???????												    											      											
							if ((AC_module_infor[TASKUART_0_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_0_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_0_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
								//????????
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
								//WDTUART0FLAG = 0;
								//SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_0_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								//	{
								//		WDTUART0FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 5;
								//	}
								//}
								AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_0_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_0_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//?????????								   
							AC_module_infor[TASKUART_0_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][69];
							AC_module_infor[TASKUART_0_PORT].Frame_mark[m] = 0x55;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][68] == 0)&(SerialDataBuf[TASKUART_0_PORT][69] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];
							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//?????								   											    
							AC_module_infor[0].Frame_sequence[m] = SerialDataBuf[TASKUART_0_PORT][69];
							AC_module_infor[0].Frame_mark[m] = 0xFF;

							//д??? 
							if ((SerialDataBuf[TASKUART_0_PORT][68] == 0)&(SerialDataBuf[TASKUART_0_PORT][69] == 1))
							{
								Weibo_number[0][m] = 0;
								AC_module_infor[0].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //???FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_0_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_0_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_0_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_0_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[0].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_0_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[0][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[0][m] / 2), &SerialDataBuf[TASKUART_0_PORT][176], 2);			//???????????	

							AC_module_infor[0].Points_number[m] = Weibo_number[0][m];

							//-----------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = Weibo_number[TASKUART_0_PORT][m] + 12;
							WDTUART0FLAG = 0;
							SerialDataBuf[TASKUART_0_PORT][3] = 0X00;                                //??????????
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_0_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_0_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_0_PORT][17 + Weibo_number[TASKUART_0_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_0_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][11], (Weibo_number[TASKUART_0_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 5, 0);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], Weibo_number[0][m] + 19, 0) < 0)
								{
									WDTUART0FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 5;
								}*/
							}
							SendLogToServer(1, 5);  //???1???????????
							//--------------------------------

							EEPROM_Write(0, 5, (void*)&AC_module_infor[TASKUART_0_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 26; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 52);
						DC_address_temp += 26;
						len += 52;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[0] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
					for (n = 0; n < 26; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_0_PORT][14], 52);
					DC_address_temp += 26;
					len += 52;
				}
				osDelay(1);
				break;


/**************************************************************************************************************************/
			default://板子类型结束
				break;

			}
			WDTUART0FLAG = 0;
		}

		if (ALARM_TEMP[TASKUART_0_PORT] != 0XFFFF)      //????????
		{
			GPIO_PinWrite(2, 2, 1);
			ALARM |= (1 << TASKUART_0_PORT);
		}
		else
		{
			GPIO_PinWrite(2, 2, 0);
			ALARM &= ~(1 << TASKUART_0_PORT);
		}

		if ((*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7])) != ((1 << BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum) - 1))
		{
			GPIO_PinWrite(2, 7, 1);
		}
		else
		{
			GPIO_PinWrite(2, 7, 0);
		}
		if (Task0AdjustNum >= BoardCfgTab[TASKUART_0_PORT].SlaveModuelNum && changetime[0] != 0)
		{
			changetime[0] = 0;
		}

		//?????????????????????鶼?????????????á?????????3?λ?δ????????????
		if (*((CPU_INT32U *)&SerialDataBuf[TASKUART_0_PORT][7]) == 0)   // Board status
		{
			erro_number++;
			if (erro_number < 4)
			{				
				switch (BoardCfgTab[TASKUART_0_PORT].PortType)
				{
				case 4:
					eMBMSerialInit(&xMBMMaster0,  MB_RTU,    1,   COM_BautRate,  MB_PAR_NONE);             //???modbus  修改20241224
					port1Error = 1;
					break;
			
				case 1:
					CustomSerialInit(1, COM_BautRate, BoardCfgTab[0].DataBit, BoardCfgTab[0].Parity, BoardCfgTab[0].StopBit); // 修改20241224
					port1Error = 1;
					break;

				default:
					break;
				}
			}
			else
			{
				if (port1Error == 0) port1Error = 2;
				WTD_RESET = 6;
			}
		}
		else
		{
			port1Error = 0;
			erro_number = 0;
		}

		switch (BoardCfgTab[TASKUART_0_PORT].PortType)
		{
		case 1:                                           //??????鷢???????								           
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = len + 4;		 //??????????
			SerialDataBuf[TASKUART_0_PORT][3] = 0X01;                            //??????????						             	
			memcpy(&SerialDataBuf[TASKUART_0_PORT][11], DC_electricity_SSZ, len);

			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], len + 11, 7, 0);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], len + 11, 0) < 0)
			{
				WDTUART0FLAG = 0;
				osDelay(5);
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 7;
			}*/
			break;

		case  4:					    	                                                					// ????modbus???????????
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_0_PORT][5]) = len + 4;

			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_0_PORT][0], len + 11, 8, 0);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_0_PORT][0], len + 11, 0) < 0)
			{
				WDTUART0FLAG = 0;
				osDelay(10);				
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 8;
			}*/
			break;

		default:
			break;

		}
		osDelay(UART_delay[TASKUART_0_PORT]);
	}
}

/*----------------------------------------------------------------------------
  Thread 'Server': BSD Server socket process
 *---------------------------------------------------------------------------*/

static void App_Taskserial_1(void const *arg)
{
	xMBHandle 			xMBMMaster1;
	CPU_INT08U 			n, m, sendCountOut;
	CPU_INT16U 			*temp, Uart_temp;
	CPU_INT16U 			len = 0, DC_address_temp = 0;
	CPU_INT08U 			Flash_time[6];
	CPU_INT08U  		Modual_ststus_temp[32];
	CPU_INT16U      Weibo_temp = 0;
	CPU_INT08U  		modbus_number = 1, erro_number;
	CPU_INT32U  		COM_BautRate = 19200;
	uint16_t       *DC_electricity_SSZ;
	uint16_t       *DC_electricity_QX;
	osEvent         evt;

	while ((BoardCfgTab[TASKUART_1_PORT].PortType == 0) | (BoardCfgTab[TASKUART_1_PORT].PortType == 3) | (download_flag == 0))
	{
		WDTUART1FLAG = 0;
		osDelay(100);
	}


	switch (BoardCfgTab[TASKUART_1_PORT].BautRate)			//tag 修改后的波特率
	{
	case 1:
		COM_BautRate = 9600;
		break;

	case 2:
		COM_BautRate = 19200;
		break;

	case 3:
		COM_BautRate = 38400;
		break;

	case 4:
		COM_BautRate = 57600;
		break;

	case 5:
		COM_BautRate = 115200;
		break;

	case 6:
		COM_BautRate = 230400;
		break;

	default:
		COM_BautRate = 19200;
		break;
	}

	switch (BoardCfgTab[TASKUART_1_PORT].PortType)
	{
	case 4:
		UART_Custom[TASKUART_1_PORT] = 0;                      //判断是普通的modbus中断还是普通中断
		eMBMSerialInit(&xMBMMaster1,                        //普通modbus
			MB_RTU,
			0,
			COM_BautRate,
			MB_PAR_NONE);
		if(BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum > 15)
		{
		UART_delay[TASKUART_1_PORT] = 150;   // 可改
		}
		else
		{
		UART_delay[TASKUART_1_PORT] = 900 - BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum * 50;   // 可改	
		}
		break;

	case 1:
		UART_Custom[TASKUART_1_PORT] = 1;
		CustomSerialInit(0,                                   //道岔阻力模块
			COM_BautRate,
			BoardCfgTab[1].DataBit,
			BoardCfgTab[1].Parity,
			BoardCfgTab[1].StopBit);
		UART_delay[TASKUART_1_PORT] = 900 - BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum * 25;
		break;

	default:
		UART_delay[TASKUART_1_PORT] = 440;
		break;
	}

	SerialDataBuf[TASKUART_1_PORT][0] = BoardCfgTab[TASKUART_1_PORT].PortNum;
	SerialDataBuf[TASKUART_1_PORT][1] = BoardCfgTab[TASKUART_1_PORT].MainProType;
	SerialDataBuf[TASKUART_1_PORT][2] = BoardCfgTab[TASKUART_1_PORT].SubProType;
	SerialDataBuf[TASKUART_1_PORT][3] = 0x00;
	SerialDataBuf[TASKUART_1_PORT][4] = BoardCfgTab[TASKUART_1_PORT].FenjiNum;

	ALARM_TEMP[TASKUART_1_PORT] = 0xFFFF;
	DC_electricity_SSZ = (uint16_t *)(SDRAM_BASE_ADDR + 0X1000);
	DC_electricity_QX = (uint16_t *)(SDRAM_BASE_ADDR + 0x5000 + 0X1E000);
	while (1) 
	{
		WDTUART1FLAG = 0;
		temp = (CPU_INT16U*)&SerialDataBuf[TASKUART_1_PORT][11];               //7字节包头，4字节板件状态
		Uart_temp = 252;
		len = 0;
		DC_address_temp = 0;
		if(BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum==0)
		{
			osDelay(50);
			continue;
		}
		for (m = 0; m < BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum; m++)
		{
			if (changetime[1] != 0)Task1AdjustNum++;
			switch (BoardCfgTab[TASKUART_1_PORT].SlaveModuelType[m])
			{
			case FLFXG_XHJ:
			case FLFXG_XHJ_KGL:
				modbus_number = 13;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					// if (BoardCfgTab[TASKUART_1_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 22);
					// 	//Uart_temp += 11;
					// }
					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// if (BoardCfgTab[TASKUART_1_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	for (n = 0; n < (modbus_number - 2); n++)
					// 	{
					// 		//MBRegHoldingBuf[Uart_temp] = 0;
					// 		//Uart_temp++;
					// 	}
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_NEW_XHJ:
				modbus_number=50;
				if(MB_ENOERR ==eMBMReadInputRegisters( xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp ))
					{		     
						Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);               					//1???????0?????
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					// if (BoardCfgTab[TASKUART_1_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 96);
					// 	//Uart_temp += 48;
					// }
					temp = temp + modbus_number;																						
					}
				else
					{	
						if (Modual_ststus_temp[m] > 2)
						{
							Modual_ststus_temp[m] = 0;
							*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
							//ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);

							//MBRegHoldingBuf[250] &= ~(1 << m);
						}
						else
						{
							Modual_ststus_temp[m]++;
						}

						for (n = 0; n < modbus_number; n++)
						{
							*temp++ = 0xffff;
						}

						//MBRegHoldingBuf[251] &= ~(1 << m);
						
						// for (n = 0; n < (modbus_number - 2); n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }
						
					}

				len +=(modbus_number*2);
				osDelay (5);
				break;



			case FLFXG_GD_25_J:
				modbus_number = 34;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 64);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_GD_25_J_HIGH:// 高频
				modbus_number = 133;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 262);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



			case FLFXG_GD_25_F:
				modbus_number = 30;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 56);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


		
			case FLFXG_GD_25_F_HIGH://高频
				modbus_number = 117;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 224);	      //20191125???
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;		




			case FLFXG_CL:
				modbus_number = 8;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 12);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_CL_HIGH:// 高频
				modbus_number = 62;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 120);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_DC_AC:
				modbus_number = 10;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster1, BoardCfgTab[1].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);
					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_1_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_1_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[250] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[251] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[251] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 16);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_1_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[251] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;




			case FLFXG_AC_ZZJ:
				if (changetime[1] == 1)                           				     //校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 400);
				if (evt.status == osEventSignal)
				{
					//												if(SerialDataBuf[TASKUART_1_PORT][13]==(CustomUartHdls[0].recv_len-5))  //判断长度											
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 29);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 58);     //6线制直流道岔29个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 54);	         //20191130修改
						//Uart_temp += 27;

						DC_address_temp += 29;
						len += 58;
						switch (SerialDataBuf[TASKUART_1_PORT][73])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), 2); //增加转换方向	

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//	{
								//		WDTUART1FLAG = 0;
								//		osDelay(5);
								//		
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 9;
								//	}
								//	port2DLQX = n+1;
								//}
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][75];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][74] == 0)&(SerialDataBuf[TASKUART_1_PORT][75] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][75];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][74] == 0)&(SerialDataBuf[TASKUART_1_PORT][75] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH	
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][282], 2);		//增加转换方向												

							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							//----------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), 2); //增加转换方向	

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 9, 1);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								{
									WDTUART1FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 9;
								}*/
								//port2DLQX = n + 1;
							}
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//---------------------------------------
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 13; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 29; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 58);
						DC_address_temp += 29;
						len += 58;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 13; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 29; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 58);
					DC_address_temp += 29;
					len += 58;
				}
				osDelay(1);
				break;




			case FLFXG_AC_ZZJ_HIGH://高频
				if (changetime[1] == 1)                           				     //校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 400);
				if (evt.status == osEventSignal)
				{
					//												if(SerialDataBuf[TASKUART_1_PORT][13]==(CustomUartHdls[0].recv_len-5))  //判断长度											
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 29+39);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 136);     //6线制直流道岔29个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 132);	         //20191130修改
						//Uart_temp += 66;

						DC_address_temp += 68;
						len += 136;
						switch (SerialDataBuf[TASKUART_1_PORT][151])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), 2); //增加转换方向	

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//	{
								//		WDTUART1FLAG = 0;
								//		osDelay(5);
								//		
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 9;
								//	}
								//	port2DLQX = n+1;
								//}
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][153];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][152] == 0)&(SerialDataBuf[TASKUART_1_PORT][153] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][153];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][152] == 0)&(SerialDataBuf[TASKUART_1_PORT][153] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH	
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][360], 2);		//增加转换方向												

							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							//----------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[1][m] / 2), 2); //增加转换方向	

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 9, 1);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								{
									WDTUART1FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 9;
								}*/
								//port2DLQX = n + 1;
							}
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//---------------------------------------
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 52; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 68; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 136);
						DC_address_temp += 68;
						len += 136;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 52; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 68; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 136);
					DC_address_temp += 68;
					len += 136;
				}
				osDelay(1);
				break;



			case  FLFXG_DC_ZZJ4:
				if (changetime[1] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((SerialDataBuf[TASKUART_1_PORT][13] == (CustomUartHdls[0].recv_len - 5))&(CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 8);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 16);     //4线制直流道岔8个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 8;
						len += 16;
						switch (SerialDataBuf[TASKUART_1_PORT][31])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;

								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_1_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//{
								//	WDTUART1FLAG = 0;
								//	osDelay(5);									
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 10;
								//}
								//port2DLQX = 1;
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][33];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][32] == 0)&(SerialDataBuf[TASKUART_1_PORT][33] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][40], Weibo_temp);
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][33];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][32] == 0)&(SerialDataBuf[TASKUART_1_PORT][33] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][40], Weibo_temp);
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][90], 2);  //增加转换方向	                                                                          //增加转换方向 
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							//-------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_1_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
							
							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 10, 1);
							/*sendCountOut = TCPSENDCOUNT;
							while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
							{
								WDTUART1FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 10;
							}*/
							//port2DLQX = 1;
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//-----------------------------
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 8; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 16);
						DC_address_temp += 8;
						len += 16;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 8; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 16);
					DC_address_temp += 8;
					len += 16;
				}
				osDelay(1);
				break;



			case  FLFXG_DC_ZZJ4_HIGH: //高频
				if (changetime[1] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((SerialDataBuf[TASKUART_1_PORT][13] == (CustomUartHdls[0].recv_len - 5))&(CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 23);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 46);     //4线制直流道岔8个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 23;
						len += 46;
						switch (SerialDataBuf[TASKUART_1_PORT][61])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;

								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_1_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//{
								//	WDTUART1FLAG = 0;
								//	osDelay(5);									
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 10;
								//}
								//port2DLQX = 1;
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][63];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][62] == 0)&(SerialDataBuf[TASKUART_1_PORT][63] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][70], Weibo_temp);
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][63];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][62] == 0)&(SerialDataBuf[TASKUART_1_PORT][63] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][70], Weibo_temp);
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][120], 2);  //增加转换方向	                                                                          //增加转换方向 
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							//-------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_1_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
							
							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 10, 1);
							/*sendCountOut = TCPSENDCOUNT;
							while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
							{
								WDTUART1FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 10;
							}*/
							//port2DLQX = 1;
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//-----------------------------
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 23; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 46);
						DC_address_temp += 23;
						len += 46;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 23; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 46);
					DC_address_temp += 23;
					len += 46;
				}
				osDelay(1);
				break;




			case	FLFXG_DC_ZZJ6:
				if (changetime[1] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((SerialDataBuf[TASKUART_1_PORT][13] == (CustomUartHdls[0].recv_len - 5))&(CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 11);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 22);     //6线制直流道岔10个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 11;
						len += 22;
						switch (SerialDataBuf[TASKUART_1_PORT][37])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;

								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_1_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//	{
								//		WDTUART1FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 11;
								//	}
								//	port2DLQX = 1;
								//}
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][39];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][38] == 0)&(SerialDataBuf[TASKUART_1_PORT][39] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][39];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][38] == 0)&(SerialDataBuf[TASKUART_1_PORT][39] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][146], 2);			//增加转换方向	


							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];

							//-------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_1_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 11, 1);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								{
									WDTUART1FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 11;
								}*/
								//port2DLQX = 1;
							}
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//---------------------------

							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 11; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 22);
						DC_address_temp += 11;
						len += 22;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 11; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 22);
					DC_address_temp += 11;
					len += 22;
				}
				osDelay(1);
				break;


			case	FLFXG_DC_ZZJ6_HIGH: //高频
				if (changetime[1] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_1_PORT, 0, m);
					//CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
					//CmdBuf1[1] = 0x10;
					//CmdBuf1[2] = 0x00;
					//CmdBuf1[3] = 0x00;
					//CmdBuf1[4] = 0x00;
					//CmdBuf1[5] = 0x02;
					//CmdBuf1[6] = 0x04;
					//CmdBuf1[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf1[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf1[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf1[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf1[11]) = CRC16(CmdBuf1, 11);

					//Custom_Uart_Write(0, CmdBuf1, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART1FLAG = 0;
				}

				CmdBuf1[0] = BoardCfgTab[TASKUART_1_PORT].SlaveModuelAddress[m];
				CmdBuf1[1] = 0x03;
				CmdBuf1[2] = 0x00;
				CmdBuf1[3] = 0x00;
				CmdBuf1[4] = 0x00;
				CmdBuf1[5] = AC_module_infor[TASKUART_1_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(CmdBuf1, 6);
				Custom_Uart_Write(0, CmdBuf1, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf1[6]) = CRC16(&SerialDataBuf[TASKUART_1_PORT][11], (CustomUartHdls[0].recv_len - 2));
					if ((SerialDataBuf[TASKUART_1_PORT][13] == (CustomUartHdls[0].recv_len - 5))&(CmdBuf1[6] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 2])&(CmdBuf1[7] == SerialDataBuf[TASKUART_1_PORT][CustomUartHdls[0].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14], 26);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 52);     //6线制直流道岔10个寄存器实时值

						//MBRegHoldingBuf[250] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_1_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 26;
						len += 52;
						switch (SerialDataBuf[TASKUART_1_PORT][67])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_1_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_1_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_1_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;

								//WDTUART1FLAG = 0;
								//SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_1_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								//	{
								//		WDTUART1FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 11;
								//	}
								//	port2DLQX = 1;
								//}
								AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_1_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_1_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_1_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][69];
							AC_module_infor[TASKUART_1_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][68] == 0)&(SerialDataBuf[TASKUART_1_PORT][69] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];
							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[1].Frame_sequence[m] = SerialDataBuf[TASKUART_1_PORT][69];
							AC_module_infor[1].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_1_PORT][38] == 0)&(SerialDataBuf[TASKUART_1_PORT][39] == 1))
							{
								Weibo_number[1][m] = 0;
								AC_module_infor[1].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_1_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_1_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_1_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_1_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[1].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_1_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[1][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[1][m] / 2), &SerialDataBuf[TASKUART_1_PORT][176], 2);			//增加转换方向	


							AC_module_infor[1].Points_number[m] = Weibo_number[1][m];

							//-------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = Weibo_number[TASKUART_1_PORT][m] + 12;
							WDTUART1FLAG = 0;
							SerialDataBuf[TASKUART_1_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_1_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_1_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_1_PORT][17 + Weibo_number[TASKUART_1_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_1_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][11], (Weibo_number[TASKUART_1_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 11, 1);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], Weibo_number[1][m] + 19, 0) < 0)
								{
									WDTUART1FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 11;
								}*/
								//port2DLQX = 1;
							}
							SendLogToServer(2, 5);  //端口2发送电流曲线
							//---------------------------

							EEPROM_Write(0, 7, (void*)&AC_module_infor[TASKUART_1_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 26; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 52);
						DC_address_temp += 26;
						len += 52;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[250] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 26; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_1_PORT][14], 52);
					DC_address_temp += 26;
					len += 52;
				}
				osDelay(1);
				break;




			default:
				break;

			}
			WDTUART1FLAG = 0;
		}

		if (ALARM_TEMP[TASKUART_1_PORT] != 0XFFFF)      //驱动报警
		{
			GPIO_PinWrite(2, 1, 1);
			ALARM |= (1 << TASKUART_1_PORT);
		}
		else
		{
			GPIO_PinWrite(2, 1, 0);
			ALARM &= ~(1 << TASKUART_1_PORT);
		}

		if ((*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7])) != ((1 << BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum) - 1))
		{
			GPIO_PinWrite(2, 6, 1);
		}
		else
		{
			GPIO_PinWrite(2, 6, 0);
		}
		if (Task1AdjustNum >= BoardCfgTab[TASKUART_1_PORT].SlaveModuelNum && changetime[1] != 0)
		{
			changetime[1] = 0;
		}
		
		//端口重启，当端口里全部模块都为超时时，端口重置。连续重置3次还未恢复则为真的超时
		if (*((CPU_INT32U *)&SerialDataBuf[TASKUART_1_PORT][7]) == 0)
		{
			erro_number++;
			if (erro_number < 4)
			{
				switch (BoardCfgTab[TASKUART_1_PORT].PortType)
				{
				case 4:
						eMBMSerialInit(&xMBMMaster1,  MB_RTU,    0,   COM_BautRate,  MB_PAR_NONE);             //???modbus
						port2Error = 1;
						break;

				case 1:
					CustomSerialInit(0, COM_BautRate, BoardCfgTab[1].DataBit, BoardCfgTab[1].Parity, BoardCfgTab[1].StopBit);
					port2Error = 1;
					break;
				default:
					break;
				}
			}
			else
			{
				if (port2Error == 0) port2Error = 2;
				WTD_RESET = 12;  
			}
		}
		else
		{
			port2Error = 0;
			erro_number = 0;
		}

		switch (BoardCfgTab[TASKUART_1_PORT].PortType)
		{
		case 1:                                              													  //阻力模块发送模块状态								           
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = len + 4;		  				//数据长度实时值
			SerialDataBuf[TASKUART_1_PORT][3] = 0X01;                             			//表示是曲线帧						             	
			memcpy(&SerialDataBuf[TASKUART_1_PORT][11], DC_electricity_SSZ, len);
			
			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], len + 11, 13, 1);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], len + 11, 0) < 0)
			{
				WDTUART1FLAG = 0;
				osDelay(5);				
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 13;
			}*/
			break;

		case  4:					    	                            // 正常modbus轮训完发送数据
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_1_PORT][5]) = len + 4;

			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_1_PORT][0], len + 11, 14, 1);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_1_PORT][0], len + 11, 0) < 0)
			{
				WDTUART1FLAG = 0;
				osDelay(10);				
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 14;
			}*/
			break;

		default:
			break;

		}
		osDelay(UART_delay[TASKUART_1_PORT]);
	}
}

/*----------------------------------------------------------------------------
  Thread 'Master': 
 *---------------------------------------------------------------------------*/
static void App_Taskserial_2(void const *arg)
{
	xMBHandle 			xMBMMaster2;
	CPU_INT08U 			n, m, sendCountOut;
	CPU_INT16U 			*temp, Uart_temp;
	CPU_INT16U 			len = 0, DC_address_temp = 0;
	CPU_INT08U 			Flash_time[6];
	CPU_INT08U  		Modual_ststus_temp[32];
	CPU_INT16U      Weibo_temp = 0;
	CPU_INT08U  		modbus_number = 1, erro_number;
	CPU_INT32U  		COM_BautRate = 19200;
	uint16_t       *DC_electricity_SSZ;
	uint16_t       *DC_electricity_QX;
	osEvent         evt;

	while ((BoardCfgTab[TASKUART_2_PORT].PortType == 0) | (BoardCfgTab[TASKUART_2_PORT].PortType == 3) | (download_flag == 0))
	{
		WDTUART2FLAG = 0;
		osDelay(100);
	}


	switch (BoardCfgTab[TASKUART_2_PORT].BautRate)	//tag 修改后的波特率
	{
	case 1:
		COM_BautRate = 9600;
		break;

	case 2:
		COM_BautRate = 19200;
		break;

	case 3:
		COM_BautRate = 38400;
		break;

	case 4:
		COM_BautRate = 57600;
		break;

	case 5:
		COM_BautRate = 115200;
		break;

	case 6:
		COM_BautRate = 230400;
		break;

	default:
		COM_BautRate = 19200;
		break;
	}

	switch (BoardCfgTab[TASKUART_2_PORT].PortType)
	{
	case 4:
		UART_Custom[TASKUART_2_PORT] = 0;                      //判断是普通的modbus中断还是普通中断
		eMBMSerialInit(&xMBMMaster2,                        //普通modbus
			MB_RTU,
			3,
			COM_BautRate,
			MB_PAR_NONE);
		if(BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum > 15)
		{
		UART_delay[TASKUART_2_PORT] = 150;   // 可改
		}
		else
		{
		UART_delay[TASKUART_2_PORT] = 900 - BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum * 50;   // 可改	
		}
		break;

	case 1:
		UART_Custom[TASKUART_2_PORT] = 1;
		CustomSerialInit(3,                                   //道岔阻力模块
			COM_BautRate,
			BoardCfgTab[2].DataBit,
			BoardCfgTab[2].Parity,
			BoardCfgTab[2].StopBit);
		UART_delay[TASKUART_2_PORT] = 900 - BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum * 25;
		break;

	default:
		UART_delay[TASKUART_2_PORT] = 440;
		break;
	}

	SerialDataBuf[TASKUART_2_PORT][0] = BoardCfgTab[TASKUART_2_PORT].PortNum;
	SerialDataBuf[TASKUART_2_PORT][1] = BoardCfgTab[TASKUART_2_PORT].MainProType;
	SerialDataBuf[TASKUART_2_PORT][2] = BoardCfgTab[TASKUART_2_PORT].SubProType;
	SerialDataBuf[TASKUART_2_PORT][3] = 0x00;
	SerialDataBuf[TASKUART_2_PORT][4] = BoardCfgTab[TASKUART_2_PORT].FenjiNum;
	ALARM_TEMP[TASKUART_2_PORT] = 0Xffff;

	DC_electricity_SSZ = (uint16_t *)(SDRAM_BASE_ADDR + 0X2000);
	DC_electricity_QX = (uint16_t *)(SDRAM_BASE_ADDR + 0x5000 + 0X3C000);
	while (1)
	{
		WDTUART2FLAG = 0;
		temp = (CPU_INT16U*)&SerialDataBuf[TASKUART_2_PORT][11];               //7字节包头，4字节板件状态
		Uart_temp = 502;
		len = 0;
		DC_address_temp = 0;
		if(BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum==0)
		{
			osDelay(50);
			continue;
		}
		for (m = 0; m < BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum; m++)
		{
			if (changetime[2] != 0) Task2AdjustNum++;

			switch (BoardCfgTab[TASKUART_2_PORT].SlaveModuelType[m])
			{
			case FLFXG_XHJ:
			case FLFXG_XHJ_KGL:
				modbus_number = 13;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					// if (BoardCfgTab[TASKUART_2_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 22);
					// 	//Uart_temp += 11;
					// }

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// if (BoardCfgTab[TASKUART_2_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	for (n = 0; n < (modbus_number - 2); n++)
					// 	{
					// 		//MBRegHoldingBuf[Uart_temp] = 0;
					// 		//Uart_temp++;
					// 	}
					// }

				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



case FLFXG_NEW_XHJ:
				modbus_number=50;
				if(MB_ENOERR ==eMBMReadInputRegisters( xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp ))
					{		     
						Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);               					//1???????0?????
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					// if (BoardCfgTab[TASKUART_2_PORT].SlaveModuelType[m] == FLFXG_XHJ)
					// {
					// 	//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 96);
					// 	//Uart_temp += 48;
					// }
					temp = temp + modbus_number;																						
					}
				else
					{	
						if (Modual_ststus_temp[m] > 2)
						{
							Modual_ststus_temp[m] = 0;
							*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
							//ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);

							//MBRegHoldingBuf[500] &= ~(1 << m);
						}
						else
						{
							Modual_ststus_temp[m]++;
						}

						for (n = 0; n < modbus_number; n++)
						{
							*temp++ = 0xffff;
						}

						//MBRegHoldingBuf[501] &= ~(1 << m);
						
						// for (n = 0; n < (modbus_number - 2); n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }
						
					}

				len +=(modbus_number*2);
				osDelay (5);
				break;


			case FLFXG_GD_25_J:
				modbus_number = 34;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 64);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


				case FLFXG_GD_25_J_HIGH:
			    modbus_number = 133; 
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, 133, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 262);	   //20191125???

					//Uart_temp += 131;

					temp = temp + 133;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						// ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);												 
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}				
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_DC_AC:
				modbus_number = 10;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 16);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;



			case FLFXG_GD_25_F:
				modbus_number = 30;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 56);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


				case FLFXG_GD_25_F_HIGH:
				modbus_number = 117;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 224);	      //20191125???
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}
					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_CL:
				modbus_number = 8;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 12);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;

				case FLFXG_CL_HIGH:
				modbus_number = 62;
				if (MB_ENOERR == eMBMReadInputRegisters(xMBMMaster2, BoardCfgTab[2].SlaveModuelAddress[m], 0, modbus_number, temp))
				{
					Modual_ststus_temp[m] = 0;
					*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

					if (((*(temp + 1)) & 0x3f) == 0)
						ALARM_TEMP[TASKUART_2_PORT] |= (1 << m);
					else
						ALARM_TEMP[TASKUART_2_PORT] &= ~(1 << m);

					//MBRegHoldingBuf[500] |= (1 << m);

					// if (*(temp + 1) == 0)
					// 	//MBRegHoldingBuf[501] |= (1 << m);
					// else
					// 	//MBRegHoldingBuf[501] &= ~(1 << m);

					//memcpy(&MBRegHoldingBuf[Uart_temp], temp + 2, 120);
					//Uart_temp += (modbus_number - 2);

					temp = temp + modbus_number;
				}
				else
				{
					if (Modual_ststus_temp[m] > 2)
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//												 ALARM_TEMP[TASKUART_2_PORT]&= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					for (n = 0; n < modbus_number; n++)
					{
						*temp++ = 0xffff;
					}

					//MBRegHoldingBuf[501] &= ~(1 << m);
					// for (n = 0; n < (modbus_number - 2); n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }
				}
				len += (modbus_number * 2);
				osDelay(5);
				break;


			case FLFXG_AC_ZZJ:
				if (changetime[2] == 1)                           				     //校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 400);
				if (evt.status == osEventSignal)
				{
					//												if(SerialDataBuf[TASKUART_2_PORT][13]==(CustomUartHdls[3].recv_len-5))  //判断长度											
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 29);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 58);     //6线制直流道岔29个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 54);	        //20191130修改
						//Uart_temp += 27;

						DC_address_temp += 29;
						len += 58;
						switch (SerialDataBuf[TASKUART_2_PORT][73])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;

								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), 2); //增加转换方向	

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//	{
								//		WDTUART2FLAG = 0;
								//		osDelay(5);				
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 15;
								//	}
								//	port3DLQX = n+1;
								//}
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][75];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][74] == 0)&(SerialDataBuf[TASKUART_2_PORT][75] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][75];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][74] == 0)&(SerialDataBuf[TASKUART_2_PORT][75] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH	
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][76];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][77];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][78];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][79];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][81] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][82 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][282], 2);		//增加转换方向												
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							//-------------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;

							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), 2); //增加转换方向	

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 15, 2);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								{
									WDTUART2FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 15;
								}*/
//								port3DLQX = n + 1;
							}
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//---------------------------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 13; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 29; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 58);
						DC_address_temp += 29;
						len += 58;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 13; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 29; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 58);
					DC_address_temp += 29;
					len += 58;
				}
				osDelay(1);
				break;


			case FLFXG_AC_ZZJ_HIGH:
				if (changetime[2] == 1)                           				     //校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 400);
				if (evt.status == osEventSignal)
				{
					//												if(SerialDataBuf[TASKUART_2_PORT][13]==(CustomUartHdls[3].recv_len-5))  //判断长度											
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 29+39);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 136);     //6线制直流道岔29个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 132);	        //20191130修改
						//Uart_temp += 66;

						DC_address_temp += 68;
						len += 136;
						switch (SerialDataBuf[TASKUART_2_PORT][151])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;

								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 4; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), 2); //增加转换方向	

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//	{
								//		WDTUART2FLAG = 0;
								//		osDelay(5);				
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 15;
								//	}
								//	port3DLQX = n+1;
								//}
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][153];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][152] == 0)&(SerialDataBuf[TASKUART_2_PORT][153] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][153];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][152] == 0)&(SerialDataBuf[TASKUART_2_PORT][153] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH	
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][154];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][155];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][156];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][157];
								for (n = 0; n < 4; n++)
								{
									Flash_time[1] = n + 1;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][159] * 2;
							for (n = 0; n < 4; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][160 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][360], 2);		//增加转换方向												
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							//-------------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;

							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 4; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 3 * 0x400 + 3 + Weibo_number[2][m] / 2), 2); //增加转换方向	

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 15, 2);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								{
									WDTUART2FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 15;
								}*/
//								port3DLQX = n + 1;
							}
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//---------------------------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 52; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 68; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 136);
						DC_address_temp += 68;
						len += 136;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 52; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 68; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 136);
					DC_address_temp += 68;
					len += 136;
				}
				osDelay(1);
				break;


			case  FLFXG_DC_ZZJ4:
				if (changetime[2] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((SerialDataBuf[TASKUART_2_PORT][13] == (CustomUartHdls[3].recv_len - 5))&(CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 8);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 16);     //4线制直流道岔8个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 8;
						len += 16;
						switch (SerialDataBuf[TASKUART_2_PORT][31])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_2_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);

								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//{
								//	WDTUART2FLAG = 0;
								//	osDelay(5);
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 16;
								//}
								//port3DLQX = 1;
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][33];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][32] == 0)&(SerialDataBuf[TASKUART_2_PORT][33] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][40], Weibo_temp);
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][33];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][32] == 0)&(SerialDataBuf[TASKUART_2_PORT][33] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][34];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][35];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][36];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][37];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][39] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][40], Weibo_temp);
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][90], 2);  //增加转换方向	                                                                          //增加转换方向 

							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							//----------------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_2_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);

							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 16, 2);
							/*sendCountOut = TCPSENDCOUNT;
							while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
							{
								WDTUART2FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 16;
							}*/
							//port3DLQX = 1;
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//------------------------------------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }
						for (n = 0; n < 8; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 16);
						DC_address_temp += 8;
						len += 16;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 8; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 16);
					DC_address_temp += 8;
					len += 16;
				}
				osDelay(1);
				break;

case  FLFXG_DC_ZZJ4_HIGH:
				if (changetime[2] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((SerialDataBuf[TASKUART_2_PORT][13] == (CustomUartHdls[3].recv_len - 5))&(CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 23);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 46);     //4线制直流道岔8个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 23;
						len += 46;
						switch (SerialDataBuf[TASKUART_2_PORT][61])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_2_PORT][m] + 8);
								//GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);

								//sendCountOut = TCPSENDCOUNT;
								//while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//{
								//	WDTUART2FLAG = 0;
								//	osDelay(5);
								//	if (sendCountOut>0)sendCountOut--;
								//	else WTD_RESET = 16;
								//}
								//port3DLQX = 1;
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][63];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][62] == 0)&(SerialDataBuf[TASKUART_2_PORT][63] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][70], Weibo_temp);
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][63];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][62] == 0)&(SerialDataBuf[TASKUART_2_PORT][63] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m];
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][64];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][65];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][66];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][67];
								memcpy((DC_electricity_QX + m * 0X1000), &Flash_time[0], 6);
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][69] * 2;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][70], Weibo_temp);
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][120], 2);  //增加转换方向	                                                                          //增加转换方向 

							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							//----------------------------------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000), Weibo_number[TASKUART_2_PORT][m] + 8);
							GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);

							AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 16, 2);
							/*sendCountOut = TCPSENDCOUNT;
							while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
							{
								WDTUART2FLAG = 0;
								osDelay(5);
								if (sendCountOut>0)sendCountOut--;
								else WTD_RESET = 16;
							}*/
							//port3DLQX = 1;
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//------------------------------------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{

						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }
						for (n = 0; n < 23; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 46);
						DC_address_temp += 23;
						len += 46;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 23; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 46);
					DC_address_temp += 23;
					len += 46;
				}
				osDelay(1);
				break;
				

			case	FLFXG_DC_ZZJ6:
				if (changetime[2] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((SerialDataBuf[TASKUART_2_PORT][13] == (CustomUartHdls[3].recv_len - 5))&(CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 11);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 22);     //6线制直流道岔10个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 10);
						//Uart_temp += 5;

						DC_address_temp += 11;
						len += 22;
						switch (SerialDataBuf[TASKUART_2_PORT][37])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_2_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//	{
								//		WDTUART2FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 17;
								//	}
								//	port3DLQX = n+1;
								//}
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][39];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][38] == 0)&(SerialDataBuf[TASKUART_2_PORT][39] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][39];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][38] == 0)&(SerialDataBuf[TASKUART_2_PORT][39] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][40];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][41];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][42];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][43];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][45] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][46 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][146], 2);			//增加转换方向	
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];

							//-----------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_2_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 17, 2);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								{
									WDTUART2FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 17;
								}*/
								//port3DLQX = n + 1;
							}
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 5; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 11; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 22);
						DC_address_temp += 11;
						len += 22;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 5; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 11; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 22);
					DC_address_temp += 11;
					len += 22;
				}
				osDelay(1);
				break;


			case	FLFXG_DC_ZZJ6_HIGH:
				if (changetime[2] == 1)                           				//校正时间
				{
					SendAdjustTimeToModue(TASKUART_2_PORT, 3, m);
					//CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
					//CmdBuf2[1] = 0x10;
					//CmdBuf2[2] = 0x00;
					//CmdBuf2[3] = 0x00;
					//CmdBuf2[4] = 0x00;
					//CmdBuf2[5] = 0x02;
					//CmdBuf2[6] = 0x04;
					//CmdBuf2[7] = (Systemtime >> 24) & 0xff;
					//CmdBuf2[8] = (Systemtime >> 16) & 0xff;
					//CmdBuf2[9] = (Systemtime >> 8) & 0xff;
					//CmdBuf2[10] = Systemtime & 0xff;
					//*((CPU_INT16U *)&CmdBuf2[11]) = CRC16(CmdBuf2, 11);

					//Custom_Uart_Write(3, CmdBuf2, 13);
					//evt = osSignalWait(0x02, 100); 								//发送超时								
					//evt = osSignalWait(0x01, 300);  								//接收超时
					//WDTUART2FLAG = 0;
				}

				CmdBuf2[0] = BoardCfgTab[TASKUART_2_PORT].SlaveModuelAddress[m];
				CmdBuf2[1] = 0x03;
				CmdBuf2[2] = 0x00;
				CmdBuf2[3] = 0x00;
				CmdBuf2[4] = 0x00;
				CmdBuf2[5] = AC_module_infor[TASKUART_2_PORT].Frame_sequence[m];

				*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(CmdBuf2, 6);
				Custom_Uart_Write(3, CmdBuf2, 8);                       //软件串口1口

				evt = osSignalWait(0x02, 100);
				evt = osSignalWait(0x01, 300);
				if (evt.status == osEventSignal)
				{
					*((CPU_INT16U *)&CmdBuf2[6]) = CRC16(&SerialDataBuf[TASKUART_2_PORT][11], (CustomUartHdls[3].recv_len - 2));
					if ((SerialDataBuf[TASKUART_2_PORT][13] == (CustomUartHdls[3].recv_len - 5))&(CmdBuf2[6] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 2])&(CmdBuf2[7] == SerialDataBuf[TASKUART_2_PORT][CustomUartHdls[3].recv_len + 11 - 1]))
					{
						Modual_ststus_temp[m] = 0;
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) |= (1 << m);

						GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14], 26);
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 52);     //6线制直流道岔10个寄存器实时值

						//MBRegHoldingBuf[500] |= (1 << m);
						//memcpy(&MBRegHoldingBuf[Uart_temp], &SerialDataBuf[TASKUART_2_PORT][18], 40);
						//Uart_temp += 20;

						DC_address_temp += 26;
						len += 52;
						switch (SerialDataBuf[TASKUART_2_PORT][67])
						{
						case 0:    //无曲线帧												    											      											
							if ((AC_module_infor[TASKUART_2_PORT].Frame_mark[m] == 0xFF)&(AC_module_infor[TASKUART_2_PORT].Curve_number[m] == 1)&(AC_module_infor[TASKUART_2_PORT].Points_number[m] < 2001))
							{
								AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
								//发送数据
								//*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
								//WDTUART2FLAG = 0;
								//SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
								//for (n = 0; n < 2; n++)
								//{
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								//	memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + Weibo_number[TASKUART_2_PORT][m] / 2), 2);

								//	GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								//	sendCountOut = TCPSENDCOUNT;
								//	while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								//	{
								//		WDTUART2FLAG = 0;
								//		osDelay(5);
								//		if (sendCountOut>0)sendCountOut--;
								//		else WTD_RESET = 17;
								//	}
								//	port3DLQX = n+1;
								//}
								AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
								AC_module_infor[TASKUART_2_PORT].Curve_number[m] = 0;
								EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							}
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0;
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = 0;
							Weibo_number[TASKUART_2_PORT][m] = 0;
							osDelay(1);
							break;

						case 0x55:			//过程曲线帧								   
							AC_module_infor[TASKUART_2_PORT].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][69];
							AC_module_infor[TASKUART_2_PORT].Frame_mark[m] = 0x55;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][68] == 0)&(SerialDataBuf[TASKUART_2_PORT][69] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH																
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}

							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						case 0xff:				//结束帧								   											    
							AC_module_infor[2].Frame_sequence[m] = SerialDataBuf[TASKUART_2_PORT][69];
							AC_module_infor[2].Frame_mark[m] = 0xFF;

							//写时间 
							if ((SerialDataBuf[TASKUART_2_PORT][68] == 0)&(SerialDataBuf[TASKUART_2_PORT][69] == 1))
							{
								Weibo_number[2][m] = 0;
								AC_module_infor[2].Curve_number[m] = 1;
								//														BSP_SSP1_Flash_Erase(0x10000*m);       								  //清除FLASH		
								Flash_time[1] = 5;
								Flash_time[2] = SerialDataBuf[TASKUART_2_PORT][70];
								Flash_time[3] = SerialDataBuf[TASKUART_2_PORT][71];
								Flash_time[4] = SerialDataBuf[TASKUART_2_PORT][72];
								Flash_time[5] = SerialDataBuf[TASKUART_2_PORT][73];
								for (n = 0; n < 2; n++)
								{
									Flash_time[0] = BoardCfgTab[2].SlaveModuelAddress[m] + n;
									memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400), &Flash_time[0], 6);
								}
							}
							Weibo_temp = SerialDataBuf[TASKUART_2_PORT][75] * 2;
							for (n = 0; n < 2; n++)
							{
								memcpy((DC_electricity_QX + m * 0X1000 + n * 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][76 + n * 50], Weibo_temp);
							}
							Weibo_number[2][m] += Weibo_temp;
							memcpy((DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[2][m] / 2), &SerialDataBuf[TASKUART_2_PORT][176], 2);			//增加转换方向	
							AC_module_infor[2].Points_number[m] = Weibo_number[2][m];

							//-----------
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = Weibo_number[TASKUART_2_PORT][m] + 12;
							WDTUART2FLAG = 0;
							SerialDataBuf[TASKUART_2_PORT][3] = 0X00;                                //表示是曲线帧
							for (n = 0; n < 2; n++)
							{
								memcpy(&SerialDataBuf[TASKUART_2_PORT][11], (DC_electricity_QX + m * 0X1000 + n * 0x400), Weibo_number[TASKUART_2_PORT][m] + 6);
								memcpy(&SerialDataBuf[TASKUART_2_PORT][17 + Weibo_number[TASKUART_2_PORT][m]], (DC_electricity_QX + m * 0X1000 + 0x400 + 3 + Weibo_number[TASKUART_2_PORT][m] / 2), 2);

								GAO_di_change((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][11], (Weibo_number[TASKUART_2_PORT][m] + 8) / 2);
								
								AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 17, 2);
								/*sendCountOut = TCPSENDCOUNT;
								while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], Weibo_number[2][m] + 19, 0) < 0)
								{
									WDTUART2FLAG = 0;
									osDelay(5);
									if (sendCountOut>0)sendCountOut--;
									else WTD_RESET = 17;
								}*/
								//port3DLQX = n + 1;
							}
							SendLogToServer(3, 5);  //端口3发送电流曲线
							//------------
							EEPROM_Write(0, 9, (void*)&AC_module_infor[TASKUART_2_PORT].Curve_number[0], MODE_8_BIT, 100);
							osDelay(1);
							break;

						default:
							break;
						}
					}
					else
					{
						// for (n = 0; n < 20; n++)
						// {
						// 	//MBRegHoldingBuf[Uart_temp] = 0;
						// 	//Uart_temp++;
						// }

						for (n = 0; n < 26; n++)
						{
							*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
						}
						memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 52);
						DC_address_temp +=26;
						len += 52;
					}
				}
				else
				{
					if (Modual_ststus_temp[m] > 3)
					{
						*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) &= ~(1 << m);
						//MBRegHoldingBuf[500] &= ~(1 << m);
					}
					else
					{
						Modual_ststus_temp[m]++;
					}

					// for (n = 0; n < 20; n++)
					// {
					// 	//MBRegHoldingBuf[Uart_temp] = 0;
					// 	//Uart_temp++;
					// }

					for (n = 0; n < 26; n++)
					{
						*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][14 + 2 * n]) = 0XFFFF;
					}
					memcpy((DC_electricity_SSZ + DC_address_temp), &SerialDataBuf[TASKUART_2_PORT][14], 52);
					DC_address_temp += 26;
					len += 52;
				}
				osDelay(1);
				break;

			default:
				break;

			}
			WDTUART2FLAG = 0;
		}

		
		if (ALARM_TEMP[TASKUART_2_PORT] != 0xffff)      //驱动报警
		{
			GPIO_PinWrite(2, 0, 1);
			ALARM |= (1 << TASKUART_2_PORT);
		}
		else
		{
			GPIO_PinWrite(2, 0, 0);
			ALARM &= ~(1 << TASKUART_2_PORT);
		}

		if ((*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7])) != ((1 << BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum) - 1))
		{
			GPIO_PinWrite(2, 5, 1);
		}
		else
		{
			GPIO_PinWrite(2, 5, 0);
		}
		if (Task2AdjustNum >= BoardCfgTab[TASKUART_2_PORT].SlaveModuelNum && changetime[2] != 0)   // BUG:判断条件反了
		{
			changetime[2] = 0;
		}

		//端口重启，当端口里全部模块都为超时时，端口重置。连续重置3次还未恢复则为真的超时
		if (*((CPU_INT32U *)&SerialDataBuf[TASKUART_2_PORT][7]) == 0)
		{
			erro_number++;
			if (erro_number < 4)
			{
				switch (BoardCfgTab[TASKUART_2_PORT].PortType)
				{
				case 4:
						eMBMSerialInit(&xMBMMaster2,  MB_RTU,    3,   COM_BautRate,  MB_PAR_NONE);             //???modbus
						port3Error = 1;
						break;
				case 1:
					CustomSerialInit(3, COM_BautRate, BoardCfgTab[2].DataBit, BoardCfgTab[2].Parity, BoardCfgTab[2].StopBit);
					port3Error = 1;
					break;

				default:
					break;
				}
			}
			else
			{
				WTD_RESET = 18;
				if (port3Error == 0) port3Error = 2;
			}
		}
		else
		{
			port3Error = 0;
			erro_number = 0;
		}

		switch (BoardCfgTab[TASKUART_2_PORT].PortType)
		{
		case 1:                                              				//阻力模块发送模块状态								           
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = len + 4;	//数据长度实时值
			SerialDataBuf[TASKUART_2_PORT][3] = 0X01;                       //表示是曲线帧						             	
			memcpy(&SerialDataBuf[TASKUART_2_PORT][11], DC_electricity_SSZ, len);
			
			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], len + 11, 19, 2);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], len + 11, 0) < 0)
			{
				WDTUART2FLAG = 0;
				osDelay(5);
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 19;
			}*/
			break;

		case  4:					    	                            // 正常modbus轮训完发送数据
			*((CPU_INT16U *)&SerialDataBuf[TASKUART_2_PORT][5]) = len + 4;
			
			AddToTcpSendBuf((char *)&SerialDataBuf[TASKUART_2_PORT][0], len + 11, 20, 2);
			/*sendCountOut = TCPSENDCOUNT;
			while (send(sock, (char *)&SerialDataBuf[TASKUART_2_PORT][0], len + 11, 0) < 0)
			{
				WDTUART2FLAG = 0;
				osDelay(10);				
				if (sendCountOut>0)sendCountOut--;
				else WTD_RESET = 20;
			}*/
			break;

		default:
			break;

		}
		osDelay(UART_delay[TASKUART_2_PORT]);
	}
}

/*----------------------------------------------------------------------------
  Thread 'Master': 
 *---------------------------------------------------------------------------*/
static void App_Taskserial_slave(void const *arg)      
{
	xMBSHandle       xMBSlave;
	CPU_INT32U  		COM_BautRate = 19200;
	//        CPU_INT16U	    m=0;

	//	        for(m=0;m<MB_REG_HOLDING_NREGS;m++)
	//	        {
	//		       MBRegHoldingBuf[m]=0Xffff;
	//					}
	UART_Custom[TASKUART_3_PORT] = 0;                      //判断是普通的modbus中断还是普通中断
	eMBSSerialInit(&xMBSlave, MB_RTU, Adress_ip[3], 2, COM_BautRate, MB_PAR_NONE); // 通信板下面的彩屏通信modbus

	if (MB_ENOERR != eMBSRegisterInputCB(xMBSlave, eMBSRegInputCB))
	{
		(void)eMBSClose(xMBSlave);
	}

	if (MB_ENOERR != eMBSRegisterHoldingCB(xMBSlave, eMBSRegHoldingCB))
	{
		(void)eMBSClose(xMBSlave);
	}


	while (1)
	{
		WDTUART3FLAG = 0;
		eMBSPoll(xMBSlave);					            /* Poll the communication stack. */
		osDelay(1);
	}
}
/*----------------------------------------------------------------------------
  Thread 'Server': BSD Server socket process
 *---------------------------------------------------------------------------*/

static void App_TaskSave(void const *arg)
{
	osEvent         evt;
	while (1)
	{
		evt = osSignalWait(0x01, osWaitForever);   // 阻塞等待
		if (evt.status == osEventSignal)
		{
		}
	}
}
/*----------------------------------------------------------------------------
  Thread 'Server': BSD Server socket process
 *---------------------------------------------------------------------------*/
static void App_TaskStart(void const *arg)
{

	CPU_INT08U  cnts;
	CPU_INT16U  number = 3000;
	while (1)
	{
		cnts++;
		heartbeat++;
		if (download_flag == 1)   //上位机的配置信息已下发
		{
			number = 1000;
			if (cnts > 50)
			{
				cnts = 0;
				BSP_LED_Toggle(2, 13);
				BSP_LED_Toggle(2, 4);
			}

			if (ALARM != 0)
			{
				GPIO_PinWrite(2, 11, 1);
			}
			else
			{
				GPIO_PinWrite(2, 11, 0);
			}
		}
		else
		{
			if (cnts > 10)
			{
				cnts = 0;
				BSP_LED_Toggle(2, 13);
				BSP_LED_Toggle(2, 4);
			}
		}
		WDTUART0FLAG++;
		WDTUART1FLAG++;
		WDTUART2FLAG++;
		WDTUART3FLAG++;
		WDTTCPSENDFLAG++;
		if (WDTUART0FLAG > 500) WTD_RESET = 21;
		if (WDTUART1FLAG > 500) WTD_RESET = 22;
		if (WDTUART2FLAG > 500) WTD_RESET = 23;
		if (WDTUART3FLAG > 500) WTD_RESET = 24;
		if (WDTTCPSENDFLAG > 500) WTD_RESET = 26;
		if (heartbeat > number) WTD_RESET = 25;
		if (WTD_RESET == 0)
		{
			WatchDog_Clear();  //喂狗.
		}
		else
		{
			//板子复位
			if (logWTD_RESET == 0)
			{
				logWTD_RESET = 1;
			}
		}
		osDelay(10);
	}
}

/*************************************************************************************
									App_ObjCreate()
 Description : Create application kernel objects tasks.
 Argument(s) : none
 Return(s)   : none
 Caller(s)   : AppTaskStart()
 Note(s)     : none.
*************************************************************************************/
static void App_ObjCreate(void)
{
//	 EEPROM_Read(0,0,(void*)CoefficientBuf,MODE_16_BIT,(FULL_CALIBRATION-10));			
// 	 memcpy(&MBRegHoldingBuf[0], CoefficientBuf, 264);	
//	 memcpy(&MBRegHoldingBuf[MB_REG_CONFIG_START], &CoefficientBuf[132], 18);
//	 EEPROM_Read(0,5,(void*)&CoefficientBuf[142],MODE_16_BIT,10);	
//	 memcpy(&MBRegHoldingBuf[208], &CoefficientBuf[142], 20);

		  
}

/*----------------------------------------------------------------------------
  Main Thread 'main': Run Network
 *---------------------------------------------------------------------------*/
int main(void)
{

	BSP_PreInit();
	App_ObjCreate();

	MYMAC[5] = BSP_BoardAddr_Read();
	memcpy(eth0_config.MacAddr, MYMAC, 6);
	Adress_ip[3] = MYMAC[5];
	memcpy(nlocalm.IpAddr, Adress_ip, 4);
	net_initialize();
	BSP_TIMER1_Init();
	pStart = 1;
	//	  Sqr_number_ptr = (uint16_t *)(SDRAM_BASE_ADDR+0X4f000);
	// Create networking task	
	App_TaskClien_id = osThreadCreate(osThread(App_TaskClient), NULL);  // 与上位机的通信任务
	App_TaskStart_id = osThreadCreate(osThread(App_TaskStart), NULL);  // 任务监测超时与看门狗喂狗
	App_TaskSave_id = osThreadCreate(osThread(App_TaskSave), NULL);    
	App_Taskserial_0_id = osThreadCreate(osThread(App_Taskserial_0), NULL);
	App_Taskserial_1_id = osThreadCreate(osThread(App_Taskserial_1), NULL);
	App_Taskserial_2_id = osThreadCreate(osThread(App_Taskserial_2), NULL);
	App_Taskserial_slave_id = osThreadCreate(osThread(App_Taskserial_slave), NULL); // 彩屏modbus通信
	App_Task_TcpSend_id = osThreadCreate(osThread(App_Task_TcpSend), NULL);   // tcp发送任务

	while (1)
	{
		net_main();
		osThreadYield();
	}
}

/*************************************************************************************
									USER FUNCTIONS
*************************************************************************************/
//Modbus Slave functions

static eMBException
eMBSRegHoldingCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs, eMBSRegisterMode eRegMode)
{
	eMBException    eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
	USHORT          usRegIndex;

	if ((usAddress >= MB_REG_HOLDING_START) &&
		(usAddress + usNRegs <= MB_REG_HOLDING_START + MB_REG_HOLDING_NREGS))
	{
		usRegIndex = (USHORT)(usAddress - MB_REG_HOLDING_START);

		switch (eRegMode)
		{
			/* Pass current register values to the protocol stack. */
		case MBS_REGISTER_READ:
			while (usNRegs > 0)
			{
				*pubRegBuffer++ = (UCHAR)(MBRegHoldingBuf[usRegIndex] >> 8);
				*pubRegBuffer++ = (UCHAR)(MBRegHoldingBuf[usRegIndex] & 0xFF);
				usRegIndex++;
				usNRegs--;
			}
			break;

			/* Update current register values with new values from the protocol stack. */
		case MBS_REGISTER_WRITE:
			while (usNRegs > 0)
			{
				MBRegHoldingBuf[usRegIndex] = *pubRegBuffer++ << 8;
				MBRegHoldingBuf[usRegIndex] |= *pubRegBuffer++;
				usRegIndex++;
				usNRegs--;
			}
			break;
		}

		eException = MB_PDU_EX_NONE;
	}

	return (eException);
}


static eMBException
eMBSRegInputCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs)
{
	eMBException    eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
	USHORT          usRegIndex;

	if ((usAddress >= MB_REG_INPUT_START) &&
		(usAddress + usNRegs <= MB_REG_INPUT_START + MB_REG_INPUT_NREGS))
	{
		usRegIndex = (USHORT)(usAddress - MB_REG_INPUT_START);

		while (usNRegs > 0)
		{
			*pubRegBuffer++ = (UCHAR)(MBRegInputBuf[usRegIndex] >> 8);
			*pubRegBuffer++ = (UCHAR)(MBRegInputBuf[usRegIndex] & 0xFF);
			usRegIndex++;
			usNRegs--;
		}

		eException = MB_PDU_EX_NONE;
	}

	return (eException);
}

