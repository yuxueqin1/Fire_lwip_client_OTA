/*
*********************************************************************************************************
*
*                                       APPLICATION CONFIGURATION
*
*                                             NXP LPC18xx
*                                               on the
*                                    KEIL MCB1800 Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/


/***************************************************************************************
							  MODBUS REGISTER SIZES
***************************************************************************************/

#define  CONFIG_PORT						    0x00
#define TASKUART_0_PORT             ( 0 )
#define TASKUART_1_PORT             ( 1 )
#define TASKUART_2_PORT             ( 2 )
#define TASKUART_3_PORT             ( 3 )

#define  MB_REG_HOLDING_START					0
#define  MB_REG_HOLDING_NREGS					800

#define  MB_REG_INPUT_START						0
#define  MB_REG_INPUT_NREGS						50

#define  FLFXG_XHJ_KGL              0x31              //13   信号机采样模块代替采开关量
#define  FLFXG_XHJ                  0x32             	//13   信号机采样模块
#define	 FLFXG_GD_25_J							0x33             	//19   25HZ轨道接收
#define	 FLFXG_DC_AC							  0x34             	//10   道岔表示电压模块
#define	 FLFXG_GD_25_F					    0x35            	//15   25HZ发送
#define	 FLFXG_CL					          0x36            	//8    6线3对模块采样协议场联，站联
#define	 FLFXG_AC_ZZJ					      0x37              //交流道岔电流曲线加表示电压
#define	 FLFXG_DC_ZZJ4					    0x38              //直流道岔电流采集加表示电压
#define	 FLFXG_DC_ZZJ6					    0x39              //直流道岔电流采集加表示电压6线制

#define	 FLFXG_NEW_XHJ					    	0x47     //71  高频信号机        
#define	 FLFXG_GD_25_J_HIGH				    	0x48	 //72  高频25Hz接收		
#define	 FLFXG_GD_25_F_HIGH				    	0x49     //73  高频25Hz发送
#define	 FLFXG_CL_HIGH							0x4A     //74  高频场联
#define	 FLFXG_AC_ZZJ_HIGH						0x4B	 //75  高频交流道岔
#define	 FLFXG_DC_ZZJ4_HIGH						0x4C     //76  高频四线直流道岔
#define	 FLFXG_DC_ZZJ6_HIGH						0x4D     //77  高频六线直流道岔
