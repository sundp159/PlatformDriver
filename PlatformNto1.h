

#ifndef PLATFORMNTO1_H_
#define PLATFORMNTO1_H_

#include "stdio.h"
#include "tistdtypes.h"
#include "cslr_device.h"
#include "cslr_uart.h"
#include "cslr_tpcc.h"
#include <cslr_cgem.h>
/*************************************************************************************************************/
//lib版本查询
/*************************************************************************************************************/
/*****************************************************************************
 Prototype    : LibVersion
 Description  : lib当前版本查询。
 Input        : 无
 Output       : 无
 Return Value : unsigned int，版本号。
*****************************************************************************/
extern unsigned int LibVersion(void);

/*************************************************************************************************************/
//设备平台硬件初始化
/*************************************************************************************************************/


/*****************************************************************************
 Prototype    : PlatformNto1_Init
 Description  : 平台初始化函数，包括mainPLL、DDR3初始化，配置为内核时钟1000MHz，DDR3为1000MHz。
 Input        : 无
 Output       : 无
 Return Value : 1为初始化成功。
*****************************************************************************/
extern int PlatformNto1_Init(void);






/************************************************************************************************************/
//GPIO
/************************************************************************************************************/
/* GPIO pins */
#define	GPIO_0					(0)
#define	GPIO_1					(1)
#define	GPIO_2					(2)
#define	GPIO_3					(3)
#define	GPIO_4					(4)
#define	GPIO_5					(5)
#define	GPIO_6					(6)
#define	GPIO_7					(7)
#define	GPIO_8					(8)
#define	GPIO_9					(9)
#define	GPIO_10					(10)
#define	GPIO_11					(11)
#define	GPIO_12					(12)
#define	GPIO_13					(13)
#define	GPIO_14					(14)
#define	GPIO_15					(15)
#define	GPIO_MAX_NUM			GPIO_15
/*****************************************************************************
 Prototype    : GPIO_init
 Description  : GPIO初始化
 Input        : unsigned short Dir,采用位图表示IO方向，LSB为GPIO0；0为输出，1为输入；初始化默认输出值为0。
 	 	 	 	unsigned char IntEn, 中断使能，1使能，0关闭
 	 	 	 	unsigned char TrigMode, 中断触发方式，0，上升沿，1下降沿
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void GPIO_init(unsigned short Dir, unsigned char IntEn, unsigned char TrigMode);
/*****************************************************************************
 Prototype    : Set_GPIO
 Description  : GPIO端口置1
 Input        : unsigned char num，GPIO端口号。
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void Set_GPIO(unsigned char num);
/*****************************************************************************
 Prototype    : Clr_GPIO
 Description  : GPIO端口置0
 Input        : unsigned char num，GPIO端口号。
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void Clr_GPIO(unsigned char num);




/************************************************************************************************************/
//GBE
/************************************************************************************************************/

/*****************************************************************************
 Prototype    : GBE_ConfigSGMIISerdes
 Description  : 配置SERDES
 Input        : 无
 Output       : 无
 Return Value : 配置成功返回1。
*****************************************************************************/
extern int GBE_ConfigSGMIISerdes(void);
/*****************************************************************************
 Prototype    : GBE_InitEthernetSGMII
 Description  : 配置SGMII
 Input        : unsigned int macPortNum，选择MAC端口，0或者1。
 Output       : 无
 Return Value : 配置成功返回1。
*****************************************************************************/
extern int GBE_InitEthernetSGMII(unsigned int macPortNum);



/************************************************************************************************************/
//SRIO
/************************************************************************************************************/
//SRIO设备ID
#define DSP0_SRIO_BASE_ID	0x08
#define DSP1_SRIO_BASE_ID	0x09
#define DSP2_SRIO_BASE_ID   0x00
#define DSP3_SRIO_BASE_ID   0x03
#define K7_SRIO_BASE_ID     0x06
/*****************************************************************************
 Prototype    : SRIOInterrupts_Init
 Description  : SRIO doorbell中断配置，在BIOS环境下配置完此函数后要参照以下代码配置CORE中断
					Hwi_Params hwiParams;
					Hwi_Handle myHwi;
					Hwi_Params_init(&hwiParams);
					hwiParams.eventId = CSL_GEM_INTDST_N_PLUS_16;
					hwiParams.enableInt = TRUE;
					myHwi = Hwi_create(5, (ti_sysbios_hal_Hwi_FuncPtr)SRIO_Doorbell_ISR, &hwiParams, NULL);//中断5，中断服务函数SRIO_Doorbell_ISR，需用户自己实现
					if (myHwi == NULL){
						printf("Hwi create failed\n");
					}
 Input        : 无
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void SRIOInterrupts_Init(void);
/*****************************************************************************
 Prototype    : SRIO_Init
 Description  : SRIO初始化。注：必须在SRIOInterrupts_Init函数之后运行！
 Input        : unsigned char DspSrioId:DSP的SRIO源ID号，各自DSP要配置成自己的ID号。
 Output       : 无
 Return Value : 配置成功返回1。
*****************************************************************************/
extern int SRIO_Init(unsigned char DspSrioId);
/*****************************************************************************
 Prototype    : SRIO_Send
 Description  : SRIO发送函数
 Input        : unsigned int source_address,发送数据源地址
				unsigned int dest_address,发送数据目标地址
				unsigned int send_size,数据大小，最大1MB
				unsigned char desID,发送设备目标ID
 Output       : 无
 Return Value : 发送成功返回1。
*****************************************************************************/
extern int SRIO_Send(unsigned int source_address,unsigned int dest_address,unsigned int send_size, unsigned char desID);
/*****************************************************************************
 Prototype    : SRIO_Send_DoorBell
 Description  : SRIO发送doorbell
 Input        : unsigned int uiDestID, 目标ID
 	 	 	 	unsigned int uiDoorBellInfo，doorbell信息
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void SRIO_Send_DoorBell(unsigned int uiDestID, unsigned int uiDoorBellInfo);
/*****************************************************************************
 Prototype    : SRIO_Read_DoorBell
 Description  : SRIO读doorbell，注，该函数应在doorbell中断函数中使用。
 Input        : 无
 Output       : 无
 Return Value : doorbell信息
*****************************************************************************/
extern unsigned int SRIO_Read_DoorBell(void);



/************************************************************************************************************/
//TIMER
/************************************************************************************************************/

typedef enum
{
	TIMER64_ONE_SHOT_PULSE = 0, //单次定时器
	TIMER64_PERIODIC_PULSE, 	//周期性脉冲定时器
	TIMER64_PERIODIC_CLOCK, 	//周期性时钟定时器
	TIMER64_WATCH_DOG 		    //看门狗
}Timer64Mode;

typedef struct  {
	int timer_num; 				//定时器号
	Timer64Mode timerMode; 		//定时器模式
	unsigned long long period; 	//定时器周期
}Timer64Config;

/*****************************************************************************
 Prototype    : delay_cycles
 Description  : 时钟周期延时函数
 Input        : unsigned long long cycles，延时周期
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void delay_cycles( unsigned long long cycles);
/*****************************************************************************
 Prototype    : delay_us
 Description  : us延时函数
 Input        : 延时时间，us
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void delay_us( unsigned int us);
/*****************************************************************************
 Prototype    : delay_ms
 Description  : ms延时函数
 Input        : 延时时间，ms
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void delay_ms( unsigned int ms);
/*****************************************************************************
 Prototype    : Timer64Local_Init
 Description  : 定时器初始化函数，初始化完成后立即开始计数。
 Input        : Timer64Config config，定时器配置，注：最小单位为us
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void Timer64Local_Init(Timer64Config config);
/*****************************************************************************
 Prototype    : Timer64Local_Reset
 Description  : 定时器复位，计数清零，停止计数
 Input        : Timer64Config config，为初始化时的定时器配置。
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void Timer64Local_Reset(Timer64Config config);
/*****************************************************************************
 Prototype    : Timer64Local_InterruptCore_Init
 Description  : 定时器内核中断配置，在无BIOS环境下使用。当在BIOS环境下时，参照以下函数配置中断，
					Hwi_Params hwiParams;
					Hwi_Handle myHwi;
					Hwi_Params_init(&hwiParams);
					hwiParams.eventId = CSL_GEM_TINTLN;//本地定时器中断
					hwiParams.enableInt = TRUE;
					myHwi = Hwi_create(6, (ti_sysbios_hal_Hwi_FuncPtr)Timer64Local_ISR, &hwiParams, NULL);//中断6，中断函数为Timer64Local_ISR，用户自己实现
					if (myHwi == NULL)
					{
						printf("Hwi create failed\n");
					}
 Input        : unsigned int IntNum，CPU中断号
 Output       : 无
 Return Value : 配置成功为1.
*****************************************************************************/
extern int Timer64Local_InterruptCore_Init(unsigned int IntNum);


/************************************************************************************************************/
//内核中断配置相关函数，在无BIOS环境下使用
/************************************************************************************************************/

/*****************************************************************************
 Prototype    : InterruptCore_GlobleInit
 Description  : 内核中断配置初始化，在无BIOS环境下使用，配置中断时首先要调用该函数
 Input        : 无
 Output       : 无
 Return Value : 信息
*****************************************************************************/
extern void InterruptCore_GlobleInit(void);
/*****************************************************************************
 Prototype    : InterruptCore_GlobleEn
 Description  : 内核中断全部使能，在无BIOS环境下使用，要在所有中断配置完成后，最后调用该函数
 Input        : 无
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void InterruptCore_GlobleEn(void);




/************************************************************************************************************/
//UART
/************************************************************************************************************/
typedef enum
{
	UART_USE_CORE_TO_TX = 0,
	UART_USE_EDMA_TO_TX = 1
}UART_Tx_Master;
typedef enum
{
	TRIGGER_LEVEL_1BYTE = CSL_UART_FCR_RXFIFTL_CHAR1,
	TRIGGER_LEVEL_4BYTE = CSL_UART_FCR_RXFIFTL_CHAR4,
	TRIGGER_LEVEL_8BYTE = CSL_UART_FCR_RXFIFTL_CHAR8,
	TRIGGER_LEVEL_14BYTE = CSL_UART_FCR_RXFIFTL_CHAR14
}UART_triLevel;
typedef enum
{
	OVER_SAMPLING_16X = CSL_UART_MDR_OSM_SEL_16XOVERSAMPLING,
	OVER_SAMPLING_13X = CSL_UART_MDR_OSM_SEL_13XOVERSAMPLING
}UART_overSampling;
typedef enum
{
	AUTO_FLOW_DIS = 0,
	AUTO_FLOW_CTS_EN = 1,
	AUTO_FLOW_RTS_CTS_EN = 2
}UART_autoFlow;
typedef enum
{
	ONE_STOP_BIT = CSL_UART_LCR_STB_1BIT,
	WLS_STOP_BIT = CSL_UART_LCR_STB_WLS
}UART_stopMode;
typedef enum
{

	DATA_LEN_5BIT = CSL_UART_LCR_WLS_5BITS,
	DATA_LEN_6BIT = CSL_UART_LCR_WLS_6BITS,
	DATA_LEN_7BIT = CSL_UART_LCR_WLS_7BITS,
	DATA_LEN_8BIT = CSL_UART_LCR_WLS_8BITS
}UART_dataLength;
typedef enum
{
	PARITY_DISABLE = 0,
	ODD_PARITY_ENABLE_SET1 = 1,
	EVEN_PARITY_ENABLE_SET1 = 2,
	STICK_PARITY_ENABLE_SET = 3,
	STICK_PARITY_ENABLE_CLR = 4
}UART_parityMode;
typedef struct
{
	unsigned int 		baudRate;//波特率
	unsigned int  		DSP_Core_Speed_Hz;//DSP内核时钟
	UART_parityMode 	parityMode;//奇偶校验方式
	UART_stopMode   	stopMode;//停止位
	UART_dataLength 	dataLen;//数据位
	UART_autoFlow   	autoFlow;//Flow
	UART_overSampling	osmSel;//采样方式
	UART_triLevel   	fifoRxTriBytes;//中断触发字节数
    UART_Tx_Master      txMaster;//传输方式，DMA/core
    Bool 		        bLoopBackEnable;//回环
}UART_Config;

/*****************************************************************************
 Prototype    : Uart_Init
 Description  : 串口初始化
 Input        : UART_Config *UARTCfg，串口配置
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void Uart_Init(UART_Config *UARTCfg);
/*****************************************************************************
 Prototype    : UartInterrupt_Init
 Description  : 串口中断初始化(CIC部分),内核部分需要在bios中进行配置（包括中断号、中断函数），中断配置内核部分配置参照以下，
					Hwi_Params hwiParams;
					Hwi_Handle myHwi;
					Hwi_Params_init(&hwiParams);
					hwiParams.eventId = CSL_GEM_INTC0_OUT0_OR_INTC1_OUT0;
					hwiParams.enableInt = TRUE;
					myHwi = Hwi_create(4, (ti_sysbios_hal_Hwi_FuncPtr)KeyStone_UART_Rx_ISR, &hwiParams, NULL);//中断4，中断服务函数KeyStone_UART_Rx_ISR，要用户自行实现
					if (myHwi == NULL)
						printf("Hwi create failed\n");
 Input        : 无
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void UartInterrupt_Init(void);
/*****************************************************************************
 Prototype    : UartWrite
 Description  : 串口发送函数
 Input        : unsigned char*buff,发送数据的指针
 	 	 	 	unsigned int len，发送的数据量
 Output       : 无
 Return Value : 无
*****************************************************************************/
extern void UartWrite(unsigned char*buff,unsigned int len);
/*****************************************************************************
 Prototype    : UartRead
 Description  : 串口读取函数
 Input        :	unsigned int buffByteLen,读取的数据长度
 Output       : unsigned char *buffer,读取的数据
 Return Value : 读取的数据量
*****************************************************************************/
extern unsigned int UartRead(unsigned char *buffer, unsigned int buffByteLen);

#endif /* PLATFORMNTO1_H_ */
