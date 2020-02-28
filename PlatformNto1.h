

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





#endif /* PLATFORMNTO1_H_ */
