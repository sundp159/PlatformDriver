#include <c6x.h>
#include <stdio.h>
#include <string.h>
#include <cslr_gpio.h>
#include <csl_bootcfgAux.h>
#include "KeyStone_common.h"
#include "gpio.h"
#include "PlatformNto1.h"

//GPIO初始化
void GPIO_init(unsigned short Dir, unsigned char IntEn, unsigned char TrigMode)
{
	/*set GPIO direction*/
	gpGPIO_regs->BANK_REGISTERS[0].DIR= (unsigned int)Dir;
	/*enable interrupt*/
	gpGPIO_regs->BINTEN= IntEn;//中断
	/*trigger interrupt on both rising and falling edge*/
	if(TrigMode == 0)
	{

		gpGPIO_regs->BANK_REGISTERS[0].SET_RIS_TRIG= 0x0000FFFF;
	}
	else
	{

		gpGPIO_regs->BANK_REGISTERS[0].SET_FAL_TRIG= 0x0000FFFF;
	}

	/*clear output data*/
	gpGPIO_regs->BANK_REGISTERS[0].CLR_DATA= 0xFFFFFFFF;//输出都清零

}


void Set_GPIO(unsigned char num)
{
	volatile Uint32 PID;
	gpGPIO_regs->BANK_REGISTERS[0].SET_DATA= (1<<num);
	PID= gpGPIO_regs->PID; //read to force write out
}

void Clr_GPIO(unsigned char num)
{
	volatile Uint32 PID;
	gpGPIO_regs->BANK_REGISTERS[0].CLR_DATA= (1<<num);
	PID= gpGPIO_regs->PID; //read to force write out
}




