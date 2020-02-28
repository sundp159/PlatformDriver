
#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdio.h>
#include <c6x.h>
#include <cslr_cgem.h>
#include <cslr_device.h>



/* GPIO pin state */
#define	GPIO_LOW				(0)
#define	GPIO_HIGH				(1)

/* Specific GPIO pins used in HRDSP6678D board */
#define GPIO_LED_PIN            GPIO_0
#define GPIO_FPGA_RESET_PIN     GPIO_1
#define GPIO_SRIO_STATE_PIN     GPIO_2


extern int g_iGpioPin;

/* GPIO direction */
typedef	enum _GPIO_DIRECTION
{
	GPIO_OUT = 0,
	GPIO_IN
}GPIO_DIRECTION;

inline void delay(unsigned int kk)
{
	volatile int i;

	for(i = 0; i < kk; i++);
		asm ("NOP");
	return;
}

/**********************************************************************\
* Function Declarations
\**********************************************************************/
/* GPIO APIs for genenal purpose */
void GpioEnableGlobalInterrupt(void);
void GpioDisableGlobalInterrupt(void);
void GpioSetRisingEdgeInterrupt(unsigned int uiPinNumber);
void GpioClearRisingEdgeInterrupt(unsigned int uiPinNumber);
void GpioSetFallingEdgeInterrupt(unsigned int uiPinNumber);
void GpioClearFallingEdgeInterrupt(unsigned int uiPinNumber);
void GpioSetDir(unsigned int uiPinNum, GPIO_DIRECTION GpioDir);
void GpioSetOutput(unsigned int uiPinNum);
void GpioClearOutput(unsigned int uiPinNum);
int GpioReadInput(unsigned int uiPinNum);



#endif
