
/**************************************************************************\
* Header File Including
\**************************************************************************/
#include "gpio.h"
#include "PlatformNto1.h"

/**************************************************************************\
* Macro Definitions
\**************************************************************************/

/* GPIO registers' address */
#define GPIO_BASE_ADDR          0x02320000 // for C6678
#define GPIO_BINTEN_REG		    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x08)
#define GPIO_DIR_REG		    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x10)
#define GPIO_OUT_DATA_REG	    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x14)
#define GPIO_SET_DATA_REG	    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x18)
#define GPIO_CLEAR_DATA_REG	    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x1c)
#define GPIO_IN_DATA_REG	    *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x20)
#define GPIO_SET_RIS_TRIG_REG   *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x24)
#define GPIO_CLR_RIS_TRIG_REG   *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x28) 
#define GPIO_SET_FAL_TRIG_REG   *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x2C)
#define GPIO_CLR_FAL_TRIG_REG   *(volatile unsigned int *)(GPIO_BASE_ADDR + 0x30)




/**************************************************************************\
* Function Definitions
\**************************************************************************/
/**************************************************************************
  Function:  void GpioEnableGlobalInterrupt(void)

  Description:  This function enables GPIO interrupts to CPU.

  Parameters:  None

  Return:  None.
**************************************************************************/
void GpioEnableGlobalInterrupt(void)
{
    GPIO_BINTEN_REG = 0x1;
}

/**************************************************************************
  Function:  void GpioDisableGlobalInterrupt(void)

  Description:  This function disables GPIO interrupts to CPU.

  Parameters:  None

  Return:  None.
**************************************************************************/
void GpioDisableGlobalInterrupt(void)
{
    GPIO_BINTEN_REG = 0x0;
}

/**************************************************************************
  Function:  void GpioSetRisingEdgeInterrupt(void)

  Description:  This function sets specified GPIO's rising edge interrupt

  Parameters:  uiPinNumber - GPIO number to configure

  Return:  None.
**************************************************************************/
void GpioSetRisingEdgeInterrupt(unsigned int uiPinNumber)
{
    GPIO_SET_RIS_TRIG_REG |= (1 << uiPinNumber);
}

/**************************************************************************
  Function:  void GpioClearRisingEdgeInterrupt(void)

  Description:  This function clears specified GPIO's rising edge interrupt

  Parameters:  uiPinNumber - GPIO number to configure
  
  Return:  None.
**************************************************************************/
void GpioClearRisingEdgeInterrupt(unsigned int uiPinNumber)
{
    GPIO_CLR_RIS_TRIG_REG |= (1 << uiPinNumber);
}

/**************************************************************************
  Function:  void GpioSetFallingEdgeInterrupt(void)

  Description:  This function sets specified GPIO's falling edge interrupt

  Parameters:  uiPinNumber - GPIO number to configure

  Return:  None.
**************************************************************************/
void GpioSetFallingEdgeInterrupt(unsigned int uiPinNumber)
{
    GPIO_SET_FAL_TRIG_REG |= (1 << uiPinNumber);
}

/**************************************************************************
  Function:  void GpioClearFallingEdgeInterrupt(void)

  Description:  This function clears specified GPIO's falling edge interrupt

  Parameters:  uiPinNumber - GPIO number to configure

  Return:  None.
**************************************************************************/
void GpioClearFallingEdgeInterrupt(unsigned int uiPinNumber)
{
    GPIO_CLR_FAL_TRIG_REG |= (1 << uiPinNumber);
}

/**************************************************************************
  Function:  void GpioSetDir(void)
  
  Description:  This function configures the specified GPIO's direction
  
  Parameters:  uiPinNum - GPIO number to configure 
               Gpio_Dir - GPIO_OUT or GPIO_IN
  
  Return:  None.
**************************************************************************/
void GpioSetDir(unsigned int uiPinNum, GPIO_DIRECTION Gpio_Dir)
{   
    if(Gpio_Dir == GPIO_OUT) 
    {
    	/* Set GPIO_DIR_REG's pin bit to 0 as output */
        GPIO_DIR_REG = GPIO_DIR_REG & ~(1 << uiPinNum);  
    }
    else 
    {
    	/* Set GPIO_DIR_REG's pin bit to 1 as input */
        GPIO_DIR_REG = GPIO_DIR_REG | (1 << uiPinNum); 
    }
}


/**************************************************************************
  Function: void GpioSetOutput(unsigned int uiPinNum)
  
  Description:  This function sets the specified GPIO's pin state to 1.
                The specified GPIO should be configured as output first.
  
  Parameters:  uiPinNum - GPIO number to configure 
  
  Return:  None.
**************************************************************************/
void GpioSetOutput(unsigned int uiPinNum)
{
    /* Perform one of the following is OK */
#if 1
    /* Set GPIO_SET_DATA_REG bit to 1 to set GPIO pin to a logic-high state */
    GPIO_SET_DATA_REG |= (1 << uiPinNum);
#else /* Can't work by this. Marked by wulei, 20130529 */
    /* Set GPIO_OUT_DATA_REG bit to 1 to set GPIO pin to a logic-high state */
    GPIO_OUT_DATA_REG |= (1 << uiPinNum);
#endif
}

/**************************************************************************
  Function:  void GpioClearOutput(unsigned int uiPinNum)
  
  Description:  This function clear the specified GPIO's pin state to 0.
                The specified GPIO should be configured as output first.
  
  Parameters:  uiPinNum - GPIO number to configure 

  Return:  None.
**************************************************************************/
void GpioClearOutput(unsigned int uiPinNum)
{
    /* Perform one of the following is OK */
#if 1
    /* Set GPIO_SET_DATA_REG bit to 0 to set GPIO pin to a logic-low state */
    GPIO_CLEAR_DATA_REG |= (1 << uiPinNum);
#else /* Can't work by this. Marked by wulei, 20130529 */
    /* Set GPIO_OUT_DATA_REG bit to 0 to set GPIO pin to a logic-low state */
    GPIO_OUT_DATA_REG &= ~(1 << uiPinNum);
#endif
}

/**************************************************************************
  Function:  void GpioReadInput(unsigned int uiPinNum)
  
  Description:  This function read the specified GPIO's pin.
                The specified GPIO should be configured as input first.
  
  Parameters:  uiPinNum - GPIO number to read 
  
  Return:  0 - GPIO_LOW
           1 - GPIO_HIGH
           -1 - error for uiPinNum
**************************************************************************/
int GpioReadInput(unsigned int uiPinNum)
{    
	unsigned int uiRead;
	
	/* wrong pin number */
    if(uiPinNum > GPIO_MAX_NUM)
    	return -1;
    
    /* read from GPIO_IN_DATA_REG's pin bit */
    uiRead = (GPIO_IN_DATA_REG >> uiPinNum) & 0x1;
    
    /* 0 as GPIO_LOW, and 1 as GPIO_HIGH */
    return uiRead;
}

/**************************************************************************
  Function:  void GpioInit(void)

  Description:  Initializes the GPIO peripheral. It must be called before 
                using any other GPIO functions.

  Parameters:  None.

  Return:  None.
**************************************************************************/
void GpioInit(void)
{
	unsigned int uiPinNum;

	/* Disable the GPIO global interrupts. Do not use GPIO pins as CPU
	   interrupts and EDMA events */
	GpioDisableGlobalInterrupt();

	/* Clear all falling edge trigger and rising edge trigger */
	for(uiPinNum = GPIO_0; uiPinNum <= GPIO_MAX_NUM; uiPinNum++)
	{
        GpioClearRisingEdgeInterrupt(uiPinNum);
        GpioClearFallingEdgeInterrupt(uiPinNum);
    }
}

