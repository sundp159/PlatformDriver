#include <stdio.h>
#include <csl_bootcfgAux.h>
#include <csl_cpintc.h>
#include "KeyStone_common.h"
#include "cslr_uart.h"
#include <stdbool.h>
#include "cslr_device.h"


CSL_CPINTCRegs* CIC_Regs;

//串口0中断初始化,CIC部分，内核部分需要在bios中进行配置（中断号、中断函数）
void UartInterrupt_Init(void)
{
	CSL_CPINTCRegs* gpCIC0_regs = (CSL_CPINTCRegs*)CSL_CP_INTC_0_REGS;
	if(DNUM<4)
		CIC_Regs= gpCIC0_regs;
//	else
//		CIC_Regs= gpCIC1_regs;

	/* Disable Global host interrupts. */
	CIC_Regs->GLOBAL_ENABLE_HINT_REG= 0;
	/* Configure no nesting support in the CPINTC Module. */
	CIC_Regs->CONTROL_REG= ((CIC_Regs->CONTROL_REG&
			~CSL_CPINTC_CONTROL_REG_NEST_MODE_MASK)|
			(0x0<<CSL_CPINTC_CONTROL_REG_NEST_MODE_SHIFT));
	/*map UART RX EVTGINT to CIC out*/
	KeyStone_CIC_event_map(CIC_Regs, CSL_INTC0_URXEVT, 0);
	/* Enable Global host interrupts. */
	CIC_Regs->GLOBAL_ENABLE_HINT_REG= 1;

}





