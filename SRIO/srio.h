#ifndef SRIO_H_
#define SRIO_H_

#include "KeyStone_SRIO_Init_drv.h"
#include "Keystone_Serdes_init.h"
#include <string.h>


#define SRIO_CLOCK_TEST_SPEED	 156.25    //serdes时钟Mhz
#define SRIO_DEFAULT_TEST_SPEED	 2.5      //RIO速率G




#define DOORBELL_ICSR_ICCR0_RIO_DOORBELL_ICSR  *(unsigned int*)(CSL_SRIO_CONFIG_REGS + 0x0180)
#define DOORBELL_ICSR_ICCR0_RIO_DOORBELL_ICCR1 *(unsigned int*)(CSL_SRIO_CONFIG_REGS + 0x0188)

typedef struct
{
	SRIO_Packet_Type packet_type;
	Uint32 source;
	Uint32 dest;
	Uint32 byteCount;
}SRIO_Transfer_Param;



#endif
