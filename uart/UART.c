
#include "stdio.h"
#include "tistdtypes.h"
#include "cslr_device.h"
#include "cslr_uart.h"
#include "string.h"
#include "PlatformNto1.h"


CSL_UartRegs *gpUartRegs[CSL_UART_PER_CNT]=
{
#ifdef CSL_UART_A_REGS
	(CSL_UartRegs *)CSL_UART_A_REGS
#else
	(CSL_UartRegs *)CSL_UART_REGS
#endif
#ifdef CSL_UART_B_REGS
    ,
	(CSL_UartRegs *)CSL_UART_B_REGS
#endif
};

UART_Tx_Master txMaster[CSL_UART_PER_CNT];


void KeyStone_UART_init(UART_Config *pUARTCfg, Uint32 uartNum)
{
	Uint32 divider;
	Uint32 uartDll, uartDlh;
	Uint32 osmFactor;
//	Uint32 effectRate;

	CSL_UartRegs *localUartRegs = gpUartRegs[uartNum];

    if(uartNum >= CSL_UART_PER_CNT)
    {
        printf("Invalid UART number!\n");
        return;
    }

	//make sure TX and RX is reset in case it was enabled from last run.
    CSL_FINS(localUartRegs->PWREMU_MGMT, UART_PWREMU_MGMT_UTRST, CSL_UART_PWREMU_MGMT_UTRST_RESET);
    CSL_FINS(localUartRegs->PWREMU_MGMT, UART_PWREMU_MGMT_URRST, CSL_UART_PWREMU_MGMT_URRST_RESET);

	if(pUARTCfg->osmSel == OVER_SAMPLING_13X)
	{
		osmFactor = 13;
	}
	else
	{
		osmFactor = 16;
	}

    CSL_FINS(localUartRegs->MDR, UART_MDR_OSM_SEL, pUARTCfg->osmSel);

	divider = (Uint32)((float)(pUARTCfg->DSP_Core_Speed_Hz)/
		(float)(pUARTCfg->baudRate * 6 * osmFactor)+0.5f);

//	effectRate= pUARTCfg->DSP_Core_Speed_Hz/(6*divider*osmFactor);//zhengxin20170424
//	if(pUARTCfg->baudRate!= effectRate)
//		printf("Required baud rate %d, effective baud rate %d!\n", pUARTCfg->baudRate, effectRate);

	uartDll = divider & 0xFF;
	uartDlh = (divider & 0xFF00)>>8;

    CSL_FINS(localUartRegs->DLH, UART_DLL_DLL, uartDlh);
    CSL_FINS(localUartRegs->DLL, UART_DLH_DLH, uartDll);

	//FIFO mode is used
    CSL_FINS(localUartRegs->FCR, UART_FCR_FIFOEN, CSL_UART_FCR_FIFOEN_ENABLE);
    CSL_FINS(localUartRegs->FCR, UART_FCR_RXFIFTL, pUARTCfg->fifoRxTriBytes);
    CSL_FINS(localUartRegs->FCR, UART_FCR_DMAMODE1, CSL_UART_FCR_DMAMODE1_ENABLE);

    if (pUARTCfg->parityMode == PARITY_DISABLE)
	{
        CSL_FINS(localUartRegs->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_DISABLE);
        CSL_FINS(localUartRegs->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_ODD);
        CSL_FINS(localUartRegs->LCR, UART_LCR_SP,  CSL_UART_LCR_SP_DISABLE);
	}
	else if (pUARTCfg->parityMode == ODD_PARITY_ENABLE_SET1)
	{
        CSL_FINS(localUartRegs->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_ENABLE);
        CSL_FINS(localUartRegs->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_ODD);
        CSL_FINS(localUartRegs->LCR, UART_LCR_SP,  CSL_UART_LCR_SP_DISABLE);
	}
    else if (pUARTCfg->parityMode == EVEN_PARITY_ENABLE_SET1)
	{
        CSL_FINS(localUartRegs->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_ENABLE);
        CSL_FINS(localUartRegs->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_EVEN);
        CSL_FINS(localUartRegs->LCR, UART_LCR_SP,  CSL_UART_LCR_SP_DISABLE);
	}
    else if (pUARTCfg->parityMode == STICK_PARITY_ENABLE_SET)
	{
        CSL_FINS(localUartRegs->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_ENABLE);
        CSL_FINS(localUartRegs->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_ODD);
        CSL_FINS(localUartRegs->LCR, UART_LCR_SP,  CSL_UART_LCR_SP_ENABLE);
	}
    else if (pUARTCfg->parityMode == STICK_PARITY_ENABLE_CLR)
	{
        CSL_FINS(localUartRegs->LCR, UART_LCR_PEN, CSL_UART_LCR_PEN_ENABLE);
        CSL_FINS(localUartRegs->LCR, UART_LCR_EPS, CSL_UART_LCR_EPS_EVEN);
        CSL_FINS(localUartRegs->LCR, UART_LCR_SP,  CSL_UART_LCR_SP_ENABLE);
	}
    else
    {
        printf("Invalid UART Parity configuration!\n");
        return;
    }

    CSL_FINS(localUartRegs->LCR, UART_LCR_STB, pUARTCfg->stopMode);
    CSL_FINS(localUartRegs->LCR, UART_LCR_WLS, pUARTCfg->dataLen);

    if (pUARTCfg->autoFlow== AUTO_FLOW_DIS)
    {
        CSL_FINS(localUartRegs->MCR, UART_MCR_AFE, CSL_UART_MCR_AFE_DISABLE);
        CSL_FINS(localUartRegs->MCR, UART_MCR_RTS, CSL_UART_MCR_RTS_DISABLE);
    }
    else if (pUARTCfg->autoFlow == AUTO_FLOW_CTS_EN)
    {
        CSL_FINS(localUartRegs->MCR, UART_MCR_AFE, CSL_UART_MCR_AFE_ENABLE);
        CSL_FINS(localUartRegs->MCR, UART_MCR_RTS, CSL_UART_MCR_RTS_DISABLE);
    }
    else if (pUARTCfg->autoFlow == AUTO_FLOW_RTS_CTS_EN)
    {
        CSL_FINS(localUartRegs->MCR, UART_MCR_AFE, CSL_UART_MCR_AFE_ENABLE);
        CSL_FINS(localUartRegs->MCR, UART_MCR_RTS, CSL_UART_MCR_RTS_ENABLE);
    }
    else
    {
        printf("Invalid UART auto flow control configuration!\n");
        return;
    }

    /* Setup the UART to loopback mode */
    CSL_FINS(localUartRegs->MCR, UART_MCR_LOOP, pUARTCfg->bLoopBackEnable);

	//enable UART
    CSL_FINS(localUartRegs->PWREMU_MGMT, UART_PWREMU_MGMT_UTRST, CSL_UART_PWREMU_MGMT_UTRST_ENABLE);
    CSL_FINS(localUartRegs->PWREMU_MGMT, UART_PWREMU_MGMT_URRST, CSL_UART_PWREMU_MGMT_URRST_ENABLE);
    CSL_FINS(localUartRegs->PWREMU_MGMT, UART_PWREMU_MGMT_FREE,  CSL_UART_PWREMU_MGMT_FREE_STOP);

	//enable receive interrupts
    CSL_FINS(localUartRegs->IER, UART_IER_ELSI, CSL_UART_IER_ELSI_ENABLE);
    CSL_FINS(localUartRegs->IER, UART_IER_ERBI, CSL_UART_IER_ERBI_ENABLE);

    //localUartRegs->IER = 0xFFFFFFFF;

	//record the master used for transmit for this UART
    txMaster[uartNum]= pUARTCfg->txMaster;

	return;
}

void UartWrite(unsigned char*buff,unsigned int len)
{
	 int i;
	 CSL_UartRegs *localUartRegs = gpUartRegs[0];
//	while(1)
//	{
	 for(i=0;i<len;i++)
	 {
	      //wait for UART FIFO empty
	      while(!((localUartRegs->LSR) & CSL_UART_LSR_THRE_MASK));
           localUartRegs->THR = buff[i];
	 }
//	}
}



void Uart_Init(UART_Config *UARTCfg)
{
	KeyStone_UART_init(UARTCfg, 0);//串口0
}

unsigned int UartRead(unsigned char *buffer, unsigned int buffByteLen)
{
    Uint32 byteCnt=0;
    Uint32 NumCnt=0;
    CSL_UartRegs *localUART_Regs;
	localUART_Regs = gpUartRegs[0];

	for(byteCnt=0; byteCnt<buffByteLen; )
	{
     	if(localUART_Regs->LSR & CSL_UART_LSR_DR_MASK)//FIFO不空就读
		{
			buffer[byteCnt] = localUART_Regs->RBR;
			byteCnt++;
		}
     	NumCnt++;
     	//if(NumCnt>0x10000)
     	if(NumCnt>0x1000000)//增加等待FIFO空的时间，因为读取的数据量大，读取FIFO的速度快于数据进入FIFO的速度，
     	{					//等待时间短的话会超时进入if，return 0，不能完全读取完151字节数据。
     						//等待时间与bps有很大关系，bps越低，等待时间就要越长，该等待时间可以低到1200bps
     		return 0;
     	}
	}
	return byteCnt;
}

