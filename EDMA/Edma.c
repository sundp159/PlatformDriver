#include <csl_edma3.h>
#include <cslr_device.h>
#include "KeyStone_common.h"
#include "stdint.h"
#include <csl_edma3Aux.h>
#include "cslr_tpcc.h"


int  EdmaEnable (Uint32 src, Uint32 dst, Uint32 uiACount,
	Uint32 uiBCount, Uint32 srcBIndex, Uint32 desBIndex, CSL_TpccRegs*  EDMACCRegs, Uint32 TC_channel)
{
	//clear completion flag
	EDMACCRegs->TPCC_ICR=1<<(TC_channel);
	EDMACCRegs->PARAMSET[TC_channel].OPT=
		CSL_EDMA3_OPT_MAKE(CSL_EDMA3_ITCCH_DIS,
			CSL_EDMA3_TCCH_DIS,
			CSL_EDMA3_ITCINT_DIS,
			CSL_EDMA3_TCINT_EN,
			TC_channel,//应该用channel
			CSL_EDMA3_TCC_NORMAL,
			CSL_EDMA3_FIFOWIDTH_NONE,
			CSL_EDMA3_STATIC_DIS,
			CSL_EDMA3_SYNC_AB,
			CSL_EDMA3_ADDRMODE_INCR,
			CSL_EDMA3_ADDRMODE_INCR);
	EDMACCRegs->PARAMSET[TC_channel].SRC= src;
	EDMACCRegs->PARAMSET[TC_channel].A_B_CNT= (uiBCount<<16)|uiACount;
	EDMACCRegs->PARAMSET[TC_channel].DST= dst;
	EDMACCRegs->PARAMSET[TC_channel].SRC_DST_BIDX= ((desBIndex<<16)|(srcBIndex));
	EDMACCRegs->PARAMSET[TC_channel].LINK_BCNTRLD= (uiBCount<<16)|0xFFFF;
	EDMACCRegs->PARAMSET[TC_channel].SRC_DST_CIDX= 0;
	EDMACCRegs->PARAMSET[TC_channel].CCNT= 1;

	/*Manually trigger the EDMA*/
	EDMACCRegs->TPCC_ESR= 1<<(TC_channel);//

	/* Wait for completion */
	while ((EDMACCRegs->TPCC_IPR&(1<<(TC_channel))) ==0);
	//printf("transfer %4d * %5d Bytes with index=%5d from 0x%8x to 0x%8x,\n ", uiBCount, uiACount, uiIndex, src, dst);
	//clear completion flag
	EDMACCRegs->TPCC_ICR=1<<(TC_channel);
	return 0;

}
