#include "srio.h"
#include "PlatformNto1.h"

SRIO_Loopback_Mode loopback_mode= SRIO_NO_LOOPBACK;//回环模式

SRIO_Config srio_cfg;
SRIO_Interrupt_Cfg interrupt_cfg;

SerdesSetup_4links serdes_cfg;
SerdesLinkSetup serdesLinkSetup;

SRIO_Interrupt_Map interrupt_map[]=
{
    /*interrupt_event*/ /*INTDST_number*/
	{DOORBELL0_0_INT, 	INTDST_16}, 	/*route to core 0*/
	{DOORBELL0_1_INT, 	INTDST_16}, 	/*route to core 0*/
	{DOORBELL0_2_INT, 	INTDST_16}, 	/*route to core 0*/
	{DOORBELL0_3_INT, 	INTDST_16}, 	/*route to core 0*/
	{DOORBELL0_4_INT, 	INTDST_16} 		/*route to core 0*/
};

SRIO_Device_ID_Routing_Config dsp0_device_ID_routing_config[]=
{
		/*idPattern 	idMatchMask 	routeMaintenance*/
		{DSP0_SRIO_BASE_ID+0, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+1, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+2, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+3, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+4, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+5, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+6, 	0xFFFF, 	1},
		{DSP0_SRIO_BASE_ID+7, 	0xFFFF, 	1},
};


SRIO_Device_ID_Routing_Config dsp1_device_ID_routing_config[]=
{
		/*idPattern 	idMatchMask 	routeMaintenance*/
		{DSP1_SRIO_BASE_ID+0, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+1, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+2, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+3, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+4, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+5, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+6, 	0xFFFF, 	1},
		{DSP1_SRIO_BASE_ID+7, 	0xFFFF, 	1},
};


SRIO_Device_ID_Routing_Config dsp2_device_ID_routing_config[]=
{
		/*idPattern 	idMatchMask 	routeMaintenance*/
		{DSP2_SRIO_BASE_ID+0, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+1, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+2, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+3, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+4, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+5, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+6, 	0xFFFF, 	1},
		{DSP2_SRIO_BASE_ID+7, 	0xFFFF, 	1},
};

SRIO_Device_ID_Routing_Config dsp3_device_ID_routing_config[]=
{
		/*idPattern 	idMatchMask 	routeMaintenance*/
		{DSP3_SRIO_BASE_ID+0, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+1, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+2, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+3, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+4, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+5, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+6, 	0xFFFF, 	1},
		{DSP3_SRIO_BASE_ID+7, 	0xFFFF, 	1},
};


void srio_identify_used_ports_lanes(SRIO_1x2x4x_Path_Control srio_1x2x4x_path_control);
void SRIOInterrupts_Init(void);
int SRIO_Init(unsigned char DspSrioId);
void InitLsuTransfer(SRIO_LSU_Transfer * lsuTransfer,
	SRIO_Transfer_Param * transferParam, Uint32 uiLsuNum,
	Uint32 uiDstID, Uint32 uiSrcIDMap);

void SRIOInterrupts_Init(void)
{
	interrupt_cfg.interrupt_map = interrupt_map;
	interrupt_cfg.uiNumInterruptMap = sizeof(interrupt_map) / sizeof(SRIO_Interrupt_Map);
	interrupt_cfg.interrupt_rate = NULL;
    interrupt_cfg.uiNumInterruptRateCfg = 0;
    interrupt_cfg.doorbell_route_ctl = SRIO_DOORBELL_ROUTE_TO_DEDICATE_INT;
    srio_cfg.interrupt_cfg = &interrupt_cfg; 
}


int SRIO_Init(unsigned char DspSrioId)
{
	serdes_cfg.commonSetup.inputRefClock_MHz = SRIO_CLOCK_TEST_SPEED;

	memset(&srio_cfg, 0, sizeof(srio_cfg));
	srio_cfg.blockEn.bBLK1_LSU_EN= 1;
	srio_cfg.blockEn.bBLK2_MAU_EN= 1;
	srio_cfg.loopback_mode= loopback_mode;//回环模式

	switch(DspSrioId)
	{
	 case DSP0_SRIO_BASE_ID: srio_cfg.device_ID_routing_config= dsp0_device_ID_routing_config;break;
	 case DSP1_SRIO_BASE_ID: srio_cfg.device_ID_routing_config= dsp1_device_ID_routing_config;break;
	 case DSP2_SRIO_BASE_ID: srio_cfg.device_ID_routing_config= dsp2_device_ID_routing_config;break;
	 case DSP3_SRIO_BASE_ID: srio_cfg.device_ID_routing_config= dsp3_device_ID_routing_config;break;
	}

	srio_cfg.uiNumDeviceId = sizeof(dsp0_device_ID_routing_config)/sizeof(SRIO_Device_ID_Routing_Config);/*结构体数组*/

	serdes_cfg.commonSetup.loopBandwidth= SERDES_PLL_LOOP_BAND_MID; /** set pll loop bandwidth mid */

	srio_cfg.serdes_cfg= &serdes_cfg;
	serdesLinkSetup.txOutputSwing    = 10;
	serdesLinkSetup.testPattern      = SERDES_TEST_DISABLED; /** Test mode disabled */
	serdesLinkSetup.rxAlign          = SERDES_RX_COMMA_ALIGNMENT_ENABLE;
	serdesLinkSetup.rxInvertPolarity = SERDES_RX_NORMAL_POLARITY;/** Selects the Receive pair normal polarity*/
	serdesLinkSetup.rxTermination    = SERDES_RX_TERM_COMMON_POINT_AC_COUPLE;
	serdesLinkSetup.rxEqualizerConfig= SERDES_RX_EQ_ADAPTIVE; /** Selects the adaptive Receiver equalizer*/
	serdesLinkSetup.rxCDR            = SERDES_RX_CDR_1; /* First order. Phase offset tracking up to ±488 ppm */
	serdesLinkSetup.txInvertPolarity = SERDES_TX_NORMAL_POLARITY;/** Selects Tx pair normal polarity */
	serdesLinkSetup.linkSpeed_GHz    = SRIO_DEFAULT_TEST_SPEED;/*配置的速率*/
	srio_cfg.srio_1x2x4x_path_control= SRIO_PATH_CTL_4xLaneABCD;/*配置工作模式*/
	srio_identify_used_ports_lanes(srio_cfg.srio_1x2x4x_path_control);
	KeyStone_SRIO_Init(&srio_cfg);
	return 1;
}

void srio_identify_used_ports_lanes(SRIO_1x2x4x_Path_Control srio_1x2x4x_path_control)
{
	int i;
	Uint32 uiPathConfig, uiPathMode;
	Uint32 uiLaneEnableMask=0, uiLogicPortEnableMask=0;
	uiPathConfig= (srio_1x2x4x_path_control&
			CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION_MASK)>>
			CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_CONFIGURATION_SHIFT;

	uiPathMode= (srio_1x2x4x_path_control&
			CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE_MASK)>>
			CSL_SRIO_RIO_PLM_SP_PATH_CTL_PATH_MODE_SHIFT;

	if(1==uiPathConfig)
	{
		uiLaneEnableMask= 0x1; 	/*0001*/

		uiLogicPortEnableMask= 0x1; 	/*0001*/
	}
	else if(2==uiPathConfig)
	{
		uiLaneEnableMask= 0x3; 	/*0011*/

		if(0==uiPathMode)
			uiLogicPortEnableMask= 0x3; 	/*0011*/
		else if(1==uiPathMode)
			uiLogicPortEnableMask= 0x1; 	/*0001*/
	}
	else if(4==uiPathConfig)
	{
		uiLaneEnableMask= 0xF; 	/*1111*/

		if(0==uiPathMode)
			uiLogicPortEnableMask= 0xF; 	/*1111*/
		else if(1==uiPathMode)
			uiLogicPortEnableMask= 0xD; 	/*1101*/
		else if(2==uiPathMode)
			uiLogicPortEnableMask= 0x7; 	/*0111*/
		else if(3==uiPathMode)
			uiLogicPortEnableMask= 0x5; 	/*0101*/
		else if(4==uiPathMode)
			uiLogicPortEnableMask= 0x1; 	/*0001*/
	}
	/*enable ports data path according to 1x 2x 4x path configuration*/
	srio_cfg.blockEn.bBLK5_8_Port_Datapath_EN[0]= uiLaneEnableMask&1;
	srio_cfg.blockEn.bBLK5_8_Port_Datapath_EN[1]= (uiLaneEnableMask>>1)&1;
	srio_cfg.blockEn.bBLK5_8_Port_Datapath_EN[2]= (uiLaneEnableMask>>2)&1;
	srio_cfg.blockEn.bBLK5_8_Port_Datapath_EN[3]= (uiLaneEnableMask>>3)&1;
	/*disable Serdes according to 1x 2x 4x path configuration*/
	for(i= 0; i<4; i++ )
	{
		if(uiLaneEnableMask&(1<<i))
			srio_cfg.serdes_cfg->linkSetup[i]= &serdesLinkSetup;
		else
			srio_cfg.serdes_cfg->linkSetup[i]= NULL;
	}
	/*enable loggical ports according to 1x 2x 4x path configuration*/
	srio_cfg.blockEn.bLogic_Port_EN[0]= uiLogicPortEnableMask&1;
	srio_cfg.blockEn.bLogic_Port_EN[1]= (uiLogicPortEnableMask>>1)&1;
	srio_cfg.blockEn.bLogic_Port_EN[2]= (uiLogicPortEnableMask>>2)&1;
	srio_cfg.blockEn.bLogic_Port_EN[3]= (uiLogicPortEnableMask>>3)&1;
}

int SRIO_Send(unsigned int source_address,unsigned int dest_address,unsigned int send_size, unsigned char desID)
{
	SRIO_Transfer_Param  transferParam;
	SRIO_LSU_Transfer lsuTransfer;
	transferParam.byteCount  = send_size;
	transferParam.packet_type= SRIO_PKT_TYPE_NWRITE;/*工作模式*/
	transferParam.source     = source_address;
	transferParam.dest       = dest_address;
	InitLsuTransfer(&lsuTransfer, &transferParam, 0, desID, 0);//lsu号，目标ID，发送端口号
	KeyStone_SRIO_LSU_transfer(&lsuTransfer);
	KeyStone_SRIO_wait_LSU_completion(0,lsuTransfer.transactionID, lsuTransfer.contextBit);/*等待LSU的传输是否完成*/
	return 1;
}


void InitLsuTransfer(SRIO_LSU_Transfer * lsuTransfer,
	SRIO_Transfer_Param * transferParam, Uint32 uiLsuNum,
	Uint32 uiDstID, Uint32 uiSrcIDMap)
{
    lsuTransfer->rioAddressMSB=0;
	/*swap source/dest for READ*/
	if(SRIO_PKT_TYPE_NREAD==transferParam->packet_type)
	{
	    lsuTransfer->rioAddressLSB_ConfigOffset= transferParam->source;
	    lsuTransfer->localDspAddress= transferParam->dest;
	}
	else
	{
	    lsuTransfer->rioAddressLSB_ConfigOffset= transferParam->dest;
	    lsuTransfer->localDspAddress= transferParam->source;
	}
    lsuTransfer->bytecount= transferParam->byteCount;
    lsuTransfer->packetType= transferParam->packet_type;
    lsuTransfer->dstID= uiDstID;
    lsuTransfer->doorbellInfo= 0;
    lsuTransfer->waitLsuReady= TRUE;
    lsuTransfer->lsuNum= uiLsuNum;
    lsuTransfer->doorbellValid = 0;
    lsuTransfer->intrRequest = 0;
    lsuTransfer->supGoodInt = 0;
    lsuTransfer->priority = 0;
    lsuTransfer->outPortID = uiLsuNum;
    lsuTransfer->idSize = 0;
    lsuTransfer->srcIDMap = uiSrcIDMap;
    lsuTransfer->hopCount = 0;
}
void SRIO_Send_DoorBell(unsigned int uiDestID, unsigned int uiDoorBellInfo)
{
	KeyStone_SRIO_DoorBell(0, 0, uiDestID, uiDoorBellInfo);
}

unsigned int SRIO_Read_DoorBell(void)
{
	Uint32 doorbell;
	doorbell= DOORBELL_ICSR_ICCR0_RIO_DOORBELL_ICSR;
	DOORBELL_ICSR_ICCR0_RIO_DOORBELL_ICCR1 = doorbell;

	return doorbell;
}


