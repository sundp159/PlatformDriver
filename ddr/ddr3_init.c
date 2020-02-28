#include <stdio.h>
#include "ddr3_init.h"
#include "PlatformNto1.h"
/****************系统时钟配置********************************
功能：PLL系统初始化,程序当中进行了减法和除法运算，因此配置参数只需要原始值即可
//Target Frequency (MHz) = input_clock (MHz) * PLL1_M / PLL1_D
// +--------------------+---------------+--------+--------+
// | DDR3 PLL VCO       | (CLKIN) Input |        |        |
// | Rate (MHz)         | Clock (MHz)   | PLL1_M | PLL1_D |
// +--------------------+---------------+--------+--------+
// | 1333               | 66.667 (EVM)  | 20     | 1      |
// | 1066               | 66.667        | 16     | 1      |
// | 800                | 66.667        | 12     | 1      |
// | 1333               | 50     (35)   | 80     | 3      |
// | 1066               | 50            | 64     | 3      |
// | 1000               | 50            | 20     | 1      |
// | 800                | 50            | 16     | 1      |

// +--------------------+---------------+--------+--------+

****************系统时钟配置********************************/

void KeyStone_DDR_PLL_init1 (unsigned int DDR_PLLM, unsigned int DDR_PLLD)
{
	int i;

	if(0<DDR_PLLM&&DDR_PLLM<=8192&&0<DDR_PLLD&&DDR_PLLD<=64)
	{
		KICK0 = KICK0_UNLOCK;
		KICK1 = KICK1_UNLOCK;
//		printf("Initialize DDR PLL = x%d/%d\n", DDR_PLLM, DDR_PLLD);
		MY_DDR3_PLL_CTL1 = 0x00000040      /*Set ENSAT bit = 1*/
			|((DDR_PLLM-1)/2)>>8;	/*BWADJ[11:8]*/
		MY_DDR3_PLL_CTL1 |= 0x00002000;      //Set RESET bit = 1
		MY_DDR3_PLL_CTL0 = (MY_DDR3_PLL_CTL0&0x00780000)|
			(((DDR_PLLM-1)/2)<<24)|((DDR_PLLM-1)<<6)|(DDR_PLLD-1);
		for(i=0;i<1000;i++);

		MY_DDR3_PLL_CTL1 &= ~(0x00002000);   //Clear RESET bit
		for(i=0;i<1000;i++);
		KICK0 = KICK0_LOCK;
		KICK1 = KICK1_LOCK;


	}
}


void ddr3_setup_auto_lvl_1333(void)
{
	int TEMP,i;
	KICK0 = 0x83E70B13;
	KICK1 = 0x95A4F1E0;
	/**************** 3.3 Leveling register configuration ********************/
	DDR3_CONFIG_REG_0 &= ~(0x007FE000);  // clear ctrl_slave_ratio field
	DDR3_CONFIG_REG_0 |= 0x00200000;     // set ctrl_slave_ratio to 0x100
	DDR3_CONFIG_REG_12 |= 0x08000000;    // Set invert_clkout = 1
	DDR3_CONFIG_REG_0 |= 0xF;            // set dll_lock_diff to 15

	//From 4.2.1 Executing Partial Automatic Leveling -- Start
	DDR3_CONFIG_REG_23 |= 0x00000200;    //Set bit 9 = 1 to use forced ratio leveling for read DQS
	//From 4.2.1 Executing Partial Automatic Leveling -- End

	//Values with invertclkout = 1
	/**************** 3.3 Partial Automatic Leveling ********************/
	DATA0_WRLVL_INIT_RATIO = 0x5E;
	DATA1_WRLVL_INIT_RATIO = 0x5E;
	DATA2_WRLVL_INIT_RATIO = 0x5E;
	DATA3_WRLVL_INIT_RATIO = 0x51;
	DATA4_WRLVL_INIT_RATIO = 0x38;
	DATA5_WRLVL_INIT_RATIO = 0x3A;
	DATA6_WRLVL_INIT_RATIO = 0x24;
	DATA7_WRLVL_INIT_RATIO = 0x20;
	DATA8_WRLVL_INIT_RATIO = 0x44;

	DATA0_GTLVL_INIT_RATIO = 0xDD;
	DATA1_GTLVL_INIT_RATIO = 0xDD;
	DATA2_GTLVL_INIT_RATIO = 0xBE;
	DATA3_GTLVL_INIT_RATIO = 0xCA;
	DATA4_GTLVL_INIT_RATIO = 0xA9;
	DATA5_GTLVL_INIT_RATIO = 0xA7;
	DATA6_GTLVL_INIT_RATIO = 0x9E;
	DATA7_GTLVL_INIT_RATIO = 0xA1;
	DATA8_GTLVL_INIT_RATIO = 0xBA;

	//Do a PHY reset. Toggle DDR_PHY_CTRL_1 bit 15 0->1->0
	DDR_DDRPHYC &= ~(0x00008000);
	DDR_DDRPHYC |= (0x00008000);
	DDR_DDRPHYC &= ~(0x00008000);

	/***************** 3.4 Basic Controller and DRAM Configuration ************/
	DDR_SDRFC    = 0x00005162;    // enable configuration

	/* DDR_SDTIM1   = 0x1113783C; */
	TEMP = 0;
	TEMP |= 0x8 << 25; // T_RP bit field 28:25
	TEMP |= 0x8 << 21; // T_RCD bit field 24:21
	TEMP |= 0x9 << 17; // T_WR bit field 20:17
	TEMP |= 0x17 << 12; // T_RAS bit field 16:12
	TEMP |= 0x20 << 6; // T_RC bit field 11:6
	TEMP |= 0x7 << 3; // T_RRD bit field 5:3
	TEMP |= 0x4; // T_WTR bit field 2:0
	DDR_SDTIM1 = TEMP;

	/* DDR_SDTIM2   = 0x30717FE3; */
	TEMP = 0;
	TEMP |= 0x3 << 28; // T_XP bit field 30:28
	TEMP |= 0x71 << 16; // T_XSNR bit field 24:16
	TEMP |= 0x1ff << 6; // T_XSRD bit field 15:6
	TEMP |= 0x4 << 3; // T_RTP bit field 5:3
	TEMP |= 0x3; // T_CKE bit field 2:0
	DDR_SDTIM2 = TEMP;

	/*  DDR_SDTIM3   = 0x559F86AF; */
	TEMP = 0;
	TEMP |= 0x5 << 28; // T_PDLL_UL bit field 31:28 (fixed value)
	TEMP |= 0x5 << 24; // T_CSTA bit field 27:24 (fixed value)
	TEMP |= 0x4 << 21; // T_CKESR bit field 23:21
	TEMP |= 0x3f << 15; // T_ZQCS bit field 20:15
	TEMP |= 0x6a << 4; // T_RFC bit field 12:4
	TEMP |= 0xf; // T_RAS_MAX bit field 3:0 (fixed value)
	DDR_SDTIM3 = TEMP;

	DDR_DDRPHYC  = 0x0010010F;

	DDR_ZQCFG    = 0x70073214;

	DDR_PMCTL    = 0x0;

	DDR_SDRFC = 0x00005162; // enable configuration

	/* DDR_SDCFG    = 0x63062A32; */
	/* New value with DYN_ODT disabled and SDRAM_DRIVE = RZQ/7 //0x63222A32;    // last config write DRAM init occurs */
	TEMP = 0;
	TEMP |= 0x3 << 29; // SDRAM_TYPE bit field 31:29 (fixed value)
	TEMP |= 0x0 << 27; // IBANK_POS bit field 28:27
	TEMP |= 0x3 << 24; // DDR_TERM bit field 26:24
	TEMP |= 0x0 << 21; // DYN_ODT bit field 22:21
	TEMP |= 0x1 << 18; // SDRAM_DRIVE bit field 19:18
	TEMP |= 0x2 << 16; // CWL bit field 17:16
	TEMP |= 0x1 << 14; // NM bit field 15:14   0X1-32bit,0x0-64bit
	TEMP |= 0xA << 10; // CL bit field 13:10
	TEMP |= 0x4 << 7; // ROWSIZE bit field 9:7
	TEMP |= 0x3 << 4; // IBANK bit field 6:4
	TEMP |= 0x0 << 3; // EBANK bit field 3:3
	TEMP |= 0x2; // PAGESIZE bit field 2:0
	DDR_SDCFG = TEMP;

	//Wait 600us for HW init to complete
	//    Delay_milli_seconds(1);
	for(i=0;i<1000;i++);
	DDR_SDRFC = 0x00001450;       //Refresh rate = (7.8*666MHz)

	/**************** 4.2.1 Executing Partial Automatic Leveling ********************/

	DDR_RDWR_LVL_RMP_CTRL = 0x80000000; //enable full leveling

	DDR_RDWR_LVL_CTRL = 0x80000000; //Trigger full leveling - This ignores read DQS leveling result and uses ratio forced value

	//    Delay_milli_seconds(1);
	for(i=0;i<1000;i++);
//    printf("\nDDR3 initialization is complete.\n");
	KICK0 = KICK0_LOCK;
	KICK1 = KICK1_LOCK;

}
/****************系统时钟配置********************************
功能：PLL系统初始化
// Please select PLL1_M values such that 0 < PLL1_M <= 64
// 公式：Target Frequency (MHz) = input_clock (MHz) * PLL1_M / (2*PLL1_D)
// +--------------------+---------------+--------+--------+
// | (CLK)Desired       | (CLKIN) Input |        |        |
// | Device Speed (MHz) | Clock (MHz)   | PLL1_M | PLL1_D |
// +--------------------+---------------+--------+--------+
// | 1000               | 100           | 10     | 1      |
// +--------------------+---------------+--------+--------+
****************系统时钟配置********************************/
void KeyStone_main_PLL_init1 ( unsigned int main_PLLM, unsigned int main_PLLD)
{
	unsigned int i;

	if(0<main_PLLM&&main_PLLM<=4096&&0<main_PLLD&&main_PLLD<=64)
	{
		KICK0 = KICK0_UNLOCK;
		KICK1 = KICK1_UNLOCK;

		MAINPLL_CTL1 |= 0x00000040; //Set ENSAT = 1

//		printf("Initialize main PLL = x%d/%d\n", main_PLLM, main_PLLD);

		//Bypass the PLL, PLL OUTPUT_DIVIDER = 2
		PLL_SECCTL |= 0x00880000;

		/*Clear the PLLENSRC bit in PLLCTL to 0 to allow PLLCTL.PLLEN to take effect*/
		PLL_PLLCTL= PLL_PLLCTL&(~(1<<5));

		/*Clear the PLLEN bit in PLLCTL to 0 (select PLL bypass mode)*/
		PLL_PLLCTL= PLL_PLLCTL&(~(1<<0));

		/*Wait for 4 input clock cycles to ensure PLLC switches to bypass mode properly*/
		for(i=0; i< 4*8192/5+1; i++)
			asm(" nop 5");

		/*The PLL has a bandgap generator circuit driving the PLL voltage regulator. In order
		to ensure proper PLL startup, the PLL power_down pin needs to be toggled. This is
		accomplished by toggling the “PLLPWRDN” bit in the PLLCTL register. This needs to
		be done before the main PLL initialization sequence. */
		PLL_PLLCTL |= (1<<1);	//Power down the PLL
		// wait for 5 us (min)
		for(i=0; i< 1000; i++)
			asm(" nop 5");

		// Verify if pll is in power down
		if ((PLL_PLLCTL & (1<<1)) !=0 )
		{
			PLL_PLLCTL &= ~(1<<1);         // Power up PLL
			// Wait PLL Stabilization time that is 150 usec
			for(i=0; i< 30000; i++)
				asm(" nop 5");
		}

		/*In PLLCTL, write PLLRST = 1 (PLL is reset)*/
		PLL_PLLCTL= PLL_PLLCTL|(1<<3);

		/*Program the required multiplier and divider values*/
		//pllc_regs->PREDIV= 0x8000;

		/*PLLM[5:0] bits of the multiplier is controlled by the PLLM
		register inside the PLL controller and PLLM[12:6] bits are
		controlled by the chip level register. MAINPLLCTL0 register
		PLLM[12:6] bits should be written just before writing to PLLM
		register PLLM[5:0] bits in the controller to have the complete
		13 bit value latched when the GO operation is initiated in the
		PLL controller.*/
		/*********该模式对应的是通用Keystone结构********/
		MAINPLL_CTL0 = ((main_PLLM-1)<<24)| 	/*BWADJ[7:0]*/
			(((main_PLLM*2-1)&0x1FC0)<<6)|(main_PLLD-1);
		/*********该模式结束********/
//		/*20130823对boot_cfg_regs->CORE_PLL_CTL0 修改，对应GEL*/
//		MAINPLL_CTL0 = ((main_PLLM-1)<<24)| 	/*BWADJ[7:0]*/
//			(((main_PLLM*2-1)&0x1FC0)<<6);
//		MAINPLL_CTL0  &= ~(0x0000003F);
//		MAINPLL_CTL0  |= (main_PLLD & 0x0000003F);
//		/*20130823对boot_cfg_regs->CORE_PLL_CTL0 修改结束*/
		PLL_PLLM= (main_PLLM*2-1)&0x3F;

		MAINPLL_CTL1 = 0x00000040      /*Set ENSAT bit = 1*/
			|(main_PLLM-1)>>8;	/*BWADJ[11:8]*/
		//pllc_regs->POSTDIV= 0x8000;

		/*Wait 1000 ns for PLL reset*/
		for(i=0; i< 1500/5+1; i++)
			asm(" nop 5");

		/*In PLLCTL, write PLLRST = 0 to bring PLL out of reset*/
		PLL_PLLCTL= PLL_PLLCTL&(~(1<<3));

		/*Wait 2000 input clock cycles for PLL to lock*/
		for(i=0; i< 2000*8192/5+1; i++)
			asm(" nop 5");

		//clear the bypass bit, enable PLL
		PLL_SECCTL &= ~(0x00800000);

		/*Set the PLLEN bit in PLLCTL to 1 to enable PLL mode*/
		PLL_PLLCTL= PLL_PLLCTL|(1<<0);

		/* Set GOSET bit in PLLCMD to initiate the GO operation to change the divide *
		* values and align the SYSCLKs as programmed                                */
		PLL_PLLCMD= 0x1;
		/*Wait 2000 input clock cycles for PLL to lock*/
		for(i=0; i< 2000*8192/5+1; i++)
			asm(" nop 5");

	}
	KICK0 = KICK0_LOCK;
	KICK1 = KICK1_LOCK;
}

/*
 * main.c
 */
//void main(void)
//{
////	ddr初始化
//	KeyStone_DDR_PLL_init (80, 3);
//	ddr3_setup_auto_lvl_1333();
//}


//配置PLL，1000MHz
#define MAINPLLM      64
#define MAINPLLD      10
//配置DDR-PLL 1000MHz参数
#define DDR3PLLM      32
#define DDR3PLLD      5
int PlatformNto1_Init(void)
{
	KeyStone_main_PLL_init1(MAINPLLM, MAINPLLD);
	KeyStone_DDR_PLL_init1(DDR3PLLM, DDR3PLLD);
	ddr3_setup_auto_lvl_1333();
	return 1;
}
