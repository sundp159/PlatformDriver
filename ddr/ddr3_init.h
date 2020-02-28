

#ifndef DDR3_INIT_H_
#define DDR3_INIT_H_
#ifndef CHIP_LEVEL_REG


// BOOT and CONFIG dsp system modules Definitions
#define CHIP_LEVEL_REG  	  0x02620000
#define KICK0           	  (*(unsigned int*)(CHIP_LEVEL_REG + 0x0038))
#define KICK1           	  (*(unsigned int*)(CHIP_LEVEL_REG + 0x003C))
#define MAINPLL_CTL0          (*(unsigned int*)(CHIP_LEVEL_REG + 0x0328))
#define MAINPLL_CTL1          (*(unsigned int*)(CHIP_LEVEL_REG + 0x032C))
#define MY_DDR3_PLL_CTL0         (*(unsigned int*)(CHIP_LEVEL_REG + 0x0330))
#define MY_DDR3_PLL_CTL1         (*(unsigned int*)(CHIP_LEVEL_REG + 0x0334))//原名称DDR3_PLL_CTL1、DDR3_PLL_CTL0与csl库中cslr_bootcfg.h、csl_bootcfgaux.h结构体成员重名，编译出错
#define DDR3_CONFIG_REG_0     (*(unsigned int*)(CHIP_LEVEL_REG + 0x0404))    //所以修改了名称
#define DDR3_CONFIG_REG_12    (*(unsigned int*)(CHIP_LEVEL_REG + 0x0434))
#define DDR3_CONFIG_REG_23    (*(unsigned int*)(CHIP_LEVEL_REG + 0x0460))
#define DDR3_CONFIG_REG_24    (*(unsigned int*)(0x02620464))
//#define KICK0                 (*(unsigned int*)(0x2620038))
//#define KICK1                 (*(unsigned int*)(0x262003C))
#define KICK0_UNLOCK          0x83E70B13
#define KICK1_UNLOCK          0x95A4F1E0
#define KICK0_LOCK            0
#define KICK1_LOCK            0
// DDR3 tuning registers
#define DATA0_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x043C))
#define DATA1_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0440))
#define DATA2_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0444))
#define DATA3_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0448))
#define DATA4_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x044C))
#define DATA5_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0450))
#define DATA6_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0454))
#define DATA7_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0458))
#define DATA8_GTLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x045C))

#define DATA0_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x040C))
#define DATA1_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0410))
#define DATA2_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0414))
#define DATA3_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0418))
#define DATA4_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x041C))
#define DATA5_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0420))
#define DATA6_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0424))
#define DATA7_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x0428))
#define DATA8_WRLVL_INIT_RATIO      (*(unsigned int*)(CHIP_LEVEL_REG + 0x042C))
// DDR3 definitions
#define DDR_BASE_ADDR               0x21000000
#define DDR_SDCFG                   (*(unsigned int*)(DDR_BASE_ADDR + 0x00000008))
#define DDR_SDRFC                   (*(unsigned int*)(DDR_BASE_ADDR + 0x00000010))
#define DDR_SDTIM1                  (*(unsigned int*)(DDR_BASE_ADDR + 0x00000018))
#define DDR_SDTIM2                  (*(unsigned int*)(DDR_BASE_ADDR + 0x00000020))
#define DDR_SDTIM3                  (*(unsigned int*)(DDR_BASE_ADDR + 0x00000028))
#define DDR_PMCTL                   (*(unsigned int*)(DDR_BASE_ADDR + 0x00000038))
#define DDR_ZQCFG                   (*(unsigned int*)(DDR_BASE_ADDR + 0x000000C8))
//#define RDWR_LVL_RMP_WIN            (*(unsigned int*)(DDR_BASE_ADDR + 0x000000D4))    //该宏与cslr_emif4f.h库文件中结构体成员重名，程序中也没用到，所以注销
#define DDR_RDWR_LVL_RMP_CTRL       (*(unsigned int*)(DDR_BASE_ADDR + 0x000000D8))
#define DDR_RDWR_LVL_CTRL           (*(unsigned int*)(DDR_BASE_ADDR + 0x000000DC))
#define DDR_DDRPHYC                 (*(unsigned int*)(DDR_BASE_ADDR + 0x000000E4))
//pll definitions
#define PLL_BASE_ADDR               0x02310000
#define PLL_PLLCTL                  (*(unsigned int*)(PLL_BASE_ADDR + 0x00000100))
#define PLL_SECCTL                  (*(unsigned int*)(PLL_BASE_ADDR + 0x00000108))
#define PLL_PLLM                    (*(unsigned int*)(PLL_BASE_ADDR + 0x00000110))
#define PLL_PLLCMD                  (*(unsigned int*)(PLL_BASE_ADDR + 0x00000138))
#endif


#endif /* INCLUDE_H_ */
