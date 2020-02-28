#include <C6x.h>
#include "KeyStone_common.h"
#include "PlatformNto1.h"

const unsigned int CoreFreqHz = 1000000000;

void delay_cycles(unsigned long long cycles){
	unsigned long long start_val, stop_val;
	TSCL = 0;
	TSCH = 0;
	start_val =_itoll(TSCH, TSCL);
	do{
		stop_val = _itoll(TSCH,TSCL);
	}while ((stop_val - start_val) < cycles);
}


void delay_us( unsigned int us){
	unsigned long long cycles;

	cycles = (unsigned long long)us * CoreFreqHz / 1000000;
	delay_cycles(cycles);
}


void delay_ms( unsigned int ms){
	unsigned long long cycles;

	cycles = (unsigned long long)ms * CoreFreqHz / 1000;
	delay_cycles(cycles);
}

//us定时
void Timer64Local_Init(Timer64Config config)
{
	Timer64_Config tmrCfg;
	TTimerMode mod;

	switch(config.timerMode)
	{
		case TIMER64_ONE_SHOT_PULSE: mod = TIMER_ONE_SHOT_PULSE;break;
		case TIMER64_PERIODIC_PULSE: mod = TIMER_PERIODIC_PULSE;break;
		case TIMER64_PERIODIC_CLOCK: mod = TIMER_PERIODIC_CLOCK;break;
		case TIMER64_WATCH_DOG: mod = TIMER_WATCH_DOG;break;
		default:break;
	}
	tmrCfg.timer_num = config.timer_num;
	tmrCfg.timerMode = mod;
	//tmrCfg.period = (unsigned long long)(config.period * CoreFreqHz / 6000000);//1000M/6为1s定时器计数个数，1000M/6000000为定时器1us定时个数,最小定时1us，目前
	tmrCfg.period = (unsigned long long)config.period;
	tmrCfg.pulseWidth = 3;
	Timer64_Init(&tmrCfg);
}

void Timer64Local_Reset(Timer64Config config)
{
	Reset_Timer(config.timer_num);
}


//无BIOS下，core中断配置
int Timer64Local_InterruptCore_Init(unsigned int IntNum)
{
	if((IntNum < 4) || (IntNum > 15))
	{
		return 0;
	}

	unsigned int *INTMUX_Pt = (unsigned int *)(&(gpCGEM_regs->INTMUX1) + IntNum / 4 - 1);
	unsigned int INTMUX_Index = IntNum % 4 * 8;
	*INTMUX_Pt |=  (CSL_GEM_TINTLN << INTMUX_Index);

	IER |= (1 << IntNum);
	return 1;
}

//开始配置中断时调用该函数
void InterruptCore_GlobleInit(void)
{
	gpCGEM_regs->INTMUX1 = 0;
	gpCGEM_regs->INTMUX2 = 0;
	gpCGEM_regs->INTMUX3 = 0;//不清零中断配置不上

	gpCGEM_regs->EVTMASK[0] = 0xFFFFFFFF;
	gpCGEM_regs->EVTMASK[1] = 0xFFFFFFFF;
	gpCGEM_regs->EVTMASK[2] = 0xFFFFFFFF;
	gpCGEM_regs->EVTMASK[3] = 0xFFFFFFFF;
}
//所有中断配置完成后，最后执行该函数
void InterruptCore_GlobleEn(void)
{
	/*Clear all DSP core events*/
	gpCGEM_regs->EVTCLR[0]= 	0xFFFFFFFF;
	gpCGEM_regs->EVTCLR[1]= 	0xFFFFFFFF;
	gpCGEM_regs->EVTCLR[2]= 	0xFFFFFFFF;
	gpCGEM_regs->EVTCLR[3]= 	0xFFFFFFFF;

	//clear DSP core interrupt flag
	ICR = IFR;
	IER |= 3;
	/*Interrupt Service Table Pointer to begining of LL2 memory*/
	ISTP =  0x00800000;   //中断向量表
	//enable GIE
	TSR |= 1;
}
