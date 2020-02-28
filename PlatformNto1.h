

#ifndef PLATFORMNTO1_H_
#define PLATFORMNTO1_H_

#include "stdio.h"
#include "tistdtypes.h"
#include "cslr_device.h"
#include "cslr_uart.h"
#include "cslr_tpcc.h"
#include <cslr_cgem.h>
/*************************************************************************************************************/
//lib版本查询
/*************************************************************************************************************/
/*****************************************************************************
 Prototype    : LibVersion
 Description  : lib当前版本查询。
 Input        : 无
 Output       : 无
 Return Value : unsigned int，版本号。
*****************************************************************************/
extern unsigned int LibVersion(void);



#endif /* PLATFORMNTO1_H_ */
