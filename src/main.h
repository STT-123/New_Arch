#ifndef __MAIN_H__
#define __MAIN_H__

#include "interface/G_GloabalVariable.h"
#define LOGO 0x9528  //公司标识
#define ECU_VERSION 0x115 //  表示v1.01 格式xx。yy xx 高八位yy低八位
#define yinhantong //  映翰通标志位
extern OTAObject otactrl;
static void printf_version(void);
#endif