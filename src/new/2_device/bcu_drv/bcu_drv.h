#ifndef __BCU_DRV_H__
#define __BCU_DRV_H__

#include "lib/queue/queue.h"
#include "new/1_interface/can/mycan.h"
#include "./new/1_interface/epoll/myepoll.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "../BMS/C_BMSAnalysis.h"
#include "../GLB/G_GloabalVariable.h"
#include <pthread.h>

#ifdef yinhantong
    #define BCU_CAN_DEVICE_NAME "can2"
#else
    #define BCU_CAN_DEVICE_NAME "can0"
#endif
#define BCU_CAN_BITRATE 500000

extern queue_t Queue_BCURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用
extern queue_t Queue_BCURevData_FD;

bool BCU_Init(void);
int Drv_bcu_can_send(CAN_MESSAGE *pFrame);
int Drv_bcu_canfd_send(CAN_FD_MESSAGE_BUS *pFrame);
bool Drv_BMS_Analysis(unsigned char can_flag);

#endif