#ifndef __BMU_DEAL_H__
#define __BMU_DEAL_H__

#include "interface/queue/queue.h"
#include "interface/can/mycan.h"
#include "interface/epoll/myepoll.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include "interface/bms/bms_analysis.h"

#include "globalVariable.h"
#include <pthread.h>
#include "interface/log/log.h"

extern queue_t Queue_BMURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用


#ifdef yinhantong
    #define BMU_CAN_DEVICE_NAME "can3"
#else
    #define BMU_CAN_DEVICE_NAME "can1"
#endif

#define BMU_CAN_BITRATE 500000

bool bmu_Init(void);
int Drv_bmu_can_send(CAN_MESSAGE *pFrame);
int Drv_bmu_canfd_send(struct canfd_frame *cansend_data);

#endif