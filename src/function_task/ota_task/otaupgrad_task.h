#ifndef __OTA_UPGRAD_TASK_H__
#define __OTA_UPGRAD_TASK_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include "device_drv/ota_upgrade/ota_fun.h"
#define BMUMAXNUM 15
void ota_Upgrade_TaskCreate(void);
#endif