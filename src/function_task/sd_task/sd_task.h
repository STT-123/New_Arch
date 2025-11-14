#ifndef __SD_TASK_H__
#define __SD_TASK_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include "device_drv/sd_store/sd_store.h"


extern unsigned short g_ota_flag;

void SDCardDataSaveTaskCreate(void);

#endif