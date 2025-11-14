#ifndef __SD_STORE_H__
#define __SD_STORE_H__

#include "stdbool.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

#include <sys/statvfs.h>
#include <dirent.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
#include "interface/G_GloabalVariable.h"

#define MAX_FILES_IN_FOLDER 80
#define BUFFER_SIZE 100 // 每个环形缓冲区的容量
#define BUFFERED_WRITE_SIZE 1024
#define CHECKSD_TRIGGERING_TIME 60000 * 1000

#define  SDMAXCAPACITY 90
#define CAN_ID_HISTORY_SIZE 6
typedef struct
{
    uint32_t relativeTimestamp; // 相对时间戳
    CANFD_MESSAGE msg;
    unsigned char channel;
} CAN_LOG_MESSAGE;

typedef struct
{
    CAN_LOG_MESSAGE buffer[BUFFER_SIZE];
    size_t writeIndex;
    size_t readIndex;
    size_t count;          // 当前缓冲区中的消息数量
    pthread_mutex_t mutex; // 保护缓冲区访问的互斥量
} RingBuffer;

typedef struct
{
    RingBuffer buffers[2];
    int activeBuffer;            // 当前活跃缓冲区索引
    pthread_mutex_t switchMutex; // 用于保护缓冲区切换
} DoubleRingBuffer;


static int  Drv_check_and_update_message(const CANFD_MESSAGE *msg);
static void Drv_write_canmsg_cache_to_file(FILE *file, uint32_t timestamp_ms);
static void Drv_RTCGetTime(Rtc_Ip_TimedateType *rtcTime);
static int  mount_sdcard_fat32(void);
static int  judgeTimetoUpdate(void);

int  SD_Initialize(void);
int  ensure_mount_point(const char *path);
void Drv_write_to_active_buffer(const CANFD_MESSAGE *msg, uint8_t channel);
void Drv_write_buffer_to_file(void);
void checkSDCardCapacity(void);
void sd_storeInit(void);
#endif