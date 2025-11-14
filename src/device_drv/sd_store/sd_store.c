#define _GNU_SOURCE
#include "sd_store.h"
#include "interface/globalVariable.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "device_drv/abncheck/abncheck.h"
#include "interface/log/log.h"
#include "interface/bms/bms_simulink/CANRcvFcn.h"
#include <time.h>


DoubleRingBuffer canDoubleRingBuffer;
static bool newFileNeeded = true;
Rtc_Ip_TimedateType initialTime;
Rtc_Ip_TimedateType currentTime;
struct timespec start_tick;

uint32_t CAN_IDs[] = {
    0x180110E4,
    0x180210E4,
    0x180310E4,
    0x180410E4,
    0x1A0110E4,
    0x1B0110E4,
};
 CAN_MESSAGE can_msg_1A0110E4_cache[8]; // 单体电压，一包30个，一共240个，索引分8帧
 CAN_MESSAGE can_msg_1B0110E4_cache[2]; // 单体温度，一包60个，一共120个，索引分2帧
 CAN_MESSAGE can_msg_180410E4_cache[1]; // bmu电压，AFE温度，一包15个，一共15个，索引分1帧
 CAN_MESSAGE can_msg_cache[CAN_ID_HISTORY_SIZE] = {0};

/**
 * 
 * 检查U盘是否可用   0正常 1不正常
 * 
*/

static int mount_sdcard_fat32(void)
{
    const char *device = USB_DEVICE;
    const char *mount_point = USB_MOUNT_POINT ;
    int ret;
    
    //LOG("[SD Card] 开始挂载FAT32格式的SD卡...\n");
    
    // 1. 检查设备是否存在
    struct stat st;
    if (stat(device, &st) == -1) {
        LOG("[SD Card] 错误: 设备 %s 不存在\n", device);
        return -1;
    }
    
    // 2. 检查是否已挂载
    FILE *fp = fopen("/proc/mounts", "r");
    if (fp) {
        char line[256];
        while (fgets(line, sizeof(line), fp)) {
            if (strstr(line, mount_point)) {
                fclose(fp);
                //LOG("[SD Card] SD卡已经挂载在 %s\n", mount_point);
                return 0;
            }
        }
        fclose(fp);
    }
    
    // 3. 创建挂载点
    if (mkdir(mount_point, 0755) == -1) {
        if (errno != EEXIST) {
            LOG("[SD Card] 创建挂载点失败: %s\n", strerror(errno));
            return -1;
        }
    }
    
    ret = mount(device, mount_point, "vfat", 0, "iocharset=utf8");
    
    if (ret == 0) {
        LOG("[SD Card] FAT32 SD卡挂载成功\n");
        return 0;
    } else {
        LOG("[SD Card] FAT32挂载失败: %s\n", strerror(errno));
        
        // 尝试不带字符集参数
        LOG("[SD Card] 尝试不带字符集参数挂载...\n");
        ret = mount(device, mount_point, "vfat", 0, NULL);
        if (ret == 0) {
            LOG("[SD Card] FAT32 SD卡挂载成功(无字符集)\n");
            return 0;
        } else {
            LOG("[SD Card] 最终挂载失败: %ensure_mount_points\n", strerror(errno));
            return -1;
        }
    }
}

/**
 * 获取本地时间
*/
static void Drv_RTCGetTime(Rtc_Ip_TimedateType *rtcTime)
{
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);

    rtcTime->year = tm_info->tm_year + 1900;
    rtcTime->month = tm_info->tm_mon + 1;
    rtcTime->day = tm_info->tm_mday;
    rtcTime->hour = tm_info->tm_hour;
    rtcTime->minutes = tm_info->tm_min;
    rtcTime->seconds = tm_info->tm_sec;
}
/**
 * 获取当前时间
 * 
*/

static int GetNowTime(struct tm *nowTime)
{
    struct tm timeinfo = {0};
    static time_t last_update_time = 0;
    time_t current_time = time(NULL);
    // 检查时间
    if (get_BCU_TimeYearValue() != 0) // bcu发来时间了
    {
        // 填充外部时间变量到 tm 结构体
        timeinfo.tm_year = get_BCU_TimeYearValue() + 100;
        timeinfo.tm_mon = get_BCU_TimeMonthValue() - 1;
        timeinfo.tm_mday = get_BCU_TimeDayValue();
        timeinfo.tm_hour = get_BCU_TimeHourValue();
        timeinfo.tm_min = get_BCU_TimeMinuteValue();
        timeinfo.tm_sec = get_BCU_TimeSencondValue();
        timeinfo.tm_isdst = -1;
        if (mktime(&timeinfo) == (time_t)-1) {
            LOG("[SD Card] WARNING: mktime failed for BCU time\n");
            // 设置一个默认的星期几
            timeinfo.tm_wday = 0; // 星期日
        }
        //如果是第一次更新或者距离上次更新超过1分钟
        if (last_update_time == 0 || (current_time - last_update_time) >= 60) // 60秒 = 1分钟
        {
            // 执行实际的时间更新操作
            set_system_time_from_bcu();
            LOG("[SD Card] update time\r\n");
            // 更新最后更新时间
            last_update_time = current_time;
        }
        // LOG("[SD Card] [SD Card] Time Source From Bcu");
    }
    else // bcu没发过来时间 用自己本地的时间
    {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        timeinfo = *tm_info;
        mktime(&timeinfo);
        LOG("[SD Card] [SD Card] Time Source From Local");
    }

    // 得到当前时间
    *nowTime = timeinfo;
    // LOG("[SD Card] [SD Card] Now Time: %d-%d-%d %d:%d:%d. ", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return 0;
}
/**
 * 
 * 
 * 使用一个时间创建一个文件路径（文件夹+文件名）
*/ 
static int CreateAscFilePathWithTime(struct tm timeInfo, char *filePath)
{
    // 创建文件夹路径
    char folderPath[256];
    snprintf(folderPath, sizeof(folderPath), "%s/%04d%02d%02d",
             USB_MOUNT_POINT,         // 挂载点
             timeInfo.tm_year + 1900, // 年
             timeInfo.tm_mon + 1,     // 月
             timeInfo.tm_mday);       // 日

    // 创建目录（如果不存在）
    struct stat st = {0};
    if (stat(folderPath, &st) == -1)
    {
        if (mkdir(folderPath, 0777) == -1)
        {
            perror("mkdir failed");
            LOG("[SD Card] [SD Card] mkdir failed");
            return 1;
        }
    }

    // 生成完整文件路径（年月日时分秒）
    sprintf(filePath, "%s/%04d%02d%02d%02d%02d%02d.asc",
            folderPath,              // 文件夹路径
            timeInfo.tm_year + 1900, // 年
            timeInfo.tm_mon + 1,     // 月
            timeInfo.tm_mday,        // 日
            timeInfo.tm_hour,        // 时
            timeInfo.tm_min,         // 分
            timeInfo.tm_sec);        // 秒

    LOG("[SD Card] new asc file path is = %04d%02d%02d%02d%02d%02d. \n",
        timeInfo.tm_year + 1900,
        timeInfo.tm_mon + 1,
        timeInfo.tm_mday,
        timeInfo.tm_hour,
        timeInfo.tm_min,
        timeInfo.tm_sec);

    return 0;
}

static int OpenNowWriteAscFile(const char *filePath, FILE **file) {
    static int failed_count = 0;
    
    if (!filePath || !file) {
        LOG("[SD Card] ERROR: Invalid parameters to OpenNowWriteAscFile\n");
        return 1;
    }
    
    //printf(">>> Attempting to open file: %s\n", filePath);
    
    *file = fopen(filePath, "ab");
    
    if (!*file) {
        int err = errno;
        LOG("[SD Card] ERROR: fopen failed for %s, errno=%d (%s)\n", filePath, err, strerror(err));
        failed_count++;
        
        if (failed_count >= 5) {
            LOG("[SD Card] OpenNowWriteAscFile %s failed %d times, errno=%d\n", filePath, failed_count, err);
            set_emcu_fault(SD_FAULT, SET_ERROR);
        }
        return 1;
    }

    if (failed_count >= 5) {
        LOG("[SD Card] OpenNowWriteAscFile %s recovered after %d failures\n", filePath, failed_count);
        set_emcu_fault(SD_FAULT, SET_RECOVER);
    }
    failed_count = 0;
    // LOG("[SD Card] SUCCESS: File opened successfully: %p\n", (void*)*file);
    return 0;
}

/**
 * 给ASC文件写一固定的时间头
 * 
 * */ 
static int AscFileWriteTimeHeader(FILE *file, struct tm *timeinfo)
{
    LOG("[SD Card] === AscFileWriteTimeHeader START ===\n");
    
    if (file == NULL)
    {
        LOG("[SD Card] CRITICAL ERROR: File pointer is NULL in AscFileWriteTimeHeader\n");
        return -1;
    }
    
    if (timeinfo == NULL)
    {
        LOG("[SD Card] CRITICAL ERROR: timeinfo is NULL in AscFileWriteTimeHeader\n");
        return -1;
    }
    
    // 验证时间字段的合理性
    LOG("[SD Card] Time info: wday=%d, mon=%d, mday=%d, hour=%d, min=%d, sec=%d, year=%d\n",
           timeinfo->tm_wday, timeinfo->tm_mon, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
           timeinfo->tm_year);
    
    char header[512] = {0}; // 增大缓冲区确保安全
    const char *weekDays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    // 检查数组索引边界
    if (timeinfo->tm_wday < 0 || timeinfo->tm_wday > 6) {
        LOG("[SD Card] ERROR: Invalid tm_wday: %d\n", timeinfo->tm_wday);
        return -1;
    }
    if (timeinfo->tm_mon < 0 || timeinfo->tm_mon > 11) {
        LOG("[SD Card] ERROR: Invalid tm_mon: %d\n", timeinfo->tm_mon);
        return -1;
    }

    // 使用安全的 snprintf 一次性构建整个头部
    int total_len = snprintf(header, sizeof(header),
                            "date %s %s %02d %02d:%02d:%02d %s %04d\r\n"
                            "base hex timestamps absolute\r\n"
                            "// version 7.0.0\r\n",
                            weekDays[timeinfo->tm_wday],
                            months[timeinfo->tm_mon],
                            timeinfo->tm_mday,
                            (timeinfo->tm_hour > 12) ? (timeinfo->tm_hour - 12) : 
                             (timeinfo->tm_hour == 0) ? 12 : timeinfo->tm_hour, // 处理 0 点的情况
                            timeinfo->tm_min, 
                            timeinfo->tm_sec,
                            (timeinfo->tm_hour >= 12) ? "PM" : "AM",
                            timeinfo->tm_year + 1900);
    
    LOG("[SD Card] Header length: %d, buffer size: %zu\n", total_len, sizeof(header));
    
    if (total_len < 0) {
        LOG("[SD Card] ERROR: snprintf failed\n");
        return -1;
    }
    
    if ((size_t)total_len >= sizeof(header)) {
        LOG("[SD Card] ERROR: Header too long: %d >= %zu\n", total_len, sizeof(header));
        return -1;
    }
    
    LOG("[SD Card] Header content:\n%s", header);
    
    // 写入文件
    size_t written = fwrite(header, 1, total_len, file);
    LOG("[SD Card] Bytes written: %zu, expected: %d\n", written, total_len);
    
    if (written != (size_t)total_len)
    {
        LOG("[SD Card] WARNING: Header not fully written to file: %zu != %d\n", written, total_len);
        return -1;
    }
    
    fflush(file); // 确保数据写入磁盘
    LOG("[SD Card] === AscFileWriteTimeHeader COMPLETED SUCCESSFULLY ===\n");
    return 0;
}


// 查找指定CAN ID的历史消息，并与当前消息对比
static int Drv_check_and_update_message(const CANFD_MESSAGE *msg)
{
    for (int i = 0; i < CAN_ID_HISTORY_SIZE; i++)
    {
        // printf("msg->ID :%x\r\n",msg->ID);
        if (can_msg_cache[i].ID == msg->ID)
        {
            if (msg->ID == 0x1A0110E4)
            {
                if ((msg->Data[0] > 8) || (msg->Data[0] == 0))
                {
                    return 0;
                }
                if (memcmp(&can_msg_1A0110E4_cache[msg->Data[0] - 1].Data, msg->Data, 64) == 0)
                {
                    return 0; // 表示消息未修改
                }
                else
                {
                    memcpy(&can_msg_1A0110E4_cache[msg->Data[0] - 1].Data, msg->Data, 64);
                    return 1; // 表示消息已修改
                }
            }
            if (msg->ID == 0x1B0110E4)
            {
                if ((msg->Data[0] > 2) || (msg->Data[0] == 0))
                {
                    return 0;
                }
                if (memcmp(&can_msg_1B0110E4_cache[msg->Data[0] - 1].Data, msg->Data, 64) == 0)
                {
                    return 0; // 表示消息未修改
                }
                else
                {
                    memcpy(&can_msg_1B0110E4_cache[msg->Data[0] - 1].Data, msg->Data, 64);
                    return 1; // 表示消息已修改
                }
            }
            if (msg->ID == 0x180410E4)
            {
                if ((msg->Data[0] > 1) || (msg->Data[0] == 0))
                {
                    return 0;
                }
                if (memcmp(&can_msg_180410E4_cache[msg->Data[0] - 1].Data, msg->Data, 64) == 0)
                {
                    return 0; // 表示消息未修改
                }
                else
                {
                    memcpy(&can_msg_180410E4_cache[msg->Data[0] - 1].Data, msg->Data, 64);
                    return 1; // 表示消息已修改
                }
            }

            if (memcmp(&can_msg_cache[i].Data, msg->Data, 64) == 0)
            {
                return 0; // 表示消息未修改
            }
            else
            {
                // 如果消息不同，则更新历史消息
                memcpy(&can_msg_cache[i].Data, msg->Data, 64);
                return 1; // 表示消息已修改
            }
        }
    }
    return 0; // 如果未找到该 CAN ID
}


static void Drv_write_canmsg_cache_to_file(FILE *file, uint32_t timestamp_ms)
{
    if (file == NULL)
    {
        LOG("[SD Card] Error: File pointer is NULL.\n");
        return;
    }

    unsigned short index = 0;
    for (int i = 0; i < CAN_ID_HISTORY_SIZE; i++)
    {
        CAN_MESSAGE *logMsg = &can_msg_cache[i];
        if (CAN_IDs[i] == 0x1A0110E4)
        {
            logMsg = &can_msg_1A0110E4_cache[index++];
            if (index < 8)
            {
                i--;
            }
            else
            {
                index = 0;
            }
        }
        else if (CAN_IDs[i] == 0x1B0110E4)
        {
            logMsg = &can_msg_1B0110E4_cache[index++];
            if (index < 2)
            {
                i--;
            }
            else
            {
                index = 0;
            }
        }
        else if (CAN_IDs[i] == 0x180410E4)
        {
            logMsg = &can_msg_180410E4_cache[index++];
            if (index < 1)
            {
                i--;
            }
            else
            {
                index = 0;
            }
        }
        
        char timeStampedMessage[BUFFERED_WRITE_SIZE] = {0};// 构造带时间戳的字符串
        unsigned char offset = 0;

        // 时间戳
        offset += snprintf(timeStampedMessage + offset, sizeof(timeStampedMessage),
                           "%d.%03d 1 ", timestamp_ms / 1000, timestamp_ms % 1000);
        // ID + 长度
        offset += snprintf(timeStampedMessage + offset, sizeof(timeStampedMessage),
                           "%03Xx Rx d %d ", logMsg->ID, logMsg->Length);
        
        for (int j = 0; j < logMsg->Length; ++j)// 数据
        {
            offset += snprintf(timeStampedMessage + offset, 4,
                               "%02X ", logMsg->Data[j]);
        }
        
        offset += snprintf(timeStampedMessage + offset, 4, "\r\n");// 换行
        
        size_t err = fseek(file, 0, SEEK_END);// 写入文件//阻塞
        if (err != 0)
        {
            perror("fseek");
            LOG("[SD Card] Failed to seek to end of file\n");
            return;
        }

        err = fwrite(timeStampedMessage, 1, offset, file);
        if (err != offset)
        {
            LOG("[SD Card] Failed to write to file\n");
            return;
        }
    }
    fflush(file);
}
static int judgeTimetoUpdate(void)
{
    int ret = 0;
    static int last_year = 0;
    static int last_month = 0;
    static int last_day = 0;
    
    LOG("[SD Card] Current time: %d-%d-%d, Last time: %d-%d-%d\n", 
        get_BCU_TimeYearValue(), get_BCU_TimeMonthValue(), get_BCU_TimeDayValue(),
        last_year, last_month, last_day);

    // 检查时间数据是否有效
    if (get_BCU_TimeYearValue() == 0 || get_BCU_TimeMonthValue() == 0 || get_BCU_TimeDayValue() == 0) {
        LOG("[SD Card] Time invalid, skip check\n");
        return 0;
    }

    // 变化检查
    if ((get_BCU_TimeYearValue() != last_year) || 
        (get_BCU_TimeMonthValue() != last_month) || 
        (get_BCU_TimeDayValue() != last_day)) 
    {
        
        LOG("[SD Card] TIME CHANGE DETECTED: %d-%d-%d -> %d-%d-%d\n", 
            last_year, last_month, last_day,
            get_BCU_TimeYearValue(), get_BCU_TimeMonthValue(), get_BCU_TimeDayValue());

        last_year = get_BCU_TimeYearValue();
        last_month = get_BCU_TimeMonthValue();
        last_day = get_BCU_TimeDayValue();
        ret = 1;
    }
    
    return ret;
}




static char *Drv_my_strdup(const char *str)
{
    if (!str)
        return NULL;
    char *dup = malloc(strlen(str) + 1);
    if (dup)
        strcpy(dup, str);
    return dup;
}

// 递归删除目录及内容
static int Drv_remove_directory(const char *path)
{
    DIR *dir = opendir(path);
    struct dirent *entry;
    char subPath[512];

    if (!dir)
        return -1;

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        snprintf(subPath, sizeof(subPath), "%s/%s", path, entry->d_name);

        struct stat statbuf;
        if (stat(subPath, &statbuf) == 0)
        {
            if (S_ISDIR(statbuf.st_mode))
            {
                Drv_remove_directory(subPath); // 递归删除
            }
            else
            {
                unlink(subPath); // 删除文件
            }
        }
    }

    closedir(dir);
    return rmdir(path); // 删除目录
}

static int CompareFolderNames(const void *a, const void *b)
{
    return strcmp(*(const char **)a, *(const char **)b);
}

// 删除最旧的文件夹（假设文件夹名为日期字符串）
static void Func_DeleteOldestFolder(void)
{
    DIR *dir = opendir(USB_MOUNT_POINT);
    struct dirent *entry;
    char *folders[100];
    int folderCount = 0;

    if (!dir)
    {
        perror("opendir");
        return;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") != 0 &&
            strcmp(entry->d_name, "..") != 0 &&
            strcmp(entry->d_name, "19700101") != 0)
        {

            // 用 stat 判断是不是目录
            char fullPath[512];
            snprintf(fullPath, sizeof(fullPath), "%s/%s", USB_MOUNT_POINT, entry->d_name);

            struct stat st;
            if (stat(fullPath, &st) == 0 && S_ISDIR(st.st_mode))
            {
                folders[folderCount] = Drv_my_strdup(entry->d_name);
                if (!folders[folderCount])
                    break;
                folderCount++;
            }
        }
    }

    closedir(dir);

    if (folderCount > 0)
    {
        qsort(folders, folderCount, sizeof(char *), CompareFolderNames);

        char path[512];
        snprintf(path, sizeof(path), "%s/%s", USB_MOUNT_POINT, folders[0]);

        LOG("[SD Card] Deleting oldest folder: %s\n", path);
        Drv_remove_directory(path);

        for (int i = 0; i < folderCount; i++)
        {
            free(folders[i]);
        }
    }
}


static void Drv_init_can_id_history(void)
{
    int i = 0;
    for (i = 0; i < CAN_ID_HISTORY_SIZE; i++)
    {
        can_msg_cache[i].ID = CAN_IDs[i];
        can_msg_cache[i].Length = 64;
    }
    for (i = 0; i < 8; i++)
    {
        can_msg_1A0110E4_cache[i].ID = 0x1A0110E4;
        can_msg_1A0110E4_cache[i].Length = 64;
        can_msg_1A0110E4_cache[i].Data[0] = i + 1;
    }
    for (i = 0; i < 2; i++)
    {
        can_msg_1B0110E4_cache[i].ID = 0x1B0110E4;
        can_msg_1B0110E4_cache[i].Length = 64;
        can_msg_1B0110E4_cache[i].Data[0] = i + 1;
    }
    for (i = 0; i < 1; i++)
    {
        can_msg_180410E4_cache[i].ID = 0x180410E4;
        can_msg_180410E4_cache[i].Length = 64;
        can_msg_180410E4_cache[i].Data[0] = i + 1;
    }
}

static void Drv_init_double_ring_buffer(DoubleRingBuffer *drb)
{
    for (int i = 0; i < 2; ++i)
    {
        drb->buffers[i].writeIndex = 0;
        drb->buffers[i].readIndex = 0;
        drb->buffers[i].count = 0;
        pthread_mutex_init(&drb->buffers[i].mutex, NULL);
    }
    drb->activeBuffer = 0;
    pthread_mutex_init(&drb->switchMutex, NULL);
}
/*=================================外部调用函数========================================*/

void sd_storeInit(void)
{ 
    Drv_init_double_ring_buffer(&canDoubleRingBuffer); // 初始化往sd卡写数据用的双环形缓冲区
    Drv_init_can_id_history();                         // 初始化往SD卡写的can消息的缓存区
}

/**
 * 确认挂载点
*/
int ensure_mount_point(const char *path)
{
    struct stat st;
    if (stat(path, &st) == -1)
    {
        if (mkdir(path, 0777) == -1)
        {
            perror("创建挂载目录失败");
            LOG("[SD Card] 请检查挂载点路径是否正确\n");
            return -1;
        }
        else
        {
            LOG("[SD Card] 创建挂载目录成功\n");
        }
    }
    else
    {
        LOG("[SD Card] 挂载点已存在\n");
    }
    return 0;
}
/**
 * 初始化缓存
*/

void Drv_write_to_active_buffer(const CANFD_MESSAGE *msg, uint8_t channel)
{
    DoubleRingBuffer *drb = &canDoubleRingBuffer;

    if (((msg->ID == 0x1cb0e410) && (msg->Data[0] == 0xC9)) ||
        (msg->ID == 0x1cb010e4) || (msg->ID == 0x1823E410) || (msg->ID == 0))
    {
        return;
    }

    if (Drv_check_and_update_message(msg) == 0)
    {
        return; // 临时取消
    }

    pthread_mutex_lock(&drb->switchMutex);
    RingBuffer *activeBuffer = &drb->buffers[drb->activeBuffer];
    pthread_mutex_lock(&activeBuffer->mutex);

    CAN_LOG_MESSAGE *logMsg = &activeBuffer->buffer[activeBuffer->writeIndex];
    logMsg->relativeTimestamp = GetTimeDifference_ms(start_tick);
    memcpy(&logMsg->msg, msg, sizeof(CANFD_MESSAGE));
    logMsg->channel = channel;

    activeBuffer->writeIndex = (activeBuffer->writeIndex + 1) % BUFFER_SIZE;

    if (activeBuffer->count == BUFFER_SIZE)
    {
        activeBuffer->readIndex = (activeBuffer->readIndex + 1) % BUFFER_SIZE;
    }
    else
    {
        activeBuffer->count++;
    }

    pthread_mutex_unlock(&activeBuffer->mutex);
    pthread_mutex_unlock(&drb->switchMutex);
}


// 将缓冲区数据写到sd卡
void Drv_write_buffer_to_file(void)
{
    static char filePath[512] = {0}; // 当前使用的文件路径
    int ret = 0;
    // 根据当前时间创建一个文件路径
    struct tm nowTimeInfo = {0};
    DoubleRingBuffer *drb = &canDoubleRingBuffer;

    // 获取当前时间
    GetNowTime(&nowTimeInfo);

    // 交换当前使用的缓冲区
    pthread_mutex_lock(&drb->switchMutex);
    drb->activeBuffer = 1 - drb->activeBuffer;
    pthread_mutex_unlock(&drb->switchMutex);

    // 获取需要写入的缓冲区
    int inactiveBufferIndex = 1 - drb->activeBuffer;
    RingBuffer *inactiveBuffer = &drb->buffers[inactiveBufferIndex];

    // 获取文件互斥锁
    ret = pthread_mutex_lock(&inactiveBuffer->mutex);
    if (ret != 0)
    {
        LOG("[SD Card] write_buffer_to_file end return. \n");
        return;
    }
    
    if (mount_sdcard_fat32() != 0)// 先检查存储器状态 不存在 标记错误 直接退出
    {
        LOG("[SD Card] SD_FAULT\r\n");
        set_emcu_fault(SD_FAULT, SET_ERROR);
        goto QUIT_FLAG;
    }
   

    checkSDCardCapacity();//检测SD 卡容量


    if (newFileNeeded)// 判断是否需要重新创建一个文件开始写
    {
        CreateAscFilePathWithTime(nowTimeInfo, filePath);// 将时间转换为文件路径
        filePath[sizeof(filePath) - 1] = '\0';
    }
  
    FILE *file = NULL;// 打开目标文件
    
    if (OpenNowWriteAscFile(filePath, &file) != 0  || file == NULL)
    {
        LOG("[SD Card] ERROR: OpenNowWriteAscFile failed for: %s\n", filePath);
        goto QUIT_FLAG; // 打开失败 直接返回
    }
  
    if (newFileNeeded)// 如果是新创建的文件
    {
        LOG("[SD Card] 9. Writing headers for new file\n");
        // 先写入当前文件头
        if (AscFileWriteTimeHeader(file, &nowTimeInfo) != 0) {
            LOG("[SD Card] ERROR: Failed to write time header\n");
        } else {
            //printf("9.1 Time header written\n");
        }
        
        Drv_write_canmsg_cache_to_file(file, 0) ;// 缓冲区数据写入  
        newFileNeeded = false;// 标记不需要重新创建了      
    }

    // 如果不是新创建的文件 从文件的末尾追加写入
    fseek(file, 0, SEEK_END);
    while (inactiveBuffer->count > 0)
    {
        CAN_LOG_MESSAGE *logMsg = &inactiveBuffer->buffer[inactiveBuffer->readIndex];

        char dataStr[3 * 64 + 1] = {0};
        int bytes = logMsg->msg.Length;
        if (bytes > 64) bytes = 64; // 双保险

        for (int i = 0; i < bytes; ++i) {
            // 每次写 3 个字符："%02X "，最多写入 3*64=192 字符，留了 +1 结尾
            // 这里用 sizeof(dataStr) - i*3 做保护，避免意外越界
            snprintf(&dataStr[i * 3], (size_t)(sizeof(dataStr) - i * 3), "%02X ", logMsg->msg.Data[i]);
        }
        // 2) 方向标记
        const char *dir = (logMsg->channel == 0) ? "Rx" : "Tx";
        // 3) 直接一次性写入最终字符串（避免中间缓冲 + memmove）
        char line[BUFFERED_WRITE_SIZE]; // 请把 BUFFERED_WRITE_SIZE 设为 >= 512
        int lineLen = snprintf(
            line, sizeof(line),
            "%d.%03d 1 %08lX %s d %d %s\r\n",
            logMsg->relativeTimestamp / 1000,
            logMsg->relativeTimestamp % 1000,
            (unsigned long)logMsg->msg.ID,
            dir,
            logMsg->msg.Length,
            dataStr
        );
        if (lineLen < 0 || (size_t)lineLen >= sizeof(line)) 
        {
            // 被截断或出错：可以选择丢弃，或写一条告警，再继续下一条
            // fprintf(stderr, "line truncated or error, drop this frame\n");
            // 安全起见我们直接丢弃这一帧，避免潜在越界
            inactiveBuffer->readIndex = (inactiveBuffer->readIndex + 1) % BUFFER_SIZE;
            inactiveBuffer->count--;
            continue;
        }

        size_t written = fwrite(line, 1, (size_t)lineLen, file);
        (void)written; // 如需可检查 written == (size_t)lineLen

        inactiveBuffer->readIndex = (inactiveBuffer->readIndex + 1) % BUFFER_SIZE;
        inactiveBuffer->count--;

    }
    fflush(file);

    fseek(file, 0, SEEK_END);// 写完之后 计算文件大小
    long fileSize = ftell(file);

    // 创建新文件的两个条件
    // 1. 当前写的文件大小超过10M
    // 2. 系统中不存在当前日志命名的文件夹（日期变化了）
    if ((fileSize > (10*1024*1024) )|| (judgeTimetoUpdate())) // 大于10M或者年月日发生变化
    {
        LOG("[SD Card] fileSize = %ld\r\n",fileSize);

        LOG("[SD Card] judgeTimetoUpdate = %d\r\n",judgeTimetoUpdate());
        newFileNeeded = true; // 下一轮就要创建新文件
    }
    
    fclose(file);// 关闭文件

QUIT_FLAG:
    pthread_mutex_unlock(&inactiveBuffer->mutex);

    return;
}
int SD_Initialize(void)
{
    int res;
    const char *device = USB_DEVICE;
    const char *mount_point = USB_MOUNT_POINT;

    res = system(UNMOUNT_POINT);

    if (res != 0)
    {
        LOG("[SD Card] umount failed (maybe not mounted):%d\n", res);
        return res;
    }
    else
    {
        LOG("[SD Card] umount success\n");
    }

    char cmd[256];

    // 使用 FAT32 而不是 NTFS
    snprintf(cmd, sizeof(cmd), "mkfs.vfat -F 32 %s", device);

    res = system(cmd);
    if (res != 0)
    {
        LOG("[SD Card] Format failed: %d\n", res);
        return res;
    }
    else
    {
        LOG("[SD Card] Format (FAT32) success\n");
    }

    snprintf(cmd, sizeof(cmd), "mount %s %s", device, mount_point);
    res = system(cmd);
    if (res != 0)
    {
        LOG("[SD Card] Mount failed: %d\n", res);
        return res;
    }
    else
    {
        LOG("[SD Card] Mount success\n");
    }

    usleep(100 * 1000);
    newFileNeeded = true;

    return 0;
}


// 检查SD卡容量并删除旧文件夹的线程任务
void checkSDCardCapacity(void)
{

    struct statvfs stat;
    if (statvfs(USB_MOUNT_POINT, &stat) != 0)
    {
        LOG("[SD Card] Failed to get SD card capacity.\n");
        usleep(CHECKSD_TRIGGERING_TIME);
    }
    uint64_t total = (uint64_t)stat.f_blocks * (uint64_t)stat.f_frsize;
    uint64_t free_space = (uint64_t)stat.f_bfree * (uint64_t)stat.f_frsize;
    uint64_t used = total - free_space;

    float usage_percent = ((float)used / (float)total) * 100.0f;

    
    // LOG("SD Card total:%d\n", total);
    // LOG("SD Card free_space:%d\n", free_space);
    // LOG("SD Card used:%d\n", used);
    // LOG("SD Card usage_percent:%.2f%%\n", usage_percent);

    if (usage_percent >= SDMAXCAPACITY)
    {
        Func_DeleteOldestFolder();
    }
    usleep(1000*1000);
}
