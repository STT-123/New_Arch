#define _GNU_SOURCE
// #include "C_OTAStateMonitor.h"
#include "xmodemstate.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/G_GloabalVariable.h"
#include "xmodemlisten.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "xmodemdata.h"
#include "interface/bms/bms_analysis.h"
#include "interface/log/log.h"
#include "main.h"

struct timespec AC_OTA_lastCheckTick;

unsigned short sblfilenumber = 0xFFFF; // SBL文件数量大小
unsigned short appfilenumber = 0xFFFF; // app文件数量大小
int SBl_index = 0;
int APP_index = 0;

int otasock = -1;
int otasock1 = -1;

static int otafileret = -1; // 初始化为自定义值（-1 表示未初始化）

extern unsigned char XmodemSendCFlag;
extern volatile unsigned long prvmsgtimer;
extern pthread_t *pLwIPTCPListenTaskHandle ;

unsigned char clientConnected;
unsigned char XmodemServerReceiveSOH;
unsigned char XmodemServerReceiveEOT;
unsigned char XmodemServerReceiveFileEnd = 0;
unsigned char XmodemServerEnd;

pthread_mutex_t task_mutex = PTHREAD_MUTEX_INITIALIZER;

FILE *OTAfil = NULL;

unsigned int OsIf_GetMilliseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec * 1000 + now.tv_nsec / 1000000);
}


void CloseXModemServer(void)
{
    shutdown(otasock1, SHUT_RDWR);

    if (otasock1 > 0)
    {
        close(otasock1);
        otasock1 = -1;
    }

    if (otasock > 0)
    {
        close(otasock);
        otasock = -1;
    }

    if (pLwIPTCPDataTaskHandle != NULL)
    {
        // 添加调试输出
        // printf("Before pthread_cancel: pLwIPTCPDataTaskHandle=%p, *pLwIPTCPDataTaskHandle=%lu\n",
        //        (void*)pLwIPTCPDataTaskHandle, (unsigned long)*pLwIPTCPDataTaskHandle);

        pthread_cancel(*pLwIPTCPDataTaskHandle);
        // printf("After pthread_cancel\n");
        // printf("Before pthread_join: pLwIPTCPDataTaskHandle=%p, *pLwIPTCPDataTaskHandle=%lu\n",
        //        (void*)pLwIPTCPDataTaskHandle, (unsigned long)*pLwIPTCPDataTaskHandle);

        pthread_join(*pLwIPTCPDataTaskHandle, NULL); // 等待线程回收资源

        free(pLwIPTCPDataTaskHandle);
        pLwIPTCPDataTaskHandle = NULL;
    }

    if (pLwIPTCPListenTaskHandle != NULL)
    {
        pthread_cancel(*pLwIPTCPListenTaskHandle);
        pthread_join(*pLwIPTCPListenTaskHandle, NULL); // 等待线程回收资源
        free(pLwIPTCPListenTaskHandle);
        pLwIPTCPListenTaskHandle = NULL;
    }

    if (OTAfil != NULL)
    {
        fclose(OTAfil);
        OTAfil = NULL;
    }

    prvmsgtimer = 0;
    curmsgtimer = 0;
    clientConnected = 0;
    XmodemSendCFlag = 0;
    otasock1 = -1;
    otasock = -1;
    XmodemServerReceiveSOH = 0;
    otafileret = 30;
    // XmodemServerEnd = 0;
    setXmodemServerEnd(0);
}

signed char CheckXModemClient(void)
{
    if (clientConnected)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

unsigned char getClientConnected(void)
{
    return clientConnected;
}

void setClientConnected(unsigned char value)
{
    clientConnected = value;
}

unsigned char getXmodemSendCFlag(void)
{
    return XmodemSendCFlag;
}

void setXmodemSendCFlag(unsigned char value)
{
    XmodemSendCFlag = value;
}

unsigned char getXmodemServerReceiveSOH(void)
{
    return XmodemServerReceiveSOH;
}

void setXmodemServerReceiveSOH(unsigned char value)
{
    XmodemServerReceiveSOH = value;
}

unsigned char getXmodemServerReceiveEOT(void)
{
    return XmodemServerReceiveEOT;
}

void setXmodemServerReceiveEOT(unsigned char value)
{
    XmodemServerReceiveEOT = value;
}

unsigned char getXmodemServerReceiveFileEnd(void)
{
    return XmodemServerReceiveFileEnd;
}

void setXmodemServerReceiveFileEnd(unsigned char value)
{
    XmodemServerReceiveFileEnd = value;
}

unsigned char getXmodemServerEnd(void)
{
    return XmodemServerEnd;
}

void setXmodemServerEnd(unsigned char value)
{
    XmodemServerEnd = value;
}
