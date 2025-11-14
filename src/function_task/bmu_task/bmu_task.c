#define _GNU_SOURCE
#include "bmu_task.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "interface/log/log.h"
#include "main.h"
#include "interface/bms/bms_simulink/CANRcvFcn.h"
#include "function_task/sd_task/sd_task.h"


void *bmu_DealTask(void *arg)
{
    // struct can_frame canrev_frame;
    CAN_MESSAGE canrev_frame;
    LOG("Func_thread_can1_dealwith is running\n");
    int err = 0;
    while (1)
    {
        if ((g_ota_flag == OTAIDLE || g_ota_flag == OTAFAILED || otactrl.deviceType == AC))
        {

            // 等待信号，有信号则有消息来，处理以后加进消息接收中
            if (queue_pend(&Queue_BMURevData, (unsigned char *)&canrev_frame, &err) == 0)
            {
                if ((g_ota_flag == OTAIDLE || g_ota_flag == OTAFAILED))
                {
                    memcpy(&CANRcvMsg, &canrev_frame, sizeof(CAN_MESSAGE));
                    Drv_BMS_Analysis(1); // BMS数据解析 临时取消
                    memset(&canrev_frame, 0, sizeof(canrev_frame));
                }
            }
        }

        // printf("queue_pend return err = %d\r\n", err);
        usleep(10 * 1000);
    }
}


pthread_t BMURecvDel_TASKHandle;
void bmu_DealTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&BMURecvDel_TASKHandle, NULL, bmu_DealTask, NULL);
        if (ret != 0)
        {
            LOG("Failed to create BMU_DealTask thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("BMU_DealTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}