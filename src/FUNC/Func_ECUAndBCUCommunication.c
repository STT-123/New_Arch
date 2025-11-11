#define _GNU_SOURCE

#include "Func_ECUAndBCUCommunication.h"
#include "./new/2_device/bcu_drv/bcu_drv.h"
#include "log/log.h"
#include "./BMS/C_BMSAnalysis.h"
#include "./DRV/Drv_SDCard.h"
#include "Func_SDCard.h"
#include "main.h"
#include "./BMS/bms/CANRcvFcn.h"
#include "./GLB/G_AddressDefinition.h"
void Func_print_canfd_frame(const struct canfd_frame *frame)
{
    printf("CAN FD Frame:\n");
    printf("  CAN ID   : 0x%08X\n", frame->can_id & CAN_EFF_MASK);
    printf("  DLC      : %d\n", frame->len);
    printf("  Data     : ");
    for (int i = 0; i < frame->len; i++)
    {
        printf("%02X ", frame->data[i]);
    }
    printf("\n");
}

void *Func_thread_can0_dealwith(void *arg)
{
    struct canfd_frame canrev_frame = {0};
    CANFD_MESSAGE can_msg_buf ={0};
    LOG("Func_thread_can0_dealwith is running\n");
    int len = 0;
    int bms_analysis_done = 0;
    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time); // 记录线程开始时间
    while (1)
    {


        if ((g_ota_flag == OTAIDLE || g_ota_flag == OTAFAILED || otactrl.deviceType == AC))
        {
            if (queue_pend(&Queue_BCURevData_FD, (unsigned char *)&canrev_frame, &len) == 0)
            {
                if ((canrev_frame.len > 8))
                {
                    canrev_frame.can_id &= CAN_EFF_MASK;
                    ConvertCANFDToBus(&canrev_frame, &CANFDRcvMsg);
                    Drv_BMS_Analysis(0); // BMS数据解析
                    ConvertCANFDToBus(&canrev_frame, &can_msg_buf);
                    Drv_write_to_active_buffer(&can_msg_buf, 1);
                }

                memset(&canrev_frame, 0, sizeof(canrev_frame));
            }
            else
            {
                // 仅在初始化后10秒执行一次
                if (!bms_analysis_done)
                {
                    struct timespec now;
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    long elapsed_ms = (now.tv_sec - start_time.tv_sec) * 1000 +
                                      (now.tv_nsec - start_time.tv_nsec) / 1000000;

                    if (elapsed_ms >= 10000) // 10秒后执行
                    {
                        Drv_BMS_Analysis(0);

                        bms_analysis_done = 1;
                        LOG("Drv_BMS_Analysis executed after 10s delay\r\n");
                    }
                }
            }
        }

        usleep(2 * 1000); // 临时添加测试
    }
}


pthread_t CAN0RecDel_TASKHandle;
void CAN0RecvDelTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&CAN0RecDel_TASKHandle, NULL, Func_thread_can0_dealwith, NULL);
        if (ret != 0)
        {
            LOG("Failed to create Func_thread_can0_dealwith thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("Func_thread_can0_dealwith thread created successfully.\r\n");
        }
    } while (ret != 0);
}