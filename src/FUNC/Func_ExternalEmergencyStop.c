#include "Func_ExternalEmergencyStop.h"
#include "./DRV/Drv_ExternalEmergencyStop.h"
#include "./BMS/C_BMSAnalysis.h"
#include "./BMS/bms/CANRcvFcn.h"
#include <pthread.h>
#include "log/log.h"

void *AnormalDetection(void *arg)
{
    /**
     * 目前检测的故障只有、CAN0通道、SD卡
    */
    while (1)
    {
        /* code */
        check_bmu_rx_timeout();//CAN0 通道检测
        //PHYlinktate(); //网口物理连接检测
        ECUfault_process(); // 各种故障检测
        get_BCU_FaultInfo(BCU_FaultInfoLv4H,BCU_FaultInfoLv3H,BCU_FaultInfoLv2H);
        can_monitor_fun();
        log_eror_csv();  //存储
        usleep(1000 * 1000);
    }
}

pthread_t AnormalDetectionTask_TASKHandle;
void AnormalDetectionTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&AnormalDetectionTask_TASKHandle, NULL, AnormalDetection, NULL);
        if (ret != 0)
        {
            LOG("Failed to create AnormalDetectionTask thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("AnormaletectionTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}