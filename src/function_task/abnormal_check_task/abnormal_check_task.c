#include "abnormal_check_task.h"
#include "device_drv/abncheck_drv/abncheck_drv.h"
#include "interface/BMS/C_BMSAnalysis.h"
#include "interface/BMS/bms/CANRcvFcn.h"
#include <pthread.h>
#include "interface/log/log.h"
pthread_t AnormalDetectionTask_TASKHandle;


void *AbnormalDetection(void *arg)
{
    /**
     * 目前检测的故障只有、CAN0通道、SD卡
    */
    while (1)
    {
        /* code */
        check_bcu_rx_timeout();//CAN0 通道检测
        //PHYlinktate(); //网口物理连接检测 
        get_BCU_FaultInfo(BCU_FaultInfoLv4H,BCU_FaultInfoLv3H,BCU_FaultInfoLv2H);

        ECUfault_process(); // 各种故障检测
        can_monitor_fun();
        log_eror_csv();  //存储
        usleep(1000 * 1000);
    }
}


void AbnormalDetectionTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&AnormalDetectionTask_TASKHandle, NULL, AbnormalDetection, NULL);
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