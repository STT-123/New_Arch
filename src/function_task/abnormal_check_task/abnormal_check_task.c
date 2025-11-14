#include "abnormal_check_task.h"
#include "device_drv/abncheck/abncheck.h"
#include "interface/bms/bms_analysis.h"
#include "interface/bms/bms_simulink/CANRcvFcn.h"
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
        get_BCU_FaultInfo(get_BCU_FaultInfoLv4HValue(),get_BCU_FaultInfoLv3HValue(),get_BCU_FaultInfoLv2HValue());

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
            LOG("[Check] Failed to create AnormalDetectionTask thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("[Check] AnormaletectionTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}