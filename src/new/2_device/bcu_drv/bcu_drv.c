#include "bcu_drv.h"
#include "log/log.h"
#include "./new/1_interface/epoll/myepoll.h"
#include "./DRV/Drv_ExternalEmergencyStop.h"


static pthread_mutex_t Mutex_BCUCanSend;
static int BCU_CAN_FD;
queue_t Queue_BCURevData;
queue_t Queue_BCURevData_FD;

/*======================================静态函数==================================================*/


static int Drv_bcu_resetcan_device(const char *can_name)
{
    int canState = 0;
    HAL_can_get_state(can_name, &canState);
    if (canState != 0) {
        LOG("%s status is %02X\r\n", can_name, canState);
        HAL_can_closeEx(&BCU_CAN_FD);
        if (can_ifconfig_init(BCU_CAN_DEVICE_NAME, BCU_CAN_BITRATE) == false){
            LOG("can_ifconfig_init 失败\n");
            return false;
        }

        while (can_band_init(BCU_CAN_DEVICE_NAME, &BCU_CAN_FD) == false) {
            sleep(1);
        }
    }
    return 0;
}

static void bcu_can_epoll_msg_transmit(void *arg)
{
    struct canfd_frame can_rev ;
    CAN_MESSAGE can_send ;

    memset(&can_rev, 0, sizeof(struct canfd_frame));
    memset(&can_send, 0, sizeof(CAN_MESSAGE));
    int frame_type = HAL_canfd_read(BCU_CAN_FD, &can_rev, 1);

    time(&g_last_bmu_rx_time);
    
    if (frame_type == 1)//1表示CAN 数据-8
    {
        Convert_canfd_frame_to_CAN_MESSAGE(&can_rev, &can_send);

        if (((get_TCU_PowerUpCmd()) == BMS_POWER_UPDATING) && (can_send.ID == 0x30C1600 || can_send.ID == 0x61B || can_send.ID == 0x1CB010E4))
        {
            if (queue_post(&Queue_BCURevData, &can_send, sizeof(CAN_MESSAGE)) != 0)
            {   
                queue_destroy(&Queue_BCURevData);
                queue_init(&Queue_BCURevData);
            }
            else{
            }
        }
        else if ((get_TCU_PowerUpCmd()) != BMS_POWER_UPDATING && can_send.ID != 0x18FFC13A && can_send.ID != 0x18FFC13B && can_send.ID != 0x18FFC13C && can_send.ID != 0x18FAE6E1 && can_send.ID != 0x18FD7BE1 && can_send.ID != 0X18FA78F1 && can_send.ID != 0x18FFC13D && can_send.ID != 0x18FA78F5 && can_send.ID != 0x18FAE6E2)
        {
            if (queue_post(&Queue_BCURevData, &can_send, sizeof(CAN_MESSAGE)) != 0)
            {             
                queue_destroy(&Queue_BCURevData);
                queue_init(&Queue_BCURevData);
            }
            else{
            }
        }
    }
    else if (frame_type == 2)//2表示CAN FD数据-64
    {
        //往can fd队列方数据，但是数据满了,在升级过程在，不再从canfd队列取数据了，此时fd满了
        if (queue_post(&Queue_BCURevData_FD, (unsigned char *)&can_rev, sizeof(can_rev)) != 0)
        {
            queue_destroy(&Queue_BCURevData_FD);
            queue_init(&Queue_BCURevData_FD);
        }
        else{
        }
    }
    else
    {
        printf("[bcu_can_epoll_msg_transmit] Received CAN frame with ID: %d, Length: %d\n", can_rev.can_id, can_rev.len);
    }
}


/*======================================外部函数==================================================*/
bool BCU_Init(void)
{
    struct epoll_event ev;

    my_event_data_t bcuCanEventData = {
    .fd = -1,  // 无效的文件描述符
    .pin = -1, // 无效的引脚
    .fun_handle = NULL,
    .call_back = NULL
    };

    pthread_mutex_init(&Mutex_BCUCanSend, NULL);
    queue_init(&Queue_BCURevData);    // 用于接收消息后存入
    queue_init(&Queue_BCURevData_FD); // 用于接收消息后存入

    if (can_ifconfig_init(BCU_CAN_DEVICE_NAME, BCU_CAN_BITRATE) == false)
    {
        LOG("%s can_ifconfig_init 失败\n", BCU_CAN_DEVICE_NAME);
        return false;
    }

    while (can_band_init(BCU_CAN_DEVICE_NAME, &BCU_CAN_FD) == false)
    {
        LOG("%s can_band_init 失败，重试中...\n", BCU_CAN_DEVICE_NAME);
        sleep(1);
    }

    bcuCanEventData.fd = BCU_CAN_FD;
    bcuCanEventData.fun_handle = (void *)bcu_can_epoll_msg_transmit; // 回调函数
    ev.events = EPOLLIN;
    ev.data.ptr = (void *)&bcuCanEventData;
    if (-1 == my_epoll_addtast(bcuCanEventData.fd, &ev))
    {
        LOG("%s add epoll failed \n", BCU_CAN_DEVICE_NAME);
        return false;
    }

    return true;
}


int Drv_bcu_can_send(CAN_MESSAGE *pFrame)
{
    struct can_frame can_frame;
    int retryCount = 0;
    const int maxRetries = 3;
    //  ConvertBusToCANFD(pFrame,&canfd_frame);
    Convert_CAN_MESSAGE_to_can_frame(pFrame, &can_frame);
    while (retryCount < maxRetries)
    {
        if (HAL_can_write(BCU_CAN_FD, &can_frame))
        {
            return 0;
        }

        retryCount++;
    }
    if (retryCount == 3)
    {
        Drv_bcu_resetcan_device(BCU_CAN_DEVICE_NAME);
    }

    return -1;
}

int Drv_bcu_canfd_send(CAN_FD_MESSAGE_BUS *pFrame)
{
    struct canfd_frame canfd_frame;
    int retryCount = 0;
    const int maxRetries = 3;
    Drv_write_to_active_buffer(&pFrame, 0); // 0709添加
    ConvertBusToCANFD(pFrame, &canfd_frame);
    while (retryCount < maxRetries)
    {
        if (HAL_canfd_write(BCU_CAN_FD, &canfd_frame))
        {
            return 0;
        }
        retryCount++;
    }
    return -1;
}

// 消息解析
bool Drv_BMS_Analysis(unsigned char can_flag)
{
    bool state = false;
    state = BMSAnalysis(can_flag);
    return state;
}
