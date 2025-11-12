#include "bmu_drv.h"
#include "interface/epoll/myepoll.h"


static pthread_mutex_t Mutex_Can1Send;

static int BMU_CAN_FD;

queue_t Queue_BMURevData; // 分机消息队列，用于epoll接收数据存入，防止处理不过来所以用队列，内部使用

static void bmu_can_epoll_msg_transmit(void *arg)
{
    struct can_frame can_rev;
    CAN_MESSAGE bmu_message;

    memset(&can_rev, 0, sizeof(struct can_frame));
    memset(&bmu_message, 0, sizeof(CAN_MESSAGE));
    if (HAL_can_read(BMU_CAN_FD, &can_rev, 1) > 0) //   后期改小这个参数
    {
        Convert_can_frame_to_CAN_MESSAGE(&can_rev, &bmu_message);

        if (queue_post(&Queue_BMURevData, (unsigned char *)&bmu_message, sizeof(CAN_MESSAGE)) != 0)
        {
            queue_destroy(&Queue_BMURevData);
            queue_init(&Queue_BMURevData);
        }
    }
}

// 初始化
bool BMU_Init(void)
{
    struct epoll_event ev;
    my_event_data_t bmuCanEventData = {
    .fd = -1,  // 无效的文件描述符
    .pin = -1, // 无效的引脚
    .fun_handle = NULL,
    .call_back = NULL
    };

    pthread_mutex_init(&Mutex_Can1Send, NULL);
    queue_init(&Queue_BMURevData); // 用于接收消息后存入

    if (can_ifconfig_init(BMU_CAN_DEVICE_NAME, BMU_CAN_BITRATE) == false)
    {
        LOG("[BMU] %s can_ifconfig_init 失败\n", BMU_CAN_DEVICE_NAME);
        return false;
    }

    while (can_band_init(BMU_CAN_DEVICE_NAME, &BMU_CAN_FD) == false)
    {
        LOG("[BMU]%s can_band_init 失败，重试中...\n", BMU_CAN_DEVICE_NAME);
        sleep(1);
    }

    bmuCanEventData.fd = BMU_CAN_FD;
    bmuCanEventData.fun_handle = (void *)bmu_can_epoll_msg_transmit;
    ev.events = EPOLLIN;
    ev.data.ptr = (void *)&bmuCanEventData;
    if (-1 == my_epoll_addtast(bmuCanEventData.fd, &ev))
    {
        LOG("[BMU]%s add epoll failed \n", BMU_CAN_DEVICE_NAME);
        return false;
    }

    return true;
}

// 发送can报文
int Drv_bmu_can_send(CAN_MESSAGE *pFrame)
{
    struct can_frame can_frame;
    int retryCount = 0;
    const int maxRetries = 3;
    Convert_CAN_MESSAGE_to_can_frame(pFrame, &can_frame);
    while (retryCount < maxRetries)
    {
        if (HAL_can_write(BMU_CAN_FD, &can_frame))
        {
            return 0;
        }

        retryCount++;
    }

    return -1;
}

// 发送canfd报文
int Drv_bmu_canfd_send(struct canfd_frame *cansend_data)
{
    int retryCount = 0;
    const int maxRetries = 3;

    while (retryCount < maxRetries)
    {
        if (HAL_canfd_write(BMU_CAN_FD, &cansend_data))
        {
            return 0;
        }

        retryCount++;
        usleep(100);
    }

    return -1;
}
