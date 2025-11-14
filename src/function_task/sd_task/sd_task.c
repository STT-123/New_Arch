#define _GNU_SOURCE
#include "sd_task.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/G_GloabalVariable.h"
#include "interface/log/log.h"
#include "main.h"

unsigned short g_ota_flag = 0;

/*==========================================================**/
void *SDCardDataSaveTask(void *arg)
{
    const char *mount_point = USB_MOUNT_POINT; // 使用自动挂载的路径
    uint16_t SD_INIT_flag = 0;

    sd_storeInit();//SD卡存储消息的初始化

    // 1. 检查挂载点
    if (ensure_mount_point(mount_point) != 0)
    {
        LOG("[SD Card] 请检查挂载点路径是否正确.\n");
        return NULL;
    }
    LOG("[SD Card] SD卡挂载 %s 成功. \n", mount_point);

    while (1)
    {
        usleep(3000 * 1000);

        // 获取 ota标识 和 sd卡初始化标识
        get_modbus_reg_val(OTASTATUSREGADDR, &g_ota_flag);
        get_modbus_reg_val(0x6721, &SD_INIT_flag);//接收上位机指令

        // 如果sd卡未初始化 则初始化sd卡
        if (SD_INIT_flag == 1)
        {
            int result = SD_Initialize();
            if (result == 0)
            {
                set_modbus_reg_val(0x6721, 2); // 成功
                LOG("[SD Card] SD_Initialize succeeded.\n");
            }
            else
            {
                set_modbus_reg_val(0x6721, 3); // 失败
                LOG("[SD Card] SD_Initialize error code: %d \n", result);
            }
        }

        if ((g_ota_flag == OTAIDLE || g_ota_flag == OTAFAILED) && SD_INIT_flag != 1)// 没有ota中 并且 sd卡初始化过
        {
            if (get_ftp_read_file_flag() == 0)
            {
                Drv_write_buffer_to_file(); // 将缓冲区内容写入文件
            }
        }
        else
        {
            LOG("[SD Card] ----------------OTAing---------flag = 0x%X-----\r\n", g_ota_flag);
        }
    }
}

pthread_t SDCardDataSave_TASKHandle;
void SDCardDataSaveTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&SDCardDataSave_TASKHandle, NULL, SDCardDataSaveTask, NULL);
        if (ret != 0)
        {
            LOG("[SD Card] Failed to create SDCardDataSaveTaskCreate thread : %s", strerror(ret));
            sleep(1);
        }
        else
        {
            LOG("[SD Card] SDCardDataSaveTaskCreate thread created successfully.\r\n");
        }
    } while (ret != 0);
}