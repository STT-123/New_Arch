#include "Net_Config_Task.h"
#include "./Modbus/C_ModbusServer.h"
#include "log/log.h"
#include "./Xmodem/C_OTAStateMonitor.h"

pthread_t NetConfig_TASKHandle;
pthread_t TcpServerExample_TASKHandle;
/**
 * reamdme:
 * 本C文件包含ModusTCP连接人物和OTA文件升级处理任务
 * ModbusTCPServer：新建ModusTCP连接，监听读取寄存器和写寄存器的功能，要是ota则由TcpServerExample处理
 * TcpServerExample：OTA文件升级处理任务
*/
void NETConfigTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&NetConfig_TASKHandle, NULL, ModbusTCPServer, NULL);
        if (ret != 0){
            LOG("Failed to create NETConfigTask thread : %s", strerror(ret));
            sleep(1);
        }else{
            LOG("NETConfigTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}

void TcpServerExampleTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&TcpServerExample_TASKHandle, NULL, TcpServerExample, NULL);
        if (ret != 0){
            LOG("Failed to create TcpServerExampleTask thread : %s", strerror(ret));
            sleep(1);
        } else{
            LOG("TcpServerExampleTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}