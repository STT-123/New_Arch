#define _GNU_SOURCE
#include "modbustcp_task.h"
#include "interface/log/log.h"
#include "interface/modbus/modbus_defines.h"
#include "interface/G_GloabalVariable.h"
#include "interface/setting/ip_setting.h"
#include "interface/BMS/C_BMSAnalysis.h"
#include "device_drv/modbustcp_drv/modbustcp_drv.h"

#include "main.h"

// modbus服务器信息
modbus_t *ctx = NULL;
modbus_mapping_t *g_mb_mapping;
unsigned char modbus_ip[16] = "192.168.1.110";
const uint16_t REGISTERS_START_ADDRESS = 0x3000; // 寄存器起始地址
extern unsigned short g_ota_flag;
uint16_t *modbusBuff = NULL;
pthread_t NetConfig_TASKHandle;


// modbus连接超时标志
static int timeout_flag = 0;
int get_timeout_flag()
{
    return timeout_flag;
}

// modbus接收数据处理，只处理06的写入操作
static void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length)
{
    int header_length = 0;
    unsigned short data = 0;
    unsigned short address = 0;
  
    header_length = modbus_get_header_length(ctx); // 获取数据长度

    if (query[header_length] == 0x06) // 功能码
    {
        if (req_length < 12){return; }// 长度不够直接退出

        // 获取目标地址和数据
        address = (query[header_length + 1] << 8) | query[header_length + 2];
        data = (query[header_length + 3] << 8) | query[header_length + 4];

        // 判断地址范围
        if ((address >= REGISTERS_START_ADDRESS) && (address < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
        {
            // 开关机操作
            if ((address == 0x6700) && (otactrl.UpDating == 0)) // 过滤，自己需要判断是否在升级来进行自主上下电
            {
                if (data == 0)
                {
                    if(g_ota_flag != OTASTARTRUNNING)
                    {
                        set_TCU_PowerUpCmd(BMS_POWER_ON);
                        printf("1get_TCU_PowerUpCmd(BMS_POWER_ON) = %d\r\n",(int)get_TCU_PowerUpCmd());
                    }
                }
                else if (data == 1)
                {
                    set_TCU_PowerUpCmd(BMS_POWER_OFF);
                    printf("2get_TCU_PowerUpCmd(BMS_POWER_ON) = %d\r\n",(int)get_TCU_PowerUpCmd());
                }
            }
            // RTC时间设置
            else if (address >= 0x6705 && address <= 0x670A)
            {
                RTC_ModBus_Deal(address, data);
            }
            // 设置ip
            else if (address == 0x6711 || address == 0x6712)
            {
                G_ip_set_deal(address, data);
            }
            // 重启
            else if ((address == 0x6720) && (data == 1))
            {
                set_ems_bms_reboot();
            }
            else if ((address == 0x6718))//节能模式使能控制
            {
                if (data == 0)
                {
                    set_modbus_reg_val(0x3418, 0);
                    set_TCU_ECOMode(0);
                }
                else if (data == 1)
                {
                    set_modbus_reg_val(0x3418, 1);
                    set_TCU_ECOMode(1);
                }
            }
            else if ((address == 0x6719) || (address == 0x6734) || (address == 0x6735))
            {
                VoltageCalibration_ModBus_Deal(address, data);
            }
            else if (address == 0x6714)//SOHCmd
            {
                BatteryCalibration_ModBus_Deal(address, data);
            }
            else if (address == 0x6715)//SOCMinCmd,SOCMaxCmd
            {
                BatteryCalibration_ModBus_Deal(address, data);
            }
            else if (address == 0x6719)//bit0：屏蔽故障，支持开关离网,bit1：屏蔽绝缘故障，但是计算绝缘值,bit2：屏蔽绝缘功能，不计算绝缘值
            {
                set_modbus_reg_val(address, data);
                set_TCU_FcnStopSet(data);
            }
            else if (address == 0x6721)//SD卡格式化
            {
                printf("SDCardDataSaveTaskCreate\r\n");
                set_modbus_reg_val(address, data);
            }
        }
    }
}

static int set_ip_address(const char *if_name, const char *ip_addr)
{
    int fd;
    struct ifreq ifr;
    struct sockaddr_in sin;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        perror("socket");
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);

    memset(&sin, 0, sizeof(struct sockaddr));
    sin.sin_family = AF_INET;
    inet_pton(AF_INET, ip_addr, &sin.sin_addr);
    memcpy(&ifr.ifr_addr, &sin, sizeof(struct sockaddr));

    if (ioctl(fd, SIOCSIFADDR, &ifr) < 0)
    {
        perror("SIOCSIFADDR");
        close(fd);
        return -1;
    }

    // 设置网口状态为 up
    if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0)
    {
        perror("SIOCGIFFLAGS");
        close(fd);
        return -1;
    }

    ifr.ifr_flags |= IFF_UP | IFF_RUNNING;

    if (ioctl(fd, SIOCSIFFLAGS, &ifr) < 0)
    {
        perror("SIOCSIFFLAGS");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

void *ModbusTCPServerTask(void *arg)
{
    int rc;
    unsigned int i;
    fd_set refset;  //所有监听的socket）
    fd_set rdset;   //select检查的socket
    int fdmax;      // 最大文件描述符值
    int server_socket;  // 服务器监听socket
    int master_socket;  // 客户端连接socket

    if (g_ipsetting.flag == 1 && g_ipsetting.ip != 0)// 创建modbus服务端
    {
        sprintf(modbus_ip, "%d.%d.%d.%d", (g_ipsetting.ip >> 24) & 0xFF, (g_ipsetting.ip >> 16) & 0xFF, (g_ipsetting.ip >> 8) & 0xFF, g_ipsetting.ip & 0xFF);
    }
    set_ip_address("eth1", modbus_ip);//设置当前机的网口的ip地址
    ctx = modbus_new_tcp(modbus_ip, 502);//新建一个tcp服务端
    LOG("ctx =%d ,modbus ip =%s \r\n", ctx, modbus_ip);

    // 创建寄存器映射，只创建保持寄存器
    g_mb_mapping = modbus_mapping_new_start_address(0, 0, 0, 0, REGISTERS_START_ADDRESS, REGISTERS_NB, 0, 0);
    if (g_mb_mapping == NULL){
        LOG("Failed to allocate the mapping: %s \r\n", modbus_strerror(errno));
        modbus_free(ctx);
        for (;;){
            usleep(5000);
        }
    }

    modbusBuff = g_mb_mapping->tab_registers;// 全局 外部在用

    for (i = 0; i < 2; i++)// 填充部分信息
    {
        g_mb_mapping->tab_registers[i] = 10;
    }
    g_mb_mapping->tab_registers[MDBUS_ADDR_PRODUCTION - REGISTERS_START_ADDRESS] = LOGO;       // 智充
    g_mb_mapping->tab_registers[MDBUS_ADDR_ECU_VERSION - REGISTERS_START_ADDRESS] = ECU_VERSION; // 版本号

    server_socket = modbus_tcp_listen(ctx, NB_CONNECTION);// 开启监听
    FD_ZERO(&refset); //初始化集合为NULL
    FD_SET(server_socket, &refset); // 将服务器socket加入集合
    fdmax = server_socket;
    struct timeval timeout;

    for (;;)
    {
        timeout.tv_sec = 10; // 设置select超时时间为10秒
        timeout.tv_usec = 0;

        rdset = refset;// 复制参考集合到读集合    
        if (select(fdmax + 1, &rdset, NULL, NULL, &timeout) == 0)//阻塞10s
        {      
            timeout_flag = 1;// select超时处理
             // 清理所有客户端连接
            for (master_socket = 0; master_socket <= fdmax; master_socket++){
                if (FD_ISSET(master_socket, &refset) && master_socket != server_socket){ //判断当前fd是否为refset中的集合
                    LOG("Closing connection on socket %d\n", master_socket);
                    close(master_socket); // 关闭服务器套接字
                    FD_CLR(master_socket, &refset);// 从集合中移除
                    if (master_socket == fdmax){ fdmax--;}  // 更新最大文件描述符
                }
            }
        }else{
            timeout_flag = 0;
        }

        // 遍历所有可能的socket
        for (master_socket = 0; master_socket <= fdmax; master_socket++)
        {
            if (FD_ISSET(master_socket, &rdset))// 检查socket是否就绪
            {
                if (master_socket == server_socket) // 服务器进入就绪状态，有客户端要连接
                {      
                    socklen_t addrlen;// 处理新连接请求
                    struct sockaddr_in clientaddr;
                    int newfd;

                    addrlen = sizeof(clientaddr);
                    memset(&clientaddr, 0, sizeof(clientaddr));
                    // 接受新连接
                    newfd = accept(server_socket, (struct sockaddr *)&clientaddr, &addrlen); //任意客户端连接出错，客户端都关了
                    if (newfd == -1)
                    {
                        LOG("Server accept() error: %s\n", strerror(errno));  // 出错时清理所有连接
                        for (master_socket = 0; master_socket <= fdmax; master_socket++)
                        {
                            if (FD_ISSET(master_socket, &refset) && master_socket != server_socket)
                            {
                                // LOG("Closing connection on socket %d\n", master_socket);
                                close(master_socket); // 关闭服务器套接字
                                FD_CLR(master_socket, &refset);
                                if (master_socket == fdmax)
                                {
                                    fdmax--;
                                }
                            }
                        }
                        usleep(1000);
                    }
                    else
                    {                     
                        FD_SET(newfd, &refset); // 成功接受连接,添加到监控集合
                        if (newfd > fdmax)  // 更新最大fd
                        {
                            fdmax = newfd;// 更新最大fd
                        }
                        LOG("New connection from %s:%d on socket %d \r\n", inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
                    }
                }
                else
                {
                    // 处理已连接客户端的请求
                    unsigned char query[MODBUS_TCP_MAX_ADU_LENGTH];
                    modbus_set_socket(ctx, master_socket); // 设置当前socket到modbus上下文
                    
                    rc = modbus_receive(ctx, query);// 接收Modbus请求
                    if (rc != -1)
                    {
                        modbus_write_reg_deal(ctx, query, rc); // 写寄存器处理
                        modbus_reply(ctx, query, rc, g_mb_mapping); // 回复寄存器
                    }
                    else
                    {
                        LOG("Connection closed or error on socket %d: %s\n", master_socket, modbus_strerror(errno));                       
                        close(master_socket); // 关闭连接
                        FD_CLR(master_socket, &refset); // 从集合移除

                        // 若该 socket 是当前最大值，更新 fdmax
                        if (master_socket == fdmax)
                        {
                            while (fdmax > 0 && !FD_ISSET(fdmax, &refset))
                            {
                                fdmax--;
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * reamdme:
 * 本C文件包含ModusTCP连接人物和OTA文件升级处理任务
 * ModbusTCPServer：新建ModusTCP连接，监听读取寄存器和写寄存器的功能，要是ota则由TcpServerExample处理
 * TcpServerExample：OTA文件升级处理任务
*/
void modbusTcpServerTaskCreate(void)
{
    int ret;
    do
    {
        ret = pthread_create(&NetConfig_TASKHandle, NULL, ModbusTCPServerTask, NULL);
        if (ret != 0){
            LOG("Failed to create NETConfigTask thread : %s", strerror(ret));
            sleep(1);
        }else{
            LOG("NETConfigTask thread created successfully.\r\n");
        }
    } while (ret != 0);
}