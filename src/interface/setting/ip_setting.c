#include "ip_setting.h"
#include "interface/G_GloabalVariable.h"
#include "interface/log/log.h"
#include "./BMS/bms/CANRcvFcn.h"
#define CONFIG_FILE_PATH "setting.conf"
Setting_t g_ipsetting;

void static save_setting_to_file(const char *filepath, Setting_t *ipsetting)
{
    FILE *fp = fopen(filepath, "wb");
    if (fp)
    {
        fwrite(ipsetting, sizeof(Setting_t), 1, fp);
        fclose(fp);
    }
    else
    {
        LOG("Failed to write ipsetting\r\n");
    }
}

int static load_setting_from_file(const char *filepath, Setting_t *ipsetting)
{
    FILE *fp = fopen(filepath, "rb");
    if (fp)
    {
        fread(ipsetting, sizeof(Setting_t), 1, fp);
        fclose(fp);
        return 0;
    }
    return -1;
}

void G_settings_init()
{
    memset(&g_ipsetting, 0, sizeof(g_ipsetting));

    struct stat st;
    int config_exist = (stat(CONFIG_FILE_PATH, &st) == 0);

    if (config_exist && load_setting_from_file(CONFIG_FILE_PATH, &g_ipsetting) == 0 && g_ipsetting.flag == 1)
    {
        struct in_addr addr;
        addr.s_addr = htonl(g_ipsetting.ip);
        LOG("Read IP from config: %s\n", inet_ntoa(addr));
    }
    else
    {
        // 配置文件不存在或数据无效，写入默认值
        g_ipsetting.flag = 1;
        g_ipsetting.ip = (192 << 24) | (168 << 16) | (1 << 8) | 110;

        save_setting_to_file(CONFIG_FILE_PATH, &g_ipsetting);

        struct in_addr addr;
        addr.s_addr = htonl(g_ipsetting.ip);
        LOG("Default IP used and saved: %s\n", inet_ntoa(addr));
    }
}

void G_ip_set_deal(uint16_t address, uint16_t data)
{
    static uint32_t ip_addr = 0;
    if (address == 0x6711) // 高 16 位
    {
        ip_addr &= 0x0000ffff;
        ip_addr |= ((uint32_t)data) << 16;
        return;
    }
    else if (address == 0x6712) // 低 16 位
    {
        ip_addr &= 0xffff0000;
        ip_addr |= data & 0xFFFF;

        g_ipsetting.ip = ip_addr;
        g_ipsetting.flag = 1;

        // 显示 IP
        struct in_addr ip_struct;
        ip_struct.s_addr = htonl(ip_addr); // 转为网络字节序
        printf("Set IP to: %s\n", inet_ntoa(ip_struct));
        save_setting_to_file(CONFIG_FILE_PATH, &g_ipsetting);
        usleep(500 * 1000);
        system("reboot");
    }
}

void G_set_system_time_from_bcu(void)
{
    struct tm tm;
    struct timeval tv;

    memset(&tm, 0, sizeof(struct tm));

    tm.tm_year = 2000 + get_BCU_TimeYearValue() - 1900; // tm_year 是从1900年开始计数的
    tm.tm_mon = get_BCU_TimeMonthValue() - 1;           // tm_mon 范围是 0-11
    tm.tm_mday = get_BCU_TimeDayValue();
    tm.tm_hour = get_BCU_TimeHourValue();
    tm.tm_min = get_BCU_TimeMinuteValue();
    tm.tm_sec = get_BCU_TimeSencondValue();

    time_t t = mktime(&tm); // 转为 time_t 类型（时间戳）
    if (t == -1)
    {
        printf("Invalid time provided\n");
        return;
    }

    tv.tv_sec = t;
    tv.tv_usec = 0;

    if (settimeofday(&tv, NULL) != 0)
    {
        perror("settimeofday failed");
    }
    else
    {
        // 设置成功
    }
}

int set_ip_address(const char *if_name, const char *ip_addr)
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