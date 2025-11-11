#include <time.h>
#include <features.h>
// #include "main.h"
#include "interface/log/log.h"
// #include "interface/modbus"
#include "interface/BMS/C_BMSAnalysis.h"
#include "device_drv/abncheck_drv/abncheck_drv.h"
#include "device_drv/bcu_drv/bcu_drv.h"
#include "device_drv/bmu_drv/bmu_drv.h"
#include "device_drv/modbustcp_drv/modbustcp_drv.h"
#include "interface/G_GloabalVariable.h"
#include "interface/BMS/bms/CANRcvFcn.h"


#define _POSIX_C_SOURCE 199309L
#define RECOVER_REPORT_TIME 5000
#define FAULT_REPORT_TIME 3000
#define RECOVER_KM_DEFAULT_STATE 0 // 恢复接触器默认状态
#define RECOVER_KM_ACTION_STATE 1  // 恢复接触器动作状态
struct timespec lastCheckTick;
int PHY_RECOVER = 0;
int PHY_ERROR = 0;
uint8_t ECUState = 0;
int RECOVER = 0;
int ERROR = 0;
struct timespec lasttimes;

time_t g_last_bmu_rx_time = 0;

static int g_can2_ready = 0;
static int g_can3_ready = 0;

void log_eror_csv(void)
{
	unsigned char log_flag = 0;
	static unsigned int BCU_FaultInfoLv1H_LAST = 0;
	static unsigned int BCU_FaultInfoLv1L_LAST = 0;
	static unsigned int BCU_FaultInfoLv2H_LAST = 0;
	static unsigned int BCU_FaultInfoLv2L_LAST = 0;
	static unsigned int BCU_FaultInfoLv3H_LAST = 0;
	static unsigned int BCU_FaultInfoLv3L_LAST = 0;
	static unsigned int BCU_FaultInfoLv4H_LAST = 0;
	static unsigned int BCU_FaultInfoLv4L_LAST = 0;
	static unsigned short BCU_SystemWorkMode_LAST = 0;

	if (ecu_fault_last.emcu_fault0 != ecu_fault.emcu_fault0)
	{
		log_flag = 1;
		ecu_fault_last.emcu_fault0 = ecu_fault.emcu_fault0;
	}
	if (ecu_fault_last.emcu_fault1 != ecu_fault.emcu_fault1)
	{
		log_flag = 1;
		ecu_fault_last.emcu_fault1 = ecu_fault.emcu_fault1;
	}
	if (ecu_fault_last.emcu_fault2 != ecu_fault.emcu_fault2)
	{
		log_flag = 1;
		ecu_fault_last.emcu_fault2 = ecu_fault.emcu_fault2;
	}

	if (ecu_fault_last.emcu_fault_state != ecu_fault.emcu_fault_state)
	{
		log_flag = 1;
		ecu_fault_last.emcu_fault_state = ecu_fault.emcu_fault_state;
	}
	if (BCU_SystemWorkMode_LAST != BCU_SystemWorkMode)
	{
		log_flag = 1;
		BCU_SystemWorkMode_LAST = BCU_SystemWorkMode;
	}

	if (BCU_FaultInfoLv1H_LAST != BCU_FaultInfoLv1H)
	{
		log_flag = 1;
		BCU_FaultInfoLv1H_LAST = BCU_FaultInfoLv1H;
	}
	if (BCU_FaultInfoLv1L_LAST != BCU_FaultInfoLv1L)
	{
		log_flag = 1;
		BCU_FaultInfoLv1L_LAST = BCU_FaultInfoLv1L;
	}
	if (BCU_FaultInfoLv2H_LAST != BCU_FaultInfoLv2H)
	{
		log_flag = 1;
		BCU_FaultInfoLv2H_LAST = BCU_FaultInfoLv2H;
	}
	if (BCU_FaultInfoLv2L_LAST != BCU_FaultInfoLv2L)
	{
		log_flag = 1;
		BCU_FaultInfoLv2L_LAST = BCU_FaultInfoLv2L;
	}
	if (BCU_FaultInfoLv3H_LAST != BCU_FaultInfoLv3H)
	{
		log_flag = 1;
		BCU_FaultInfoLv3H_LAST = BCU_FaultInfoLv3H;
	}
	if (BCU_FaultInfoLv3L_LAST != BCU_FaultInfoLv3L)
	{
		log_flag = 1;
		BCU_FaultInfoLv3L_LAST = BCU_FaultInfoLv3L;
	}
	if (BCU_FaultInfoLv4H_LAST != BCU_FaultInfoLv4H)
	{
		log_flag = 1;
		BCU_FaultInfoLv4H_LAST = BCU_FaultInfoLv4H;
	}
	if (BCU_FaultInfoLv4L_LAST != BCU_FaultInfoLv4L)
	{
		log_flag = 1;
		BCU_FaultInfoLv4L_LAST = BCU_FaultInfoLv4L;
	}

	if (log_flag == 1)
	{
		LOG_CSV("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			ecu_fault.emcu_fault0,
			ecu_fault.emcu_fault1,
			ecu_fault.emcu_fault2,
			BCU_FaultInfoLv1H,
			BCU_FaultInfoLv1L,
			BCU_FaultInfoLv2H,
			BCU_FaultInfoLv2L,
			BCU_FaultInfoLv3H,
			BCU_FaultInfoLv3L,
			BCU_FaultInfoLv4H,
			BCU_FaultInfoLv4L,
			BCU_SOC,
			BCU_SystemStatus,
			BCU_SystemWorkMode);
	}
}

typedef struct
{
	unsigned long long rx_bytes;
	unsigned long long tx_bytes;
	unsigned long long rx_packets;
	unsigned long long tx_packets;
} net_stats_t;

int CheckSinglePHYStatus(const char *ifname)
{
	FILE *fp;
	char line[512];
	char devname[32];
	net_stats_t stats = {0};
	static net_stats_t last_stats[2] = {0}; // 简单缓存上次统计
	static int initialized = 0;

	fp = fopen("/proc/net/dev", "r");
	if (!fp)
	{
		perror("fopen /proc/net/dev failed");
		return 0; // 没有
	}

	// 跳过前两行标题
	fgets(line, sizeof(line), fp);
	fgets(line, sizeof(line), fp);

	while (fgets(line, sizeof(line), fp))
	{
		// 解析接口统计信息
		if (sscanf(line, "%31[^:]: %llu %llu %*u %*u %*u %*u %*u %*u %llu %llu",
				   devname, &stats.rx_bytes, &stats.rx_packets,
				   &stats.tx_bytes, &stats.tx_packets) >= 4)
		{

			// 去除接口名末尾的空格
			char *p = devname;
			while (*p == ' ')
				p++;

			if (strcmp(p, ifname) == 0)
			{
				fclose(fp);

				// 检查是否有数据活动
				int has_traffic = 0;

				if (initialized)
				{
					// 比较当前统计和上次统计
					net_stats_t *last = &last_stats[strcmp(ifname, "eth0") == 0 ? 0 : 1];
					if (stats.rx_bytes > last->rx_bytes ||
						stats.tx_bytes > last->tx_bytes ||
						stats.rx_packets > last->rx_packets ||
						stats.tx_packets > last->tx_packets)
					{
						has_traffic = 1;
					}
				}

				// 更新缓存
				net_stats_t *last = &last_stats[strcmp(ifname, "eth0") == 0 ? 0 : 1];
				*last = stats;
				initialized = 1;

				return has_traffic;
			}
		}
	}

	fclose(fp);
	return 0; // 接口未找到
}

void PHYlinktate()
{
	static struct timespec lastCheckTick = {0};
	static int PHY_RECOVER_FLAG = 0;
	static int PHY_ERROR_FLAG = 0;

	int eth0_status = 0; // 1表示连接，0表示未连接
	int eth1_status = 0;

	// 检查eth0
	eth0_status = CheckSinglePHYStatus("eth0");
	// 检查eth1
	eth1_status = CheckSinglePHYStatus("eth1");//检测网线是否物理连接上

	// LOG("TTTTTTTTTTTTTTTTT eth0_status = %d,  eth1_status=%d ", eth0_status, eth1_status);
	// eth0_status = 1;
	// eth1_status = 1;

	if (eth0_status && eth1_status) // 两个都连接
	{
		if (PHY_RECOVER_FLAG == 1)
		{
			if (GetTimeDifference_ms(lastCheckTick) >= RECOVER_REPORT_TIME)
			{
				set_emcu_fault(PHY_LINK_FAULT, SET_RECOVER);
				PHY_ERROR_FLAG = 0;
			}
		}
		else
		{
			clock_gettime(CLOCK_MONOTONIC, &lastCheckTick);
			PHY_RECOVER_FLAG = 1;
			PHY_ERROR_FLAG = 0;
		}
	}
	else // 任意一个未连接就报故障
	{
		if (PHY_ERROR_FLAG == 1)
		{
			if (GetTimeDifference_ms(lastCheckTick) >= FAULT_REPORT_TIME)
			{
				set_emcu_fault(PHY_LINK_FAULT, SET_ERROR);
				PHY_RECOVER_FLAG = 0;
			}
		}
		else
		{
			clock_gettime(CLOCK_MONOTONIC, &lastCheckTick);
			PHY_ERROR_FLAG = 1;
			PHY_RECOVER_FLAG = 0;
		}
	}
}

void ECUfault_process()
{
	update_fault_tomodus();
}

/**
 * 检测BCU的通信是否超时函数
*/
void check_bcu_rx_timeout(void)
{
	static bool can0_fault_reported = false;
	time_t current_time;
	time(&current_time);
	double diff = difftime(current_time, g_last_bmu_rx_time);

	if (diff >= 5.0)
	{
		if (!can0_fault_reported)
		{
			set_emcu_fault(BMS_COM_FAULT, SET_ERROR);
			can0_fault_reported = true;
			printf("CAN0 TimeoutCheck warning\n");
		}
	}
	else
	{
		if (can0_fault_reported)
		{
			set_emcu_fault(BMS_COM_FAULT, SET_RECOVER);
			can0_fault_reported = false;
			printf("CAN0 TimeoutCheckv normal\n");
		}
	}
}

/**
 * 检测CAN 是否异常函数
*/
int can_monitor_fun(void) {

    // 检查 can2
	int can2_state = check_can_state(BCU_CAN_DEVICE_NAME);
	if (can2_state == 0 && g_can2_ready == 1) {
		LOG("can2 abnormal, restarting...\n");
		restart_can_interface(BCU_CAN_DEVICE_NAME);
	}
	g_can2_ready = can2_state;

	// 检查 can3
	int can3_state = check_can_state(BMU_CAN_DEVICE_NAME);
	if (can3_state == 0 && g_can3_ready == 1) {
		LOG("can3 abnormal, restarting...\n");
		restart_can_interface("can3");
	}
	g_can3_ready = can3_state;

}

// 重启CAN接口
static void restart_can_interface(const char* can_if) {
    LOG("Restarting %s...\n", can_if);
    char cmd[128];
    
    snprintf(cmd, sizeof(cmd), "sudo /bin/ip link set %s down", can_if);
    system(cmd);
    usleep(100000);
    
    snprintf(cmd, sizeof(cmd), "sudo /bin/ip link set %s up", can_if);
    system(cmd);
    usleep(500000);
}

// 检查单个CAN接口状态
static int check_can_state(const char* can_if) {
    char command[128];
    snprintf(command, sizeof(command), 
             "/bin/ip link show %s 2>/dev/null | grep -q 'LOWER_UP'", can_if);
    return (system(command) == 0);
}

// 主业务判断函数
int is_can2_ready(void) {
    return g_can2_ready;
}
int is_can3_ready(void) {
    return g_can3_ready;
}