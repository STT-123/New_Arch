#include <time.h>
#include <features.h>
#include "interface/log/log.h"
#include "interface/bms/bms_analysis.h"
#include "device_drv/abncheck/abncheck.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/bmu_deal/bmu_deal.h"
#include "device_drv/modbustcp_pro/modbustcp_pro.h"
#include "interface/globalVariable.h"
#include "interface/bms/bms_simulink/CANRcvFcn.h"


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
ecu_fault_t ecu_fault;
ecu_fault_t ecu_fault_last;
static int g_can2_ready = 0;
static int g_can3_ready = 0;
/*--------------*/
static const fault_mapping_t fault_map_4H[] = {
    {23, EMERGENCY_STOP},//BCU急停
};

static const fault_mapping_t fault_map_3H[] = {
    {8, PCS_STOP},//外部急停(充电打桩)
    {9, ISO_SWITCH_FAULT},//隔开开关故障
    {10,DOOR_OPEN},//门禁故障
};

static const fault_mapping_t fault_map_2H[] = {
    {6, INSIDE_NTC_FAULT},//内部温度故障
    {7, OUTSIDE_COM_FAULT},//外部温度故障
};


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
	if (BCU_SystemWorkMode_LAST != get_BCU_SystemWorkModeValue())
	{
		log_flag = 1;
		BCU_SystemWorkMode_LAST = get_BCU_SystemWorkModeValue();
	}

	if (BCU_FaultInfoLv1H_LAST != get_BCU_FaultInfoLv1HValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv1H_LAST = get_BCU_FaultInfoLv1HValue();
	}
	if (BCU_FaultInfoLv1L_LAST != get_BCU_FaultInfoLv1LValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv1L_LAST = get_BCU_FaultInfoLv1LValue();
	}
	if (BCU_FaultInfoLv2H_LAST != get_BCU_FaultInfoLv2HValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv2H_LAST = get_BCU_FaultInfoLv2HValue();
	}
	if (BCU_FaultInfoLv2L_LAST != get_BCU_FaultInfoLv2LValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv2L_LAST = get_BCU_FaultInfoLv2LValue();
	}
	if (BCU_FaultInfoLv3H_LAST != get_BCU_FaultInfoLv3HValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv3H_LAST = get_BCU_FaultInfoLv3HValue();
	}
	if (BCU_FaultInfoLv3L_LAST != get_BCU_FaultInfoLv3LValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv3L_LAST = get_BCU_FaultInfoLv3LValue();
	}
	if (BCU_FaultInfoLv4H_LAST != get_BCU_FaultInfoLv4HValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv4H_LAST = get_BCU_FaultInfoLv4HValue();
	}
	if (BCU_FaultInfoLv4L_LAST != get_BCU_FaultInfoLv4LValue())
	{
		log_flag = 1;
		BCU_FaultInfoLv4L_LAST = get_BCU_FaultInfoLv4LValue();
	}

	if (log_flag == 1)
	{
		LOG_CSV("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			ecu_fault.emcu_fault0,
			ecu_fault.emcu_fault1,
			ecu_fault.emcu_fault2,
			get_BCU_FaultInfoLv1HValue(),
			get_BCU_FaultInfoLv1LValue(),
			get_BCU_FaultInfoLv2HValue(),
			get_BCU_FaultInfoLv2LValue(),
			get_BCU_FaultInfoLv3HValue(),
			get_BCU_FaultInfoLv3LValue(),
			get_BCU_FaultInfoLv4HValue(),
			get_BCU_FaultInfoLv4LValue(),
			get_BCU_SOCValue(),
			get_BCU_SystemStatusValue(),
			get_BCU_SystemWorkModeValue());
	}
}


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


void update_fault_tomodus(void)
{
	int temp = 0;
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT0, ecu_fault.emcu_fault0);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT1, ecu_fault.emcu_fault1);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT2, ecu_fault.emcu_fault2);
	set_modbus_reg_val(MDBUS_ADDR_BECU_FAULT3, ecu_fault.emcu_fault3);


	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT0, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT0 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT1, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT1 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT2, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT2 = 0x%x\r\n",temp);
	// get_modbus_reg_val(MDBUS_ADDR_BECU_FAULT3, &temp);
	// printf("MDBUS_ADDR_BECU_FAULT3 = 0x%x\r\n",temp);

}

void ECUfault_process()
{
	update_fault_tomodus();
}


/********************************************************************************
 *
 * 输入参数：
 *                      unsigned int parameter   参数 //詳見fault_intaface.h
 *                      unsigned char status        分机的状态 1 0
 * 			无
 * 输出参数：无
 ********************************************************************************/
void set_emcu_fault(unsigned char parameter, unsigned char status)
{

	unsigned char byte_num = (parameter & 0xf0) >> 4; // 高4位字节号
	unsigned short bit_num = (parameter & 0x0F);	  //  低4位bit位

	switch (byte_num)
	{
	case 0:
		if (status)
		{
			ecu_fault.emcu_fault0 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault0 |= (1 << bit_num);
		}
		break;
	case 1:
		if (status)
		{
			ecu_fault.emcu_fault1 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault1 |= (1 << bit_num);
		}
		break;
	case 2:
		if (status)
		{
			ecu_fault.emcu_fault2 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault2 |= (1 << bit_num);
			
		}
		break;
	case 3:
		if (status)
		{
			ecu_fault.emcu_fault3 &= ~(1 << bit_num);
		}
		else
		{
			ecu_fault.emcu_fault3 |= (1 << bit_num);
		}
		break;
	default:
		break;
	}
	if (ecu_fault.emcu_fault0 + ecu_fault.emcu_fault1 + ecu_fault.emcu_fault2)
	{
		ecu_fault.emcu_fault_state = 1;
	}
	else
	{
		ecu_fault.emcu_fault_state = 0;
	}
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
		LOG("[Check] can2 abnormal, restarting...\n");
		restart_can_interface(BCU_CAN_DEVICE_NAME);
	}
	g_can2_ready = can2_state;

	// 检查 can3
	int can3_state = check_can_state(BMU_CAN_DEVICE_NAME);
	if (can3_state == 0 && g_can3_ready == 1) {
		LOG("[Check] can3 abnormal, restarting...\n");
		restart_can_interface("can3");
	}
	g_can3_ready = can3_state;

}

// 重启CAN接口
static void restart_can_interface(const char* can_if) {
    LOG("[Check] Restarting %s...\n", can_if);
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


void get_BCU_FaultInfo(uint32_T faultValue_4H, uint32_T faultValue_3H,uint32_T faultValue_2H)
{
    unsigned int i = 0;
    //故障映射，新增故障只需要更改fault_map_4H
    // printf("faultValue_3H = %x\n", faultValue_3H);
    // printf("faultValue_2H = %x\n", faultValue_2H);
    for ( i = 0; i < sizeof(fault_map_4H)/sizeof(fault_map_4H[0]); i++) 
    {
        if (faultValue_4H & (1UL << fault_map_4H[i].bit_position))
        {
            set_emcu_fault(fault_map_4H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_4H[i].fault_type, SET_RECOVER);
        }
    }
    //故障映射，新增故障只需要更改fault_map_3H
    for ( i = 0; i < sizeof(fault_map_3H)/sizeof(fault_map_3H[0]); i++) 
    {
        if (faultValue_3H & (1UL << fault_map_3H[i].bit_position))
        {
            set_emcu_fault(fault_map_3H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_3H[i].fault_type, SET_RECOVER);
        }
    }
    
    //故障映射，新增故障只需要更改fault_map_2H
    for ( i = 0; i < sizeof(fault_map_2H)/sizeof(fault_map_2H[0]); i++) 
    {
        if (faultValue_2H & (1UL << fault_map_2H[i].bit_position))
        {
            set_emcu_fault(fault_map_2H[i].fault_type, SET_ERROR);
        } 
        else 
        {
            set_emcu_fault(fault_map_2H[i].fault_type, SET_RECOVER);
        }
    }
}