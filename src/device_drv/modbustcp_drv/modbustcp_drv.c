#include "modbustcp_drv.h"
#include "interface/G_GloabalVariable.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "device_drv/bcu_drv/bcu_drv.h"
#include "device_drv/sd_deal/sd_deal.h"
#include "interface/log/log.h"

ecu_fault_t ecu_fault;
ecu_fault_t ecu_fault_last;
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




/********************************************************************************
 * 函数名称： get_modbus_reg_val
 * 功能描述：   获取modbus寄存器的值
 * 输入参数：
 * 输出参数： 0表示获取成功，非0表示获取失败
 ********************************************************************************/
int get_modbus_reg_val(uint16_t addr, uint16_t *get_val)
{
	if (modbusBuff == NULL)
	{
		return -1;
	}
	if ((addr >= REGISTERS_START_ADDRESS) && (addr < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
	{
		*get_val = modbusBuff[addr - REGISTERS_START_ADDRESS];
		return 0;
	}
	else
	{
		*get_val = 0;
		return -2;
	}
	return 0;
}
/********************************************************************************
 * 函数名称： set_modbus_reg_val
 * 功能描述：   向modbus寄存器的值
 * 输入参数：
 * 输出参数： 0 表示写入成功，非0表示写入失败
 ********************************************************************************/
int set_modbus_reg_val(uint16_t addr, uint16_t set_val)
{
	if (modbusBuff == NULL)
	{
		return -1;
	}
	if ((addr >= REGISTERS_START_ADDRESS) && (addr < (REGISTERS_START_ADDRESS + REGISTERS_NB)))
	{
		modbusBuff[addr - REGISTERS_START_ADDRESS] = set_val;
		return 0;
	}
	else
	{
		return -2;
	}
	return 0;
}

int update_system_time(const Rtc_Ip_TimedateType *timeData)
{
	if (timeData == NULL)
	{
		fprintf(stderr, "Invalid time data pointer\n");
		return -1;
	}

	// 构造 struct tm
	struct tm external_time = {0};
	external_time.tm_year = timeData->year - 1900; // tm_year 是从1900年起计算的
	external_time.tm_mon = timeData->month - 1;	   // tm_mon 月份从0开始
	external_time.tm_mday = timeData->day;
	external_time.tm_hour = timeData->hour;
	external_time.tm_min = timeData->minutes;
	external_time.tm_sec = timeData->seconds;
	external_time.tm_isdst = -1; // 自动判断夏令时

	// 转换为 time_t
	time_t calibrated_time = mktime(&external_time);

	if (calibrated_time == -1)
	{
		perror("mktime failed");
		return -1;
	}

	// 设置系统时间
	struct timespec ts;
	ts.tv_sec = calibrated_time;
	ts.tv_nsec = 0;

	if (clock_settime(CLOCK_REALTIME, &ts) == -1)
	{
		perror("clock_settime failed (need root?)");
		return -1;
	}

	// 将系统时间写入 RTC
	system("hwclock --systohc");

	return 0;
}

/********************************************************************************
 * 函数名称： RTC_ModBus_Deal
 * 功能描述： ModBus设置RTC指令
 * 输入参数：
 * 输出参数： 0 表示写入成功，1表示写入完成，-1表示失败。
 *sqw
 ********************************************************************************/
int RTC_ModBus_Deal(uint16_t address, uint16_t data)
{
	static Rtc_Ip_TimedateType TmData = {0};

	if (address == 0x6705) // 年
	{
		TmData.year = data;
		return 0; // 成功
	}
	else if (address == 0x6706) // 月
	{
		TmData.month = data;
		return 0; // 成功
	}
	else if (address == 0x6707) // 日
	{
		TmData.day = data;
		return 0; // 成功
	}
	else if (address == 0x6708) // 时
	{
		TmData.hour = data;
		return 0; // 成功
	}
	else if (address == 0x6709) // 分
	{
		TmData.minutes = data;
		return 0; // 成功
	}
	else if (address == 0x670A) // 秒
	{
		static uint8_t rtccount = 0;
		TmData.seconds = (uint8_t)data;

		printf("RTC Set Success!  \r\n");
		set_TCU_TimeYear((TmData.year % 100));
		set_TCU_TimeMonth(TmData.month);
		set_TCU_TimeDay(TmData.day);
		set_TCU_TimeHour(TmData.hour);
		set_TCU_TimeMinute(TmData.minutes);
		set_TCU_TimeSecond(TmData.seconds);
		set_TCU_TimeCalFlg(1);
		// timeinfo.tm_year = (TmData.year%100) + 100;      // BCU年是如24，tm_year从1900起
		// timeinfo.tm_mon  = TmData.month ;
		// timeinfo.tm_hour = TmData.day;
		// timeinfo.tm_min  = TmData.hour;
		// timeinfo.tm_sec  = TmData.seconds;
		// timeinfo.tm_isdst = -1;

		update_system_time(&TmData);
		LOG("RTC_ModBus_Deal\r\n");
		for (int i = 0; i < 3; i++)
		{
			bms_CANSendFcn();
			usleep(1 * 1000);
		}
		set_TCU_TimeCalFlg(0); // RTC设置完毕标志位为0

		return 1; // 完成
	}
	else
	{
		printf("RTC Set Error!  \r\n");
		return -1; // 失败
	}
}

/********************************************************************************
 * 函数名称： BatteryCalibration_ModBus_Deal
 * 功能描述： ModBus设置电池标定指令
 * 输入参数：
 * 输出参数： 0 表示写入成功，1表示写入完成，-1表示失败。
 *sqw
 ********************************************************************************/
int BatteryCalibration_ModBus_Deal(uint16_t address, uint16_t data)
{
	static uint8_t SOHCmd, SOCMaxCmd, SOCMinCmd = 0;
	CANFD_MESSAGE bms_calibration_msg;
	if (address == 0x6714)
	{
		SOHCmd = (data >> 8);
	}
	if (address == 0x6715)
	{
		SOCMaxCmd = (data >> 8);
		SOCMinCmd = (data & 0xff);
	}

	memset(&bms_calibration_msg, 0, sizeof(CAN_MESSAGE));
	bms_calibration_msg.Extended = 1;
	bms_calibration_msg.Length = 64U;
	bms_calibration_msg.ID = 0x1824E410;
	bms_calibration_msg.Extended = 1U;
	bms_calibration_msg.Remote = 0;
	bms_calibration_msg.BRS = 1;
	bms_calibration_msg.ProtocolMode = 1;
	bms_calibration_msg.DLC = 15U;

	bms_calibration_msg.Data[9] = SOHCmd;
	bms_calibration_msg.Data[7] = SOCMinCmd;
	bms_calibration_msg.Data[6] = SOCMaxCmd;

	Drv_bmu_canfd_send(&bms_calibration_msg);
}

int VoltageCalibration_ModBus_Deal(uint16_t address, uint16_t data)
{
	static uint8_t HighVoltType, Offgridstate = 0;

	static uint16_t HighVoltValue = 0;
	if (address == 0x6719)
	{
		Offgridstate = data;
	}
	if (address == 0x6734)
	{
		HighVoltType = data;
	}
	if (address == 0x6735)
	{
		HighVoltValue = data;
	}

	set_TCU_FcnStopSet((real_T)Offgridstate);//bit0：屏蔽故障，支持开关离网,bit1：屏蔽绝缘故障，但是计算绝缘值,bit2：屏蔽绝缘功能，不计算绝缘值
	set_TCU_HighVoltType((real_T)HighVoltType);//电压校准模式
	set_TCU_HighVoltValue((real_T)HighVoltValue);//电压校准数值
}

void set_ems_bms_reboot()
{
	// Drv_bmu_canfd_send(&bms_reboot_msg);
	set_OTA_XCPConnect(170);
	BMSAnalysis(0);
	BMSAnalysis(1);
	usleep(250 * 1000);

	printf("\r\n\r\n  ******* ECU cmd Reset  *******  r\n\r\n");

	system("reboot"); // 复位并准备跳转
}
