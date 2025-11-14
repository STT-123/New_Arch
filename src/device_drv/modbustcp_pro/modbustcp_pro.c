#include "modbustcp_pro.h"
#include "interface/globalVariable.h"
#include "function_task/modbustcp_task/modbustcp_task.h"
#include "device_drv/bcu_deal/bcu_deal.h"
#include "device_drv/sd_store/sd_store.h"
#include "interface/log/log.h"
#include "device_drv/ota_upgrade/ota_fun.h"
extern unsigned short g_ota_flag;

// modbus接收数据处理，只处理06的写入操作
 void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length)
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
            if ((address == 0x6700) && (g_otactrl.UpDating == 0)) // 过滤，自己需要判断是否在升级来进行自主上下电
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
                rtc_Modbus_Deal(address, data);
            }
            // 设置ip
            else if (address == 0x6711 || address == 0x6712)
            {
                save_ip_to_conffile(address, data);
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

static int update_system_time(const Rtc_Ip_TimedateType *timeData)
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
 * 函数名称： rtc_Modbus_Deal
 * 功能描述： ModBus设置RTC指令
 * 输入参数：
 * 输出参数： 0 表示写入成功，1表示写入完成，-1表示失败。
 *sqw
 ********************************************************************************/
static int rtc_Modbus_Deal(uint16_t address, uint16_t data)
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

		update_system_time(&TmData);
		LOG("[ModbusTcp] rtc_Modbus_Deal\r\n");
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
static int BatteryCalibration_ModBus_Deal(uint16_t address, uint16_t data)
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

static int VoltageCalibration_ModBus_Deal(uint16_t address, uint16_t data)
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

static void set_ems_bms_reboot()
{
	// Drv_bmu_canfd_send(&bms_reboot_msg);
	set_OTA_XCPConnect(170);
	BMSAnalysis(0);
	BMSAnalysis(1);
	usleep(250 * 1000);

	printf("\r\n\r\n  ******* ECU cmd Reset  *******  r\n\r\n");

	system("reboot"); // 复位并准备跳转
}
