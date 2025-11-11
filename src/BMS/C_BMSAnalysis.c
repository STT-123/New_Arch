#include <arpa/inet.h> // htonl
#include <byteswap.h>  // bswap_16, bswap_32
#include "C_BMSAnalysis.h"
#include "./GLB/G_AddressDefinition.h"
#include "./BMS/bms/CANSendFcn.h"

float Electric_Meter_BCU_V3 = 102.3;
float Electric_Meter_BCU_Curr2;
int32_t Electric_Meter_BCU_RealtimePower;
float Electric_Meter_BCU_EngryAccumulateChrg;
float Electric_Meter_BCU_EngryAccumulateDisChrg;

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
bool BMSAnalysis(unsigned char can_flag)
{
    bool state = false;
    static unsigned int call_count = 0;
    bms_CANRcvFcn();

    // 每60次调用执行一次发送,6种数据，每一个都会+1
    if ((call_count == 0) && (can_flag == 0)) {
        bms_CANSendFcn();
        printf("call_count = %d\n", call_count);
    }
    call_count = (call_count + 1) % 60;

    state = true; // 天才的返回值
    return state;
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

void my_modbus_set_float_badc(float f, uint16_t *dest)
{
    uint32_t i;

    memcpy(&i, &f, sizeof(uint32_t));
    i = htonl(i);
    dest[0] = (uint16_t)bswap_16(i >> 16);
    dest[1] = (uint16_t)bswap_16(i & 0xFFFF);
}

void Set_BCU_Voltage(float voltage)
{
    uint16_t temp[2] = {0};
    // printf("voltage: %f\n", voltage);
    float adjusted_voltage = voltage;
    my_modbus_set_float_badc(adjusted_voltage, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_VOL, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_VOL + 1, temp[1]);
}

void Set_BCU_Current(float current)
{
    uint16_t temp[2] = {0};
    // printf("current: %f\n", current);
    float adjusted_current = current;
    my_modbus_set_float_badc(adjusted_current, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_CUR, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_CUR + 1, temp[1]);
}

void Set_BCU_Power(int32_t power_watt)
{
    // float power_kw = (float)power_watt / 1000.0f;
    // printf("power_watt: %f\n", power_watt);
    uint16_t temp[2] = {0};
    my_modbus_set_float_badc(power_watt, temp);
    set_modbus_reg_val(MDBUS_ADDR_DC_POW, temp[0]);
    set_modbus_reg_val(MDBUS_ADDR_DC_POW + 1, temp[1]);
}

void Set_BCU_PositiveEnergy(float energy_wh)
{
    uint32_t energy_mwh = energy_wh * 1000;
    // printf("energy_mwh: %d\n", energy_mwh);
    set_modbus_reg_val(MDBUS_ADDR_P_ENERGY, energy_mwh & 0xFFFF);
    set_modbus_reg_val(MDBUS_ADDR_P_ENERGY + 1, energy_mwh >> 16);
}

void Set_BCU_NegativeEnergy(float energy_wh)
{
    uint32_t energy_mwh = energy_wh * 1000;
    // printf("energy_mwh: %d\n", energy_mwh);
    set_modbus_reg_val(MDBUS_ADDR_N_ENERGY, energy_mwh & 0xFFFF);
    set_modbus_reg_val(MDBUS_ADDR_N_ENERGY + 1, energy_mwh >> 16);
}

void set_OTA_XCPConnect(real_T value) { OTA_XCPConnect = value; }
real_T get_OTA_XCPConnect(void) { return OTA_XCPConnect; }

void set_TCU_ACMuteSet(real_T value) { TCU_ACMuteSet = value; }
real_T get_TCU_ACMuteSet(void) { return TCU_ACMuteSet; }

void set_TCU_BCUEINSet(real_T value) { TCU_BCUEINSet = value; }
real_T get_TCU_BCUEINSet(void) { return TCU_BCUEINSet; }

void set_TCU_ChargerWorkSts(real_T value) { TCU_ChargerWorkSts = value; }
real_T get_TCU_ChargerWorkSts(void) { return TCU_ChargerWorkSts; }

// void set_TCU_ClearFault(real_T value) { TCU_ClearFault = value; }
// real_T get_TCU_ClearFault(void) { return TCU_ClearFault; }

void set_TCU_ECOMode(real_T value) { TCU_ECOMode = value; }
real_T get_TCU_ECOMode(void) { return TCU_ECOMode; }

// void set_TCU_GetTime(real_T value) { TCU_GetTime = value; }
// real_T get_TCU_GetTime(void) { return TCU_GetTime; }

void set_TCU_LifeCounter(real_T value) { TCU_LifeCounter = value; }
real_T get_TCU_LifeCounter(void) { return TCU_LifeCounter; }

void set_TCU_PowerUpCmd(real_T value) { TCU_PowerUpCmd = value; }
real_T get_TCU_PowerUpCmd(void) { return TCU_PowerUpCmd; }

void set_TCU_TimeCalFlg(real_T value) { TCU_TimeCalFlg = value; }
real_T get_TCU_TimeCalFlg(void) { return TCU_TimeCalFlg; }

void set_TCU_TimeDay(real_T value) { TCU_TimeDay = value; }
real_T get_TCU_TimeDay(void) { return TCU_TimeDay; }

void set_TCU_TimeHour(real_T value) { TCU_TimeHour = value; }
real_T get_TCU_TimeHour(void) { return TCU_TimeHour; }

void set_TCU_TimeMinute(real_T value) { TCU_TimeMinute = value; }
real_T get_TCU_TimeMinute(void) { return TCU_TimeMinute; }

void set_TCU_TimeMonth(real_T value) { TCU_TimeMonth = value; }
real_T get_TCU_TimeMonth(void) { return TCU_TimeMonth; }

void set_TCU_TimeSecond(real_T value) { TCU_TimeSecond = value; }
real_T get_TCU_TimeSecond(void) { return TCU_TimeSecond; }

void set_TCU_TimeWeek(real_T value) { TCU_TimeWeek = value; }
real_T get_TCU_TimeWeek(void) { return TCU_TimeWeek; }

void set_TCU_TimeYear(real_T value) { TCU_TimeYear = value; }
real_T get_TCU_TimeYear(void) { return TCU_TimeYear; }

void set_TCU_FcnStopSet(real_T value) { TCU_FcnStopSet = value; }
real_T get_TCU_FcnStopSet(void) { return TCU_FcnStopSet; }

void set_TCU_HighVoltType(real_T value) { TCU_HighVoltType = value; }
real_T get_TCU_HighVoltType(void) { return TCU_HighVoltType; }

void set_TCU_HighVoltValue(real_T value) { TCU_HighVoltValue = value; }
real_T get_TCU_HighVoltValue(void) { return TCU_HighVoltValue; }

/**
 * 将标准 canfd_frame 转换为 CAN_FD_MESSAGE_BUS
 */
void ConvertCANFDToBus(const struct canfd_frame *frame, CAN_FD_MESSAGE_BUS *msg)
{
    if (!frame || !msg)
        return;
    // printf("Raw can_id      : 0x%08lX\n", frame->can_id);
    msg->ProtocolMode = 1; // 1 表示 CAN FD
    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;

    msg->BRS = 0;
    msg->ESI = 0;

    msg->Length = frame->len;
    msg->DLC = frame->len;
    // printf("Raw can_id      : 0x%08lX\n", frame->can_id);
    // printf("Extended    : %d\n", (frame->can_id & CAN_EFF_FLAG) ? 1 : 0);
    // printf(" frame->can_id : %08lX\r\n", frame->can_id);
    msg->ID = frame->can_id; // 取 29 位
    // printf(" msg->ID  : %08lX\r\n", msg->ID );
    msg->Reserved = 0;

    msg->Timestamp = 0;

    memcpy(msg->Data, frame->data, frame->len);
}

void ConvertBusToCANFD(const CAN_FD_MESSAGE_BUS *msg, struct canfd_frame *frame)
{
    if (!msg || !frame)
        return;

    // 清空目标结构体
    memset(frame, 0, sizeof(*frame));

    // 设置 CAN ID 和标志位
    frame->can_id = msg->ID;
    // printf("frame->can_id : %08lX\r\n", frame->can_id);
    // 添加扩展帧标志（EFF）
    if (msg->Extended)
    {
        frame->can_id |= CAN_EFF_FLAG;
        // printf("msg->Extended : %d\r\n",msg->Extended);
    }

    // 添加远程帧标志（RTR）
    if (msg->Remote)
    {
        frame->can_id |= CAN_RTR_FLAG;
        //  printf("msg->Remote : %d\r\n",msg->Remote);
    }

    // 添加错误帧标志（ERR）
    if (msg->Error)
    {
        frame->can_id |= CAN_ERR_FLAG;
        //  printf("msg->Error : %d\r\n",msg->Error);
    }

    // 设置数据长度
    frame->len = msg->Length;
    // frame->len = msg->DLC;
    // 设置 CAN FD 特有标志（BRS 和 ESI）
    frame->flags = 0;
    if (msg->BRS)
    {
        frame->flags |= CANFD_BRS;
        // printf("msg->BRS : %d\r\n",msg->BRS);
    }
    if (msg->ESI)
    {
        frame->flags |= CANFD_ESI;
        // printf("msg->ESI : %d\r\n",msg->ESI);
        // canfd 的错误帧状态
    }

    // printf("msg->Length : %d\r\n", msg->Length);
    // printf("Data: ");
    // for (int i = 0; i < msg->Length; ++i) {
    //     printf("%02X ", msg->Data[i]);
    // }
    // printf("\r\n");
    // printf("frame->flags : %d\r\n", frame->flags);
    memcpy(frame->data, msg->Data, msg->Length);
    // printf("frame->len : %d\r\n", frame->len);
    // printf("Data: ");
    // for (int i = 0; i < frame->len; ++i) {
    //     printf("%02X ", frame->data[i]);
    // }
    // printf("\r\n");
}

void Convert_CAN_MESSAGE_to_can_frame(const CAN_MESSAGE *msg, struct can_frame *frame)
{

    memset(frame, 0, sizeof(struct can_frame));

    // printf("msg->ID : %08lX\r\n", msg->ID);
    frame->can_id = msg->ID;

    if (msg->Extended)
    {
        frame->can_id |= CAN_EFF_FLAG;
    }

    if (msg->Remote)
    {
        frame->can_id |= CAN_RTR_FLAG;
    }

    if (msg->Error)
    {
        frame->can_id |= CAN_ERR_FLAG;
    }

    //  printf("msg->Length  : %d\r\n", msg->Length );
    if (msg->Length <= 8)
    {
        frame->can_dlc = msg->Length;
    }
    else
    {
        frame->can_dlc = 8;
    }

    memcpy(frame->data, msg->Data, frame->can_dlc);
    // printf("rame->len : %d\r\n", frame->can_dlc);
    // printf("Data: ");
    // for (int i = 0; i < frame->can_dlc ; ++i) {
    //     printf("%02X ", frame->data[i]);
    // }
    // printf("\r\n");
}

void Convert_canfd_frame_to_CAN_MESSAGE(const struct canfd_frame *frame, CAN_MESSAGE *msg)
{
    memset(msg, 0, sizeof(CAN_MESSAGE));

    // 提取 CAN ID
    msg->ID = frame->can_id & CAN_EFF_MASK;

    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;

    // 判断是否是扩展帧
    // msg->Extended = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0;
    // printf(" msg->Extended:  %d", msg->Extended);

    // // 判断是否是远程帧
    // msg->Remote = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;
    // printf(" msg->Remote:  %d", msg->Remote );
    // // 判断是否是错误帧
    // msg->Error = (frame->can_id & CAN_ERR_FLAG) ? 1 : 0;
    // printf(" msg->Error:  %d", msg->Error);
    // 数据长度：截断为最大8字节
    msg->Length = (frame->len > 8) ? 8 : frame->len;

    msg->Timestamp = 0;

    // 拷贝数据
    memcpy(msg->Data, frame->data, msg->Length);
}

void Convert_can_frame_to_CAN_MESSAGE(const struct can_frame *frame, CAN_MESSAGE *msg)
{
    memset(msg, 0, sizeof(CAN_MESSAGE));
    // printf("frame->can_id : %08lX\r\n", frame->can_id);
    // 提取 ID，去掉扩展/远程/错误标志位
    msg->ID = frame->can_id & CAN_EFF_MASK;

    // 判断是否为扩展帧
    // msg->Extended = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0;

    // // 判断是否为远程帧
    // msg->Remote = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0;

    // // 判断是否为错误帧
    // msg->Error = (frame->can_id & CAN_ERR_FLAG) ? 1 : 0;
    msg->Extended = 1;
    msg->Remote = 0;
    msg->Error = 0;

    // 数据长度控制在 0~8 之间
    if (frame->can_dlc <= 8)
    {
        msg->Length = frame->can_dlc;
    }
    else
    {
        msg->Length = 8;
    }
    // printf("msg->Length : %d\r\n", msg->Length);
    // 拷贝数据
    memcpy(msg->Data, frame->data, frame->can_dlc);

    // printf("rame->len : %d\r\n", frame->can_dlc);
    // printf("Data: ");
    // for (int i = 0; i < frame->can_dlc ; ++i) {
    //     printf("%02X ", frame->data[i]);
    // }
    // printf("\r\n");

#ifndef TIMESTAMP_NOT_REQUIRED
    // 如果定义了时间戳字段，初始化为 0.0（无实际来源）
    msg->Timestamp = 0.0;
#endif
}
