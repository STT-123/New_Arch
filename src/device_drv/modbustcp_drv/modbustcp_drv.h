#ifndef __C_MODBUS_UPDATE_H__
#define __C_MODBUS_UPDATE_H__

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include "interface/modbus/modbus.h"
#include "interface/modbus/modbus-tcp.h"
#include "interface/G_GloabalVariable.h"
#define REGISTERS_NB 0x4000 // 寄存器数量

#define MDBUS_ADDR_BECU_FAULT0 0x3440
#define MDBUS_ADDR_BECU_FAULT1 0x3441
#define MDBUS_ADDR_BECU_FAULT2 0x3442
#define MDBUS_ADDR_BECU_FAULT3 0x3443

static void set_ems_bms_reboot();
static int VoltageCalibration_ModBus_Deal(uint16_t address, uint16_t data);
static int update_system_time(const Rtc_Ip_TimedateType *timeData);
static int RTC_ModBus_Deal(uint16_t address, uint16_t data);
static int BatteryCalibration_ModBus_Deal(uint16_t address, uint16_t data);

int set_modbus_reg_val(uint16_t addr, uint16_t set_val);
int get_modbus_reg_val(uint16_t addr, uint16_t *get_val);
void modbus_write_reg_deal(modbus_t *ctx, const uint8_t *query, int req_length);

#endif