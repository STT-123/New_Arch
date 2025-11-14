#ifndef __IP_SETTING_H
#define __IP_SETTING_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "stdbool.h"
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h> // for inet_ntoa etc.
#include <netinet/in.h>
#include <sys/stat.h> // stat()

#define MODBUS_ETH_NUM  "eht1"

void settings_Init();
void save_ip_to_conffile(uint16_t address, uint16_t data);
void set_system_time_from_bcu(void);
int set_ip_address(const char *if_name, const char *ip_addr);
#endif