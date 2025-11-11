#ifndef __DRV_EXTERNAL_EMERGENCY_STOP_H__
#define __DRV_EXTERNAL_EMERGENCY_STOP_H__

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

// #define EMERGENCY_STOP     496
// #define PCSSTOP     32

#define Emergencystop_output 32

extern time_t g_last_bmu_rx_time;

void PHYlinktate();
void log_eror_csv(void);
void ECUfault_process();
void check_bmu_rx_timeout(void);

int can_monitor_fun(void) ;
void restart_can_interface(const char* can_if);
int check_can_state(const char* can_if) ;
int is_can2_ready(void);
int is_can3_ready(void);
#endif