#include "time_diff.h"

/**
 * start_tick:传进来的时间
 * 计算从 start_tick 开始到当前时间经过的毫秒数。
*/
uint32_t GetTimeDifference_ms(struct timespec start_tick)
{
    struct timespec current_tick;
    clock_gettime(CLOCK_MONOTONIC, &current_tick);
    // gettimeofday(&current_tick,NULL);
    uint32_t diff_sec = current_tick.tv_sec - start_tick.tv_sec;
    int32_t diff_nsec = current_tick.tv_nsec - start_tick.tv_nsec;

    return diff_sec * 1000 + diff_nsec / 1000000;
}
