#include "ota_fun.h"
OTAObject g_otactrl;
// 互斥锁保护非原子变量
static pthread_mutex_t ota_mutex = PTHREAD_MUTEX_INITIALIZER;

inline unsigned int get_deviceID(void) {
    return atomic_load(&g_otactrl.deviceID);
}

inline void set_deviceID(unsigned int value) {
    atomic_store(&g_otactrl.deviceID, value);
}

inline unsigned char get_deviceType(void) {
    return atomic_load(&g_otactrl.deviceType);
}

inline void set_deviceType(unsigned char value) {
    atomic_store(&g_otactrl.deviceType, value);
}

inline unsigned char get_OTAStart(void) {
    return atomic_load(&g_otactrl.OTAStart);
}

inline void set_OTAStart(unsigned char value) {
    atomic_store(&g_otactrl.OTAStart, value);
}

inline unsigned char get_multDeviceOTA(void) {
    return atomic_load(&g_otactrl.multDeviceOTA);
}

inline void set_multDeviceOTA(unsigned char value) {
    atomic_store(&g_otactrl.multDeviceOTA, value);
}

inline unsigned char get_multDeviceOTANum(void) {
    return atomic_load(&g_otactrl.multDeviceOTANum);
}

inline void set_multDeviceOTANum(unsigned char value) {
    atomic_store(&g_otactrl.multDeviceOTANum, value);
}

inline unsigned char get_UpDating(void) {
    return atomic_load(&g_otactrl.UpDating);
}

inline void set_UpDating(unsigned char value) {
    atomic_store(&g_otactrl.UpDating, value);
}

inline unsigned char get_OTAFileType(void) {
    return atomic_load(&g_otactrl.OTAFileType);
}

inline void set_OTAFileType(unsigned char value) {
    atomic_store(&g_otactrl.OTAFileType, value);
}

// ============ 非原子变量的 get/set 函数实现 ============

inline const char* get_OTAFilename(void) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = g_otactrl.OTAFilename;
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

inline void set_OTAFilename(const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (filename) {
        strncpy(g_otactrl.OTAFilename, filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAFilename[OTAFILENAMEMAXLENGTH - 1] = '\0';
    } else {
        g_otactrl.OTAFilename[0] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}

inline const char* get_OTAUdsSblFilename(int index) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = NULL;
    if (index >= 0 && index < MAX_FILE_COUNT) {
        result = g_otactrl.OTAUdsSblFilename[index];
    }
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

inline void set_OTAUdsSblFilename(int index, const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (index >= 0 && index < MAX_FILE_COUNT && filename) {
        strncpy(g_otactrl.OTAUdsSblFilename[index], filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAUdsSblFilename[index][OTAFILENAMEMAXLENGTH - 1] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}

inline const char* get_OTAUdsFilename(int index) {
    pthread_mutex_lock(&ota_mutex);
    const char* result = NULL;
    if (index >= 0 && index < MAX_FILE_COUNT) {
        result = g_otactrl.OTAUdsFilename[index];
    }
    pthread_mutex_unlock(&ota_mutex);
    return result;
}

inline void set_OTAUdsFilename(int index, const char* filename) {
    pthread_mutex_lock(&ota_mutex);
    if (index >= 0 && index < MAX_FILE_COUNT && filename) {
        strncpy(g_otactrl.OTAUdsFilename[index], filename, OTAFILENAMEMAXLENGTH - 1);
        g_otactrl.OTAUdsFilename[index][OTAFILENAMEMAXLENGTH - 1] = '\0';
    }
    pthread_mutex_unlock(&ota_mutex);
}
