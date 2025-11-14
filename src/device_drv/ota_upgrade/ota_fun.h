#ifndef _OTA_FUN_H
#define _OTA_FUN_H 
#include <string.h>
#include <pthread.h>
#include "interface/globalVariable.h"


 #define XMODEMISTTRANFERINGFILE 0x0001  // XMODEM正在传输文件
#define XMODEMNORMALTERMINATION 0x0002  // XMODEM正常终止（传输成功）
#define XMODEMOUTTIMETERMINATION 0x0004 // XMODEM超时终止
#define XMODEMABORTTERMINATION 0x0006   // XMODEM异常中止
//以上4个未使用
#define FILEISDECRYPTIIONING 0x0008		// 文件正在解密中
#define FILEDECRYPTIONNORMALTERMINATION 0x0010  // 文件解密正常完成

#define OTASTARTRUNNING 0x0080 		// OTA开始运行
#define OTAIDLE 0x0000				// OTA空闲状态
#define OTASUCCESS 0x0100			// OTA升级成功
#define OTAFAILED 0x0200			// OTA升级失败

#define OTAFILENAMEMAXLENGTH 128
#define MAX_FILE_COUNT 20

typedef enum
{
	ECU = 1,
	BCU,
	BMU,
	AC,
	ACP,
	DCDC
} OTADEVICETYPE;


typedef struct
{
	_Atomic unsigned char OTAFileType;
	char OTAFilename[OTAFILENAMEMAXLENGTH];
	char OTAUdsSblFilename[MAX_FILE_COUNT][OTAFILENAMEMAXLENGTH];
	char OTAUdsFilename[MAX_FILE_COUNT][OTAFILENAMEMAXLENGTH];
	_Atomic unsigned int deviceID;//原子操作
	_Atomic unsigned char deviceType;
	_Atomic unsigned char OTAStart;
	_Atomic unsigned char multDeviceOTA;
	_Atomic unsigned char multDeviceOTANum;
	_Atomic unsigned char UpDating;
} OTAObject;

extern OTAObject g_otactrl;

// ============ 原子变量的 get/set 函数实现 ============

// deviceID 操作
inline unsigned int get_deviceID(void);
inline void set_deviceID(unsigned int value);

// deviceType 操作  
inline unsigned char get_deviceType(void);
inline void set_deviceType(unsigned char value);

// OTAStart 操作
inline unsigned char get_OTAStart(void);
inline void set_OTAStart(unsigned char value);

// multDeviceOTA 操作
inline unsigned char get_multDeviceOTA(void);
inline void set_multDeviceOTA(unsigned char value);

// multDeviceOTANum 操作
inline unsigned char get_multDeviceOTANum(void);
inline void set_multDeviceOTANum(unsigned char value);

// UpDating 操作
inline unsigned char get_UpDating(void);
inline void set_UpDating(unsigned char value);

// OTAFileType 操作
inline unsigned char get_UpDating(void);
inline void set_UpDating(unsigned char value);

// OTAFileType 操作
inline unsigned char get_OTAFileType(void);
inline void set_OTAFileType(unsigned char value);

// ============ 非原子变量的 get/set 函数 ============
// OTAFilename 操作
inline const char* get_OTAFilename(void);
inline void set_OTAFilename(const char* filename);

// OTAUdsSblFilename 操作
inline const char* get_OTAUdsSblFilename(int index);
inline void set_OTAUdsSblFilename(int index, const char* filename);

// OTAUdsFilename 操作
inline const char* get_OTAUdsFilename(int index);
inline void set_OTAUdsFilename(int index, const char* filename);

#endif