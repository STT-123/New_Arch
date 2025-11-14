#include "ota_ecu_update.h"
#include "interface/globalVariable.h"
#include "interface/log/log.h"
#include "interface/bms/bms_analysis.h"
#include "interface/modbus/modbus_defines.h"
#include "device_drv/xmodem/xmodemdata.h"
#include "device_drv/ota_upgrade/ota_fun.h"
#include "device_drv/sd_store/sd_store.h"

#define APP_PATH  "/opt/xcharge"  

ECUStatus ecustatus;


void ECU_OTA(OTAObject *pOTA)
{
    LOG("[OTA] ECU_OTA start!, pOTA->OTAStart:%d\r\n", pOTA->OTAStart);
    if (!pOTA->OTAStart) return;
    
    memset(&ecustatus, 0, sizeof(ECUStatus));
    LOG("[OTA] pOTA->deviceType : %d \r\n", pOTA->deviceType);
    LOG("[OTA] can id 0x%x device ota start!\r\n", pOTA->deviceID);
    set_modbus_reg_val(OTASTATUSREGADDR, OTASTARTRUNNING);
    
    if(pOTA->deviceID == 0 && pOTA->deviceType == ECU)
    {
        char cmd[512];
        set_modbus_reg_val(OTAPPROGRESSREGADDR, 10); // 0124
        
        // 步骤1: 修改文件后缀为.deb并检查是否已存在
        if(ecustatus.ErrorReg == 0)
        {
            char deb_filename[256] = {'\0'};
            char source_file[512] = {'\0'};
            char target_file[512] = {'\0'};
            
            // 将原文件名从.bin改为.deb
            char *dot_pos = strrchr(pOTA->OTAFilename, '.');
            if (dot_pos && strcmp(dot_pos, ".bin") == 0) {
                // 替换.bin为.deb
                strncpy(deb_filename, pOTA->OTAFilename, dot_pos - pOTA->OTAFilename);
                strcat(deb_filename, ".deb");
            } else {
                // 如果没有.bin后缀，直接添加.deb
                snprintf(deb_filename, sizeof(deb_filename), "%s.deb", pOTA->OTAFilename);
            }
            
            // 构建源文件和目标文件路径
            snprintf(source_file, sizeof(source_file), "%s/%s", USB_MOUNT_POINT, pOTA->OTAFilename);
            snprintf(target_file, sizeof(target_file), "/tmp/%s", deb_filename);
            
            LOG("Source file path: %s\n", source_file);
            LOG("Target file path: %s\n", target_file);

            // 检查源文件是否存在
            if (access(source_file, F_OK) != 0) {
                printf("OTA source file does not exist: %s\n", source_file);
                ecustatus.ErrorReg |= 1 << 2;
                return;
            }

            // 验证源文件
            if (!verify_bin_file(source_file)) {
                printf("Source file verification failed: %s\n", source_file);
                ecustatus.ErrorReg |= 1 << 4;
                return;
            }
            
            LOG("[OTA] ECU bin file verified: %s\n", pOTA->OTAFilename);
            
            // 检查目标文件是否已存在
            if (access(target_file, F_OK) == 0) {
                LOG("[OTA] Target file already exists in /tmp: %s, skipping copy and preparing for reboot\n", deb_filename);
                // 文件已存在，设置进度并准备重启
                set_modbus_reg_val(OTAPPROGRESSREGADDR, 100);
                
                // 完成OTA清理工作
                FinshhECUOtaAndCleanup(pOTA);
                
                // 延迟后重启
                LOG("[OTA] File already exists, preparing to reboot...\n");
                system("sync");
                
                sleep(5); // 5秒后重启
                system("reboot");
                return; // 直接返回，不执行后续代码
            }
            
            LOG("[OTA] Target file does not exist in /tmp, proceeding with copy\n");
        }

        set_modbus_reg_val(OTAPPROGRESSREGADDR, 40); // 0124
        
        // 步骤2: 复制文件到/tmp目录（使用修改后的文件名）
        if(ecustatus.ErrorReg == 0)
        {
            char deb_filename[256] = {'\0'};
            char source_file[512] = {'\0'};
            char target_file[512] = {'\0'};
            
            // 同样处理文件名
            char *dot_pos = strrchr(pOTA->OTAFilename, '.');
            if (dot_pos && strcmp(dot_pos, ".bin") == 0) {
                strncpy(deb_filename, pOTA->OTAFilename, dot_pos - pOTA->OTAFilename);
                strcat(deb_filename, ".deb");
            } else {
                snprintf(deb_filename, sizeof(deb_filename), "%s.deb", pOTA->OTAFilename);
            }
            
            snprintf(source_file, sizeof(source_file), "%s/%s", USB_MOUNT_POINT, pOTA->OTAFilename);
            snprintf(target_file, sizeof(target_file), "/tmp/%s", deb_filename);
            
            memset(cmd, 0, sizeof(cmd));
            snprintf(cmd, sizeof(cmd), "cp \"%s\" \"%s\"", source_file, target_file);
            LOG("[OTA] Copy command: %s\n", cmd);
            
            int ret = system(cmd);
            if (ret == 0) {
                LOG("[OTA] File copy to /tmp successful: %s\n", deb_filename);
            } else {
                LOG("[OTA] File copy to /tmp failed\n");
                ecustatus.ErrorReg |= 1 << 2;
            }
        }

        set_modbus_reg_val(OTAPPROGRESSREGADDR, 80); // 0124
        
        // 步骤3: 验证复制后的文件
        if(ecustatus.ErrorReg == 0)
        {
            char deb_filename[256] = {'\0'};
            char target_file[512] = {'\0'};
            
            // 处理文件名
            char *dot_pos = strrchr(pOTA->OTAFilename, '.');
            if (dot_pos && strcmp(dot_pos, ".bin") == 0) {
                strncpy(deb_filename, pOTA->OTAFilename, dot_pos - pOTA->OTAFilename);
                strcat(deb_filename, ".deb");
            } else {
                snprintf(deb_filename, sizeof(deb_filename), "%s.deb", pOTA->OTAFilename);
            }
            
            snprintf(target_file, sizeof(target_file), "/tmp/%s", deb_filename);
            
            if (!verify_bin_file(target_file)) {
                LOG("[OTA] Copied file verification failed: %s\n", target_file);
                ecustatus.ErrorReg |= 1 << 4;
            } else {
                LOG("[OTA] Copied file verification successful: %s\n", target_file);
            }
        }

        // 步骤4: 完成操作并重启
        if(ecustatus.ErrorReg == 0)
        {
            set_modbus_reg_val(OTAPPROGRESSREGADDR, 100); // 0124
            
            // 完成OTA清理工作
            FinshhECUOtaAndCleanup(pOTA);
            
            // 确保数据写入磁盘并重启
            LOG("[OTA] File copied to /tmp successfully, preparing to reboot...\n");
            system("sync");
            
            sleep(5); // 5秒后重启
            //system("reboot");
        }

        // 错误处理
        if(ecustatus.ErrorReg != 0)
        {
            printf("can id 0x%x device ota failed, error register val 0x%x!\r\n", 
                   pOTA->deviceID, ecustatus.ErrorReg);
            LOG("[OTA] can id 0x%x device ota failed, error register val 0x%x!\r\n", 
                pOTA->deviceID, ecustatus.ErrorReg);
            set_modbus_reg_val(OTASTATUSREGADDR, OTAFAILED);
        }
        
        pOTA->OTAStart = 0;
    }
    else{
        LOG("[OTA] pOTA->deviceID = 0x%x, pOTA->deviceType = %d\r\n",pOTA->deviceID, pOTA->deviceType);
        ecustatus.ErrorReg = 1;
        ecustatus.ErrorDeviceID = pOTA->deviceID;
    }
}



void FinshhECUOtaAndCleanup(OTAObject* pOTA)
{
    pOTA->deviceType = 0;//停止升级
    pOTA->OTAStart = 0;
	// delete_files_with_prefix(USB_MOUNT_POINT, "XC");//  这个要删除升级文件，判断ecustatus状态，成功或者失败删除
	// delete_files_with_prefix(USB_MOUNT_POINT, "md5");// 
	g_otactrl.UpDating = 0;//1130(升级结束)
	ecustatus.CANStartOTA = 0;
	// set_charger_cmd(BMS_POWER_DEFAULT);
    set_TCU_PowerUpCmd(BMS_POWER_DEFAULT);
	set_modbus_reg_val(OTASTATUSREGADDR, OTAIDLE);
}


// 简单的文件验证（不依赖file命令）
static int verify_bin_file(const char *filepath)
{
    FILE *fp = fopen(filepath, "rb");
    if (!fp) {
        printf("Cannot open file for verification: %s\n", filepath);
        return 0;
    }
    
    // 检查文件大小
    fseek(fp, 0, SEEK_END);
    long file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    
    printf("File size: %ld bytes\n", file_size);
    
    // ECU程序通常有一定的大小范围
    if (file_size < 5 * 1024 || file_size > 10 * 1024 * 1024) {
        printf("File size %ld not in expected range\n", file_size);
        fclose(fp);
        return 0;
    }
    
    // 可选：检查ELF魔数
    unsigned char header[4];
    if (fread(header, 1, 4, fp) == 4) {
        // ELF文件头: 0x7F 'E' 'L' 'F'
        if (header[0] == 0x7F && header[1] == 'E' && 
            header[2] == 'L' && header[3] == 'F') {
            printf("File has valid ELF header\n");
            fclose(fp);
            return 1;
        }
    }
    
    fclose(fp);
    
    // 即使不是标准ELF也继续（可能是去掉了头的二进制）
    printf("File may be a binary (not standard ELF), continuing...\n");
    return 1;
}