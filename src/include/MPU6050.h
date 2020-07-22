#ifndef _MPU6050_H_
#define _MPU6050_H_

// 用于树莓派的MPU6050驱动，连接方式为I2C，使用WiringPi操作GPIO。
// 参考：
// https://github.com/ElectronicCats/mpu6050
// https://github.com/kriswiner/MPU6050

// 使用指南：可按照如下顺序使用该设备，
// 创建类 > 设置参数 > 初始化 > 唤醒 > 读取数据

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <cmath>

#define GRAVITY 9.7985

// ******* 以下为寄存器地址 *******
#define MPU6050_RA_XG_OFFS_TC 0x00 // TC指温度补偿
#define MPU6050_RA_YG_OFFS_TC 0x01
#define MPU6050_RA_ZG_OFFS_TC 0x02
#define MPU6050_RA_X_FINE_GAIN 0x03
#define MPU6050_RA_Y_FINE_GAIN 0x04
#define MPU6050_RA_Z_FINE_GAIN 0x05
#define MPU6050_RA_XA_OFFS_H 0x06
#define MPU6050_RA_XA_OFFS_L_TC 0x07
#define MPU6050_RA_YA_OFFS_H 0x08
#define MPU6050_RA_YA_OFFS_L_TC 0x09
#define MPU6050_RA_ZA_OFFS_H 0x0A
#define MPU6050_RA_ZA_OFFS_L_TC 0x0B
// 自检结果
#define MPU6050_RA_SELF_TEST_X 0x0D
#define MPU6050_RA_SELF_TEST_Y 0x0E
#define MPU6050_RA_SELF_TEST_Z 0x0F
#define MPU6050_RA_SELF_TEST_A 0x10
#define MPU6050_RA_XG_OFFS_USRH 0x13
#define MPU6050_RA_XG_OFFS_USRL 0x14
#define MPU6050_RA_YG_OFFS_USRH 0x15
#define MPU6050_RA_YG_OFFS_USRL 0x16
#define MPU6050_RA_ZG_OFFS_USRH 0x17
#define MPU6050_RA_ZG_OFFS_USRL 0x18
// SMPLRT_DIV(Sample Rate Divider); sample rate = gyroscope output rate / (1 + SMPLRT_DIV)
// if DLPF is disabled, gyroscope output rate = 8kHz; else, gyroscope output rate = 1kHz.
// 采样率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV)
// 如果未启用低通滤波器，陀螺仪输出频率为8kHz；如果启用低通滤波器，陀螺仪输出频率为1kHz。
#define MPU6050_RA_SMPLRT_DIV 0x19
// FSYNC config and DLPF config
// 低三位为低通滤波器的设置项
#define MPU6050_RA_CONFIG 0x1A
// gyroscope self-test enable and gyroscope range
// 陀螺仪自检开关和陀螺仪量程设置
#define MPU6050_RA_GYRO_CONFIG 0x1B
// accelerometer self-test enable and accelerator range
// 加速度计自检开关和加速度计量程设置
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_RA_FF_THR 0x1D
#define MPU6050_RA_FF_DUR 0x1E
#define MPU6050_RA_MOT_THR 0x1F
#define MPU6050_RA_MOT_DUR 0x20
#define MPU6050_RA_ZRMOT_THR 0x21
#define MPU6050_RA_ZRMOT_DUR 0x22
// FIFO buffer switch
// FIFO暂存的开关
#define MPU6050_RA_FIFO_EN 0x23
#define MPU6050_RA_I2C_MST_CTRL 0x24
#define MPU6050_RA_I2C_SLV0_ADDR 0x25 // SLV 从机
#define MPU6050_RA_I2C_SLV0_REG 0x26
#define MPU6050_RA_I2C_SLV0_CTRL 0x27
#define MPU6050_RA_I2C_SLV1_ADDR 0x28
#define MPU6050_RA_I2C_SLV1_REG 0x29
#define MPU6050_RA_I2C_SLV1_CTRL 0x2A
#define MPU6050_RA_I2C_SLV2_ADDR 0x2B
#define MPU6050_RA_I2C_SLV2_REG 0x2C
#define MPU6050_RA_I2C_SLV2_CTRL 0x2D
#define MPU6050_RA_I2C_SLV3_ADDR 0x2E
#define MPU6050_RA_I2C_SLV3_REG 0x2F
#define MPU6050_RA_I2C_SLV3_CTRL 0x30
#define MPU6050_RA_I2C_SLV4_ADDR 0x31
#define MPU6050_RA_I2C_SLV4_REG 0x32
#define MPU6050_RA_I2C_SLV4_DO 0x33
#define MPU6050_RA_I2C_SLV4_CTRL 0x34
#define MPU6050_RA_I2C_SLV4_DI 0x35
#define MPU6050_RA_I2C_MST_STATUS 0x36
#define MPU6050_RA_INT_PIN_CFG 0x37 // INT 中断
#define MPU6050_RA_INT_ENABLE 0x38
#define MPU6050_RA_DMP_INT_STATUS 0x39
#define MPU6050_RA_INT_STATUS 0x3A
// accelerometer output
// 加速度计输出
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40
// temperature output
// 温度输出
#define MPU6050_RA_TEMP_OUT_H 0x41
#define MPU6050_RA_TEMP_OUT_L 0x42
// gyroscope output
// 陀螺仪输出
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48
// slave output
// 从机输出
#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS 0x61
#define MPU6050_RA_I2C_SLV0_DO 0x63
#define MPU6050_RA_I2C_SLV1_DO 0x64
#define MPU6050_RA_I2C_SLV2_DO 0x65
#define MPU6050_RA_I2C_SLV3_DO 0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL 0x67
// reset
// 低三位分别重置陀螺仪、加速度计、温度
#define MPU6050_RA_SIGNAL_PATH_RESET 0x68
#define MPU6050_RA_MOT_DETECT_CTRL 0x69
// enable FIFO buffer, I2C master mode
// reset FIFO buffer, I2C master mode, all signal path
// FIFO、I2C master mode 的开关，以及复位开关
// 陀螺仪、加速度计、温度的重置
#define MPU6050_RA_USER_CTRL 0x6A
// device reset, sleep, cycle, clock source
// 设备重置、休眠、休眠与唤醒交替模式、时钟源
#define MPU6050_RA_PWR_MGMT_1 0x6B
// individual axes standby mode
// 每个轴单独设置待命模式，用于省电
#define MPU6050_RA_PWR_MGMT_2 0x6C
#define MPU6050_RA_BANK_SEL 0x6D
#define MPU6050_RA_MEM_START_ADDR 0x6E
#define MPU6050_RA_MEM_R_W 0x6F
#define MPU6050_RA_DMP_CFG_1 0x70
#define MPU6050_RA_DMP_CFG_2 0x71
// number of samples currently in FIFO buffer
// FIFO中样本数目
#define MPU6050_RA_FIFO_COUNTH 0x72
#define MPU6050_RA_FIFO_COUNTL 0x73
// FIFO data
#define MPU6050_RA_FIFO_R_W 0x74
// WHO AN I，default value 0x68
#define MPU6050_RA_WHO_AM_I 0x75

// ******* 以上为寄存器地址 *******

#define MPU6050_DEFAULT_ADDRESS 0x68
#define MPU6050_DEFAULT_WHO_AM_I 0x68

// full scale range
#define MPU6050_GYRO_FS_250DPS 0x00
#define MPU6050_GYRO_FS_500DPS 0x01
#define MPU6050_GYRO_FS_1000DPS 0x02
#define MPU6050_GYRO_FS_2000DPS 0x03

#define MPU6050_ACCEL_FS_2G 0x00
#define MPU6050_ACCEL_FS_4G 0x01
#define MPU6050_ACCEL_FS_8G 0x02
#define MPU6050_ACCEL_FS_16G 0x03

// DLPF 低通滤波器
#define MPU6050_DLPF_BW_256 0x00
#define MPU6050_DLPF_BW_188 0x01
#define MPU6050_DLPF_BW_98 0x02
#define MPU6050_DLPF_BW_42 0x03
#define MPU6050_DLPF_BW_20 0x04
#define MPU6050_DLPF_BW_10 0x05
#define MPU6050_DLPF_BW_5 0x06

class MPU6050_SENSOR
{
public:
    MPU6050_SENSOR(uint8_t);

    uint8_t getSampleRateDivider();
    void setSampleRateDivider(uint8_t);
    unsigned int getSampleRate();
    void setSampleRate(unsigned int);

    uint8_t getDLPFConfig();
    void setDLPFConfig(uint8_t);

    uint8_t getFullScaleGyroRange();
    void setFullScaleGyroRange(uint8_t);
    uint8_t getFullScaleAccelRange();
    void setFullScaleAccelRange(uint8_t);

    void getAccelRawData(int16_t *, int16_t *, int16_t *);
    void getAccelData(double *, double *, double *);
    void getGyroRawData(int16_t *, int16_t *, int16_t *);
    void getGyroData(double *, double *, double *);
    void getTempRawData(int16_t *);
    void getTempData(double *);

    void resetGyroSignalPath();
    void resetAccelSignalPath();
    void resetTempSignalPath();
    void resetAllSignalPath();

    bool getFIFOEnabled();
    void setFIFOEnabled();
    void setFIFODisabled();
    void resetFIFO();

    void resetDevice();
    bool getSleepStatus();
    void setSleep();
    void setWake();

    bool getTempDisabled();
    void setTempDisable();
    void setTemEnable();

    uint8_t getClockSource();
    void setClockSource(uint8_t);

    void selfTest(uint8_t, uint8_t, double *);
    void initialize(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

private:
    // 使用WiringPi的I2C库时，需要先初始化I2C。
    // 使用设备地址初始化I2C，返回一个标准的 Linux file handle。
    int file_handle; // linux file handle for sensor mpu6050

    /* sensitivity
     * modified in @setFullScaleGyroRange(), @setFullScaleAccelRange()
     * used in @getGyroData(), @getAccelData()
     */
    double gyro_sensitivity;
    double accel_sensitivity;
};

#endif // _MPU6050_H_