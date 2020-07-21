#include "MPU6050.h"

MPU6050_SENSOR::MPU6050_SENSOR(uint8_t device_address)
{
    // 设置GPIO的针脚编号为WiringPi模式
    if (wiringPiSetup() < 0)
    {
        std::cerr << "ERROR: Failed to setup gpio pin numbering scheme to wiringPi mode." << std::endl;
    }
    // 初始化I2C，获取设备的 file handle
    file_handle = wiringPiI2CSetup(device_address);
    if (file_handle < 0)
    {
        std::cerr << "ERROR: Failed to setup I2C with device address: " << device_address << std::endl;
    }
    // 检测设备是否为 MPU6050
    uint8_t device_id = wiringPiI2CReadReg8(file_handle, MPU6050_RA_WHO_AM_I);
    if (device_id != MPU6050_DEFAULT_WHO_AM_I)
    {
        std::cerr << "ERROR: Wrong device, this is not a mpu6050 sensor." << std::endl;
        std::cerr << std::hex << (int)device_id << std::endl;
    }
}

/* sample rate config  设置采样频率
 * 由寄存器 MPU6050_RA_SMPLRT_DIV 决定
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 * 如果未启用低通滤波器，即低通滤波器设置为0或7，陀螺仪输出频率为8kHz；
 * 如果启用低通滤波器，陀螺仪输出频率为1kHz。
 */
uint8_t MPU6050_SENSOR::getSampleRateDivider()
{
    return wiringPiI2CReadReg8(file_handle, MPU6050_RA_SMPLRT_DIV);
}
void MPU6050_SENSOR::setSampleRateDivider(uint8_t divider)
{
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_SMPLRT_DIV, divider);
}

/* 由于采样率与SMPLRT_DIV和DLPF_CFG有关，这里实现直接设置采样率的函数
 */
unsigned int MPU6050_SENSOR::getSampleRate()
{
    uint8_t dlpf_config = getDLPFConfig();
    unsigned int gyro_output_rate = (dlpf_config == 0 || dlpf_config == 7) ? 8000 : 1000;
    uint8_t divider = getSampleRateDivider();
    return gyro_output_rate / (1 + divider);
}
void MPU6050_SENSOR::setSampleRate(unsigned int rate)
{
    uint8_t dlpf_config = getDLPFConfig();
    unsigned int gyro_output_rate = (dlpf_config == 0 || dlpf_config == 7) ? 8000 : 1000;
    uint8_t divider = gyro_output_rate / rate - 1;
    setSampleRateDivider(divider);
}

/* DLPF config  设置低通滤波器
 * 由寄存器 MPU6050_RA_CONFIG 的低三位决定，
 * 其值推荐设置为Gyroscope采样频率的一半，如采样频率125Hz，DLPF选42Hz。
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 */
uint8_t MPU6050_SENSOR::getDLPFConfig()
{
    return wiringPiI2CReadReg8(file_handle, MPU6050_RA_CONFIG) & 0x07;
}
void MPU6050_SENSOR::setDLPFConfig(uint8_t config)
{
    // 只更改低三位的值
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_CONFIG) & 0xF8 | config;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_CONFIG, whole_byte);
}

/* full scale gyroscope range  陀螺仪量程
 * 由寄存器 MPU6050_RA_GYRO_CONFIG 的三四位决定
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 */
uint8_t MPU6050_SENSOR::getFullScaleGyroRange()
{
    return wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_CONFIG) & 0x18 >> 3;
}
void MPU6050_SENSOR::setFullScaleGyroRange(uint8_t range)
{
    // 只更改三四位
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_CONFIG) & 0x07 | (range << 3);
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_GYRO_CONFIG, whole_byte);

    /* modified the sensitivity of gyroscope
     * 修改类中陀螺仪读数的 sensitivity
     * 因为此值只与量程的变化有关，尽管此值在@getGyroData中才会使用，
     * 在这里修改可以避免其它重复调用的步骤，节省资源。
     * <pre>
     * FS_SEL | Full Scale Range   | LSB Sensitivity
     * -------+--------------------+----------------
     * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
     * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
     * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
     * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
     * </pre>
     */
    switch (range)
    {
    case MPU6050_GYRO_FS_250DPS:
        gyro_sensitivity = 131.0;
        break;
    case MPU6050_GYRO_FS_500DPS:
        gyro_sensitivity = 65.5;
        break;
    case MPU6050_GYRO_FS_1000DPS:
        gyro_sensitivity = 32.8;
        break;
    case MPU6050_GYRO_FS_2000DPS:
        gyro_sensitivity = 16.4;
        break;
    }
}

/* full scale accelerometer range  加速度计量程
 * 由寄存器 MPU6050_RA_ACCEL_CONFIG 的三四位决定
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 */
uint8_t MPU6050_SENSOR::getFullScaleAccelRange()
{
    return wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_CONFIG) & 0x18 >> 3;
}
void MPU6050_SENSOR::setFullScaleAccelRange(uint8_t range)
{
    // 只更改三四位
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_CONFIG) & 0x07 | (range << 3);
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_ACCEL_CONFIG, whole_byte);

    /* modified the sensitivity of accelerometer
     * 修改类中加速度计读数的 sensitivity
     * 因为此值只与量程的变化有关，尽管此值在@getAccelData中才会使用，
     * 在这里修改可以避免其它重复调用的步骤，节省资源。
     * <pre>
     * AFS_SEL | Full Scale Range | LSB Sensitivity
     * --------+------------------+----------------
     * 0       | +/- 2g           | 16384 LSB/g
     * 1       | +/- 4g           | 8192 LSB/g
     * 2       | +/- 8g           | 4096 LSB/g
     * 3       | +/- 16g          | 2048 LSB/g
     * </pre>
     */
    switch (range)
    {
    case MPU6050_ACCEL_FS_2G:
        accel_sensitivity = 16384.0;
        break;
    case MPU6050_ACCEL_FS_4G:
        accel_sensitivity = 8192.0;
        break;
    case MPU6050_ACCEL_FS_8G:
        accel_sensitivity = 4096.0;
        break;
    case MPU6050_ACCEL_FS_16G:
        accel_sensitivity = 2048.0;
        break;
    }
}

/* accelerometer output  加速度计输出
 * 输出为三轴的16位有符号整数，除以 Sensitivity 获得重力加速度的倍率。
 * 为获得单位为 m/s^2 的加速度读数，在H文件中设置了 GRAVITY 的常量
 */
void MPU6050_SENSOR::getAccelRawData(int16_t *xa, int16_t *ya, int16_t *za)
{
    *xa = wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_XOUT_H);
    *xa <<= 8;
    *xa |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_XOUT_L);

    *ya = wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_YOUT_H);
    *ya <<= 8;
    *ya |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_YOUT_L);

    *za = wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_ZOUT_H);
    *za <<= 8;
    *za |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_ACCEL_ZOUT_L);
}
void MPU6050_SENSOR::getAccelData(float *xa, float *ya, float *za)
{
    int16_t xa_raw, ya_raw, za_raw;
    getAccelRawData(&xa_raw, &ya_raw, &za_raw);

    *xa = (float)xa_raw / accel_sensitivity * GRAVITY;
    *ya = (float)ya_raw / accel_sensitivity * GRAVITY;
    *za = (float)za_raw / accel_sensitivity * GRAVITY;
}

/* gyroscope output  陀螺仪输出
 * 输出为三轴的16位有符号整数，除以 Sensitivity 获得单位为度/秒的角速度。
 */
void MPU6050_SENSOR::getGyroRawData(int16_t *xg, int16_t *yg, int16_t *zg)
{
    *xg = wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_XOUT_H);
    *xg <<= 8;
    *xg |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_XOUT_L);

    *yg = wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_YOUT_H);
    *yg <<= 8;
    *yg |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_YOUT_L);

    *zg = wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_ZOUT_H);
    *zg <<= 8;
    *zg |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_GYRO_ZOUT_L);
}
void MPU6050_SENSOR::getGyroData(float *xg, float *yg, float *zg)
{
    int16_t xg_raw, yg_raw, zg_raw;
    getGyroRawData(&xg_raw, &yg_raw, &zg_raw);

    *xg = (float)xg_raw / gyro_sensitivity;
    *yg = (float)yg_raw / gyro_sensitivity;
    *zg = (float)zg_raw / gyro_sensitivity;
}

/* temperature output  温度输出
 * 输出为16位有符号整数，需要转换成单位为摄氏度的温度读数。
 */
void MPU6050_SENSOR::getTempRawData(int16_t *temp)
{
    *temp = wiringPiI2CReadReg8(file_handle, MPU6050_RA_TEMP_OUT_H);
    *temp <<= 8;
    *temp |= wiringPiI2CReadReg8(file_handle, MPU6050_RA_TEMP_OUT_L);
}
void MPU6050_SENSOR::getTempData(float *temp)
{
    // 根据 MPU6050 的数据手册
    int16_t raw;
    getTempRawData(&raw);
    *temp = (float)raw / 340.0 + 36.53;
}

/* reset gyroscope signal path  重置陀螺仪
 * 寄存器 MPU6050_RA_SIGNAL_PATH_RESET 的第二位
 */
void MPU6050_SENSOR::resetGyroSignalPath()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET) | 0x04;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET, whole_byte);
}

/* reset accelerometer signal path  重置加速度计
 * 寄存器 MPU6050_RA_SIGNAL_PATH_RESET 的第一位
 */
void MPU6050_SENSOR::resetAccelSignalPath()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET) | 0x02;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET, whole_byte);
}

/* reset temperature signal path  重置温度计
 * 寄存器 MPU6050_RA_SIGNAL_PATH_RESET 的第零位
 */
void MPU6050_SENSOR::resetTempSignalPath()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET) | 0x01;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_SIGNAL_PATH_RESET, whole_byte);
}

/* reset all signal path  重置所有的信号输出
 * 寄存器 MPU6050_RA_USER_CTRL 的第零位
 */
void MPU6050_SENSOR::resetAllSignalPath()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_USER_CTRL) | 0x01;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_USER_CTRL, whole_byte);
}

/* FIFO buffer switch  各轴FIFO buffer的开关
 * 由寄存器 MPU6050_RA_FIFO_EN 决定
 * <pre>
 *   index   |  7   | 6  | 5  | 4  |   3   |  2   |  1   |  0
 * ----------+------+----+----+----+-------+------+------+-------
 *  describe | TEMP | XG | YG | ZG | ACCEL | SLV2 | SLV1 | SLV0
 * </pre>
 * SLV3的FIFO_ENABLE在寄存器 MPU6050_RA_I2C_MST_CTRL 的五位,
 * 由于目前用不到多个I2C设备，MST(master)和SLV(slave)相关设置没有实现
 */

/* FIFO config and reset  用户控制的FIFO设置与重置
 * 寄存器 MPU6050_RA_USER_CTRL 的第六位用于启用FIFO，第二位用于重置FIFO。
 */
bool MPU6050_SENSOR::getFIFOEnabled()
{
    return (wiringPiI2CReadReg8(file_handle, MPU6050_RA_USER_CTRL) & 0x40) == 0x40;
}
void MPU6050_SENSOR::setFIFOEnabled()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_USER_CTRL) | 0x40;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_USER_CTRL, whole_byte);
}
void MPU6050_SENSOR::setFIFODisabled()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_USER_CTRL) & 0xBF;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_USER_CTRL, whole_byte);
}
void MPU6050_SENSOR::resetFIFO()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_USER_CTRL) | 0x04;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_USER_CTRL, whole_byte);
}

/* reset device  重置设备
 * 电源管理寄存器1 MPU6050_RA_PWR_MGMT_1 第七位
 */
void MPU6050_SENSOR::resetDevice()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) | 0x80;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}

/* sleep  休眠
 * 电源管理寄存器1 MPU6050_RA_PWR_MGMT_1 第六位
 */
bool MPU6050_SENSOR::getSleepStatus()
{
    return (wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0x40) == 0x40;
}
void MPU6050_SENSOR::setSleep()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) | 0x40;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}
void MPU6050_SENSOR::setWake()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0xBF;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}

/* temperature sensor disable  关闭温度计
 * 电源管理寄存器1 MPU6050_RA_PWR_MGMT_1 第三位
 */
bool MPU6050_SENSOR::getTempDisabled()
{
    return (wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0x08) == 0x08;
}
void MPU6050_SENSOR::setTempDisable()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) | 0x08;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}
void MPU6050_SENSOR::setTemEnable()
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0xF7;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}

/* clock source settings  时钟源设置
 * 电源管理寄存器1 MPU6050_RA_PWR_MGMT_1 低三位，
 * 推荐设置为1，即陀螺仪的X轴。
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 */
uint8_t MPU6050_SENSOR::getClockSource()
{
    return wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0x07;
}
void MPU6050_SENSOR::setClockSource(uint8_t source)
{
    uint8_t whole_byte = wiringPiI2CReadReg8(file_handle, MPU6050_RA_PWR_MGMT_1) & 0xF8 | source;
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_PWR_MGMT_1, whole_byte);
}

/* self test
 *
 *
 */
void MPU6050_SENSOR::selfTest(uint8_t gyro_range, uint8_t accel_range, float *differences)
{
    uint8_t raw[4];
    uint8_t self_test_response[6];
    // 设置陀螺仪与加速度计的量程，开启自检
    setFullScaleGyroRange(gyro_range);
    setFullScaleAccelRange(accel_range);
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_GYRO_CONFIG, 0xE0 | (gyro_range << 3));
    wiringPiI2CWriteReg8(file_handle, MPU6050_RA_ACCEL_CONFIG, 0xE0 | (accel_range << 3));
    delay(1000);

    // 读取自检结果
    raw[0] = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SELF_TEST_X);
    raw[1] = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SELF_TEST_Y);
    raw[2] = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SELF_TEST_Z);
    raw[3] = wiringPiI2CReadReg8(file_handle, MPU6050_RA_SELF_TEST_A);

    // 提取各轴自检结果
    // xg
    self_test_response[0] = raw[0] & 0x1F;
    // yg
    self_test_response[1] = raw[1] & 0x1F;
    // zg
    self_test_response[2] = raw[2] & 0x1F;
    // xa
    self_test_response[3] = ((raw[0] & 0xE0) >> 3) | ((raw[4] & 0x30) >> 4);
    // ya
    self_test_response[4] = ((raw[1] & 0xE0) >> 3) | ((raw[4] & 0x0C) >> 2);
    // za
    self_test_response[5] = ((raw[2] & 0xE0) >> 3) | (raw[4] & 0x03);

    // 计算 factory trim, 参考 MPU6050 寄存器手册
    float factory_trim[6];
    factory_trim[0] = 25.0 * gyro_sensitivity * pow(1.046, ((float)self_test_response[0] - 1.0));
    factory_trim[1] = -25.0 * gyro_sensitivity * pow(1.046, ((float)self_test_response[1] - 1.0));
    factory_trim[2] = 25.0 * gyro_sensitivity * pow(1.046, ((float)self_test_response[2] - 1.0));
    factory_trim[3] = 0.34 * accel_sensitivity * pow((0.92 / 0.34), (((float)self_test_response[3] - 1.0) / 30.0));
    factory_trim[4] = 0.34 * accel_sensitivity * pow((0.92 / 0.34), (((float)self_test_response[4] - 1.0) / 30.0));
    factory_trim[5] = 0.34 * accel_sensitivity * pow((0.92 / 0.34), (((float)self_test_response[5] - 1.0) / 30.0));

    // 计算百分比
    for (int i = 0; i < 6; ++i)
    {
        differences[i] = ((float)self_test_response[i] - factory_trim[i]) / factory_trim[i] * 100.0 + 100.0;
    }
}

// 一个常用的初始化过程，可以根据需要定制
void MPU6050_SENSOR::initialize(
    uint8_t sample_rate,
    uint8_t dlpf_config,
    uint8_t gyro_range,
    uint8_t accel_range,
    uint8_t clock_source)
{
    // 重置设备
    resetDevice();
    delay(500);
    // 必须唤醒设备才能进行设置
    setWake();
    delay(100);
    // 设置低通滤波器
    setDLPFConfig(dlpf_config);
    // 设置采样率
    setSampleRate(sample_rate);
    // 设置陀螺仪、加速度计量程
    setFullScaleGyroRange(gyro_range);
    setFullScaleAccelRange(accel_range);
    // 时钟源
    setClockSource(clock_source);
    // 关闭FIFO
    setFIFODisabled();
}