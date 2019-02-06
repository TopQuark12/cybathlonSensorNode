#ifndef ICM20602_H
#define ICM20602_H

#define ICM20602_SPI_DRIVER	                &hspi1
#define ICM20602_TIMEOUT_TICKS	            100
#define ICM20602_UPDATE_BUFFER_SIZE	        15

#define ICM20602_GYRO_MEASUREMENT_RANGE     GYRO_FS_500DPS
#define ICM20602_ACCEL_MEASUREMENT_RANGE    ACCEL_FS_16G

#define ICM20602_TEMP_SENSITIVITY	        (float) 1/326.8
#define ICM20602_TEMP_OFFSET	            (float) 25.0


//icm20602 register address list
typedef enum
{
    CONFIG = 0x1A,
    GYRO_CONFIG,
    ACCEL_CONFIG,
    ACCEL_CONFIG2,
    ACCEL_XOUT_H = 0x3B,
    ACCEL_XOUT_L,
    ACCEL_YOUT_H,
    ACCEL_YOUT_L,
    ACCEL_ZOUT_H,
    ACCEL_ZOUT_L,
    TEMP_OUT_H,
    TEMP_OUT_L,
    GYRO_XOUT_H,
    GYRO_XOUT_L,
    GYRO_YOUT_H,
    GYRO_YOUT_L,
    GYRO_ZOUT_H,
    GYRO_ZOUT_L,
    PWR_MGMT_1 = 0x6B,
    WHO_AM_I = 0x75
} icm20602Reg;

//gyroscope and temperature sensor LP-filter config
typedef enum
{
    GYRO_LPF_250HZ, //250Hz -3db bandwidth
    GYRO_LPF_176HZ,
    GYRO_LPF_92HZ,
    GYRO_LPF_41HZ,
    GYRO_LPF_20HZ,
    GYRO_LPF_10HZ,
    GYRO_LPF_5HZ,
    GYRO_LPF_3281HZ
} gyroLPF_e;

//gyroscope fullscale measurement range
typedef enum
{
    GYRO_FS_250DPS, //+-250 degrees per second full scale
    GYRO_FS_500DPS,
    GYRO_FS_1000DPS,
    GYRO_FS_2000DPS,
} gyroFS_e;

//accelerometer LP-filter config
typedef enum
{
    ACCEL_LPF_218_1 = 1,    //218.1Hz -3db bandwidth
    ACCEL_LPF_99,
    ACCEL_LPF_44_8,
    ACCEL_LPF_21_2,
    ACCEL_LPF_10_2,
    ACCEL_LPF_5_1,
    ACCEL_LPF_420
} accelLPF_e;

//accelerometer fullscale measurement range
typedef enum
{
    ACCEL_FS_2G,    //+-2g fullscale
    ACCEL_FS_4G,
    ACCEL_FS_8G,
    ACCEL_FS_16G
} accelFS_e;

//axes name
typedef enum
{
    xAxis = 0,
    yAxis,
    zAxis
} axisType_e;

//structure for scaled IMU data
typedef struct imu_t
{
    float accData[3];  //unit: g
    float gyroData[3]; //unit: dps
    float tempData;    //unit: deg C
} imu_t;

uint8_t icm20602Init(void);
void icm20602Update(void);

#endif //ICM20602_h