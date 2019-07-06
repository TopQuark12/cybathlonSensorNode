#ifndef ICM20602_H
#define ICM20602_H

#include "cmsis_os.h"

#define ICM20602_SPI_DRIVER	                &hspi1
#define ICM20602_TIMEOUT_TICKS	            100
#define ICM20602_UPDATE_BUFFER_SIZE	        15

#define ICM20602_GYRO_MEASUREMENT_RANGE     GYRO_FS_2000DPS
#define ICM20602_ACCEL_MEASUREMENT_RANGE    ACCEL_FS_16G

#define ICM20602_TEMP_SENSITIVITY	        (float) 1/326.8
#define ICM20602_TEMP_OFFSET	            (float) 25.0

//icm20602 register address list
typedef enum
{
    XG_OFFS_USRH = 0x13, //used to remove DC bias from the sensor output
    XG_OFFS_USRL,        //added to the gyroscope sensor value before going into the sensor register
    YG_OFFS_USRH,        //2's complement
    YG_OFFS_USRL,
    ZG_OFFS_USRH,
    ZG_OFFS_USRL,
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
    WHO_AM_I = 0x75,
    XA_OFFSET_H = 0x77,
    XA_OFFSET_L,
    YA_OFFSET_H = 0x7A,
    YA_OFFSET_L,
    ZA_OFFSET_H = 0x7D,
    ZA_OFFSET_L,
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

typedef struct imuRaw_t
{
    int16_t accData[3];
    int16_t gyroData[3];
    int16_t tempData;
} imuRaw_t;

typedef enum
{
    ACCEL = 0,
    GYRO
} sensorType_e;

typedef struct __attribute__((packed)) imuDataFrame_t
{
    uint32_t timeStamp;
    uint8_t node;
    axisType_e dataType;
    uint16_t dataRaw[2];
} imuDataFrame_t;

typedef union canDataFrame_t
{
    uint32_t uint32[2];
    uint16_t uint16[4];
    uint8_t uint8[8];
} canDataFrame_t;

typedef enum
{
    ROLL = 0,
    PITCH,
    YAW
} rotationalAxis_e;

extern imu_t gIMUdata;
extern imuRaw_t gIMUOffset;

// uint8_t icm20602Init(void);
// void icm20602Update(void);
extern QueueHandle_t imuDataQueueHandle;
void startIMUSampling(void *argument);
float rawConvertionAccel(int16_t *rawData);
float rawConvertionGyro(int16_t *rawData);

#endif //ICM20602_h
