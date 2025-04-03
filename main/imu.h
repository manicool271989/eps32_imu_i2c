#ifndef IMU_H
#define IMU_H
#include <stdint.h>

// IMU-6050 I2C Slave Address
#define IMU_I2C_SLV_ADDR 0x68 //0b110100 AD0 - 0b0
#define IMU_AFS_SEL 0x01 // 0x00 = +/-2g, 0x01 = +/-4g, 0x02 = +/-8g, 0x03 = +/-16g. 
#define IMU_ACCEL_SENSITIVITY_SCALE_FACTOR 8192 // 2g = 16384 LSB/g, 4g = 8192 LSB/g, 8g = 4096 LSB/g, 16g = 2048 LSB/g.
#define IMU_GYRO_SENSITIVITY_SCALE_FACTOR 131 // 250 (deg/s) = 131 LSB/(deg/s), 500 (deg/s) = 65.5 LSB/(deg/s), 1000 (deg/s) = 32.8 LSB/(deg/s), 2000 (deg/s) = 16.4 LSB/(deg/s)


// IMU-6050 Internal Registers
#define ACCEL_XOUT_15_8 0x3B
#define ACCEL_XOUT_7_0 0x3C

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

typedef struct
{
    // [0,1,2] maps to [x,y,z] axis.
    int16_t raw_data[3];
    float scaled_data[3];
} three_axis_sensor_t;

typedef struct
{
    three_axis_sensor_t accelerometer;
    three_axis_sensor_t gyroscope;
} imu_t;

extern imu_t imu6050_1;

void print_raw_data(imu_t imu);

#endif