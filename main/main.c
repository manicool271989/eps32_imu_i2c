#include <stdio.h>
#include <string.h>

// ESP32 libraries.
#include "driver/i2c.h"
#include "imu.h"

// FreeRTOS libraries.
#include "freertos/FreeRTOS.h"

// ESP32 I2C Library Functions
//
// esp_err_t i2c_master_write_to_device(i2c_port_t i2c_num,
//                                      uint8_t device_address, 
//                                      const uint8_t *write_buffer,
//                                      size_t write_size,
//                                      TickType_t ticks_to_wait
//                                      );
//
// esp_err_t i2c_master_read_from_device(i2c_port_t i2c_num,
//                                       uint8_t device_address,
//                                       uint8_t *read_buffer,
//                                       size_t read_size,
//                                       TickType_t ticks_to_wait
//                                       );



#define ACK_CHECK_EN 0x1

void app_main(void)
{

    // Define I2C peripheral configuration through I2C config struct/object (master).
    i2c_config_t conf_master = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 23,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    // I2C slave.
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = 17,
        .scl_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = 0x0A
    };

    // Configure I2C peripheral.
    i2c_param_config(I2C_NUM_0, &conf_master);
    i2c_param_config(I2C_NUM_1, &conf_slave);

    // Enable/activate I2C peripheral.
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_driver_install(I2C_NUM_1, I2C_MODE_SLAVE, 255, 255, 0);

    // Write Buffer used to tell IMU what register we will read from.
    //const uint8_t write_buf_msb[0] = {ACCEL_XOUT_15_8};
    //const uint8_t write_buf_lsb[0] = {ACCEL_XOUT_7_0};
    //const uint8_t write_buf_msb[] = {0x3B};
    //const uint8_t write_buf_lsb[] = {0x3C};   
    uint8_t write_buf_msb[1];
    uint8_t write_buf[2] = {0x6B, 0x00};
    // Read Buffer
    uint8_t read_buf[255];

    // Write IMU Register PWR_MGMT_1
    i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf, 2, 1000/portTICK_PERIOD_MS);
    vTaskDelay(500/portTICK_PERIOD_MS);

    // Write IMU Register ACCEL_CONFIG
    write_buf[0] = 0x1C;
    write_buf[1] = IMU_AFS_SEL;
    i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf, 2, 1000/portTICK_PERIOD_MS);
    vTaskDelay(500/portTICK_PERIOD_MS);
    
    // Write
    while(1)
    {
        //printf("{write_buf_msb[0]} = {%x}\n", write_buf_msb[0]);

        // Read IMU Register ACCEL_XOUT
        memset(read_buf, 0, 255);
        write_buf_msb[0] = 0x3B;
        i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf_msb, 1, 1000/portTICK_PERIOD_MS);
        i2c_master_read_from_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, read_buf, 6, 1000/portTICK_PERIOD_MS);

        // Store raw accelerometer data
        imu6050_1.accelerometer.raw_data[X_AXIS] = (int16_t) read_buf[0] << 8 | (int16_t) read_buf[1];
        imu6050_1.accelerometer.raw_data[Y_AXIS] = (int16_t) read_buf[2] << 8 | (int16_t) read_buf[3];
        imu6050_1.accelerometer.raw_data[Z_AXIS] = (int16_t) read_buf[4] << 8 | (int16_t) read_buf[5];

        // printf("IMU ACCEL RAW X Y Z: %d %d %d\n",  imu6050_1.accelerometer.raw_data[X_AXIS],
        //                                  imu6050_1.accelerometer.raw_data[Y_AXIS],
        //                                  imu6050_1.accelerometer.raw_data[Z_AXIS]
        //                                 );

        // Calculate scaled accelerometer data
        imu6050_1.accelerometer.scaled_data[X_AXIS] = (imu6050_1.accelerometer.raw_data[X_AXIS]/IMU_ACCEL_SENSITIVITY_SCALE_FACTOR)*9.81;
        imu6050_1.accelerometer.scaled_data[Y_AXIS] = (imu6050_1.accelerometer.raw_data[Y_AXIS]/IMU_ACCEL_SENSITIVITY_SCALE_FACTOR)*9.81;
        imu6050_1.accelerometer.scaled_data[Z_AXIS] = (imu6050_1.accelerometer.raw_data[Z_AXIS]/IMU_ACCEL_SENSITIVITY_SCALE_FACTOR)*9.81;

        // printf("IMU ACCEL SCALED X Y Z: %f %f %f\n",  (imu6050_1.accelerometer.scaled_data[X_AXIS], 
        //                                  (imu6050_1.accelerometer.scaled_data[Y_AXIS],
        //                                  (imu6050_1.accelerometer.scaled_data[Z_AXIS]
        //                                 );

        
        // Read IMU Register GYRO_XOUT
        memset(read_buf, 0, 255);
        write_buf_msb[0] = 0x43;
        i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf_msb, 1, 1000/portTICK_PERIOD_MS);
        i2c_master_read_from_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, read_buf, 6, 1000/portTICK_PERIOD_MS);

        // Store raw accelerometer data
        imu6050_1.gyroscope.raw_data[X_AXIS] = (int16_t) read_buf[0] << 8 | (int16_t) read_buf[1];
        imu6050_1.gyroscope.raw_data[Y_AXIS] = (int16_t) read_buf[2] << 8 | (int16_t) read_buf[3];
        imu6050_1.gyroscope.raw_data[Z_AXIS] = (int16_t) read_buf[4] << 8 | (int16_t) read_buf[5];

        // printf("IMU GYRO RAW X Y Z: %d %d %d\n",  imu6050_1.gyroscope.raw_data[X_AXIS],
        //                                  imu6050_1.gyroscope.raw_data[Y_AXIS],
        //                                  imu6050_1.gyroscope.raw_data[Z_AXIS]
        //                                 );

        // Calculate scaled gyroscope data
        imu6050_1.gyroscope.scaled_data[X_AXIS] = imu6050_1.gyroscope.raw_data[X_AXIS]/IMU_GYRO_SENSITIVITY_SCALE_FACTOR;
        imu6050_1.gyroscope.scaled_data[Y_AXIS] = imu6050_1.gyroscope.raw_data[Y_AXIS]/IMU_GYRO_SENSITIVITY_SCALE_FACTOR;
        imu6050_1.gyroscope.scaled_data[Z_AXIS] = imu6050_1.gyroscope.raw_data[Z_AXIS]/IMU_GYRO_SENSITIVITY_SCALE_FACTOR;

        printf("IMU GRYO SCALED X Y Z: %f %f %f\n",  imu6050_1.gyroscope.scaled_data[X_AXIS],
                                         imu6050_1.gyroscope.scaled_data[Y_AXIS],
                                         imu6050_1.gyroscope.scaled_data[Z_AXIS]
                                        );
        
        vTaskDelay(100/portTICK_PERIOD_MS);

        // // Read IMU Register WHO_AM_I
        // write_buf_msb[0] = 0x75;
        // i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf_msb, 1, 1000/portTICK_PERIOD_MS);
        // vTaskDelay(500/portTICK_PERIOD_MS);
        // i2c_master_read_from_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, read_buf, 2, 1000/portTICK_PERIOD_MS);
        // printf("WHO_AM_I: %x %x\n", read_buf[0], read_buf[1]);
        // memset(read_buf, 0, 255);
        // vTaskDelay(1000/portTICK_PERIOD_MS);

        // // Read IMU Register ACCEL_YOUT_7_0
        // write_buf_msb[0] = 0x3E;
        // i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf_msb, 1, 1000/portTICK_PERIOD_MS);
        // i2c_master_read_from_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, read_buf, 1, 1000/portTICK_PERIOD_MS);
        // printf("IMU ACCEL_YOUT_7_0 Data = %x %x\n", read_buf[0], read_buf[1]);
        // memset(read_buf, 0, 255);
        // vTaskDelay(1000/portTICK_PERIOD_MS);

        // // Read IMU Register ACCEL_ZOUT_7_0
        // write_buf_msb[0] = 0x40;
        // i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, write_buf_msb, 1, 1000/portTICK_PERIOD_MS);
        // vTaskDelay(500/portTICK_PERIOD_MS);
        // i2c_master_read_from_device(I2C_NUM_0, IMU_I2C_SLV_ADDR, read_buf, 1, 1000/portTICK_PERIOD_MS);
        // vTaskDelay(500/portTICK_PERIOD_MS);
        // printf("IMU ACCEL_ZOUT_7_0 Data = %x %x\n", read_buf[0], read_buf[1]);
        // memset(read_buf, 0, 255);

        //i2c_master_write_to_device(I2C_NUM_0, 0x0A, write_buf_msb, 1, 5000/portTICK_PERIOD_MS);
        // I2C Write
        // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // i2c_master_start(cmd);
        // i2c_master_write_byte(cmd, 0x0A << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
        // i2c_master_write(cmd, write_buf_msb, 1, ACK_CHECK_EN);
        // i2c_master_stop(cmd);
        // i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        // i2c_cmd_link_delete(cmd);


        //i2c_slave_read_buffer(I2C_NUM_1, read_buf, 255, 1000 / portTICK_PERIOD_MS);
        //printf("IMU ACCEL_XOUT_7_0 Data = %x %x\n", read_buf[0], read_buf[1]);

        memset(read_buf, 0, 255);
    };

}