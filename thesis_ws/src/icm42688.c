#include "icm42688.h"
#include "pico/stdlib.h"



int reg_write(i2c_inst_t *i2c, 
              const uint addr, 
              const uint8_t reg, 
              uint8_t *buf,
              const uint8_t nbytes) {

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return -1;  // Return an error if no bytes to send
    }

    uint8_t msg[nbytes + 1];

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C and check for success
    int result = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);
    if (result < 0) {
        printf("I2C write failed with error code: %d\n", result);
        return result;  // Return the error code if write failed
    }

    return result;  // Return the number of bytes written
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}

void icm42688_reset(i2c_inst_t *i2c) {
    uint8_t buf[] = {ICM42688_SIGNAL_PATH_RESET, 0x01};  // Reset the device
    reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);
    sleep_ms(100);  // Wait for reset to complete

    // Set PWR_MGMT0 to enable accelerometer and gyroscope in LN mode
    buf[0] = ICM42688_PWR_MGMT0;
    buf[1] = 0x0F;  // Set GYRO_MODE and ACCEL_MODE to LN mode, enable temp sensor
    reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);

    // Additional sensor configuration can follow here...
}


static uint8_t icm42688_who_am_i(i2c_inst_t *i2c) {
    uint8_t reg = ICM42688_WHO_AM_I;
    uint8_t id;
    i2c_write_blocking(i2c, ICM42688_I2C_L_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, ICM42688_I2C_L_ADDR, &id, 1, false);
    return id;
}


void icm42688_init(i2c_inst_t *i2c) {
    printf("Initializing ICM-42688P...\n");

    // WHO AM I
    uint8_t id = icm42688_who_am_i(i2c);
    if (id != ICM42688_ID) {
        printf("WHO_AM_I failed. Expected 0x49, got 0x%02X\n", id);
        return ; // Early return on error
    } else {
        printf("WHO_AM_I successful. Device ID: 0x%02X\n", id);
    }


    // Reset the device
    printf("Attempting to reset the device...\n");
    uint8_t buf[] = {ICM42688_SIGNAL_PATH_RESET, 0x80};
    int result = reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);
    
    if (result < 0) {
        printf("Failed to reset the device. Error code: %d\n", result);
        return; // Early return on error
    } else {
        printf("Device reset successful.\n");
    }

    sleep_ms(100);  // Wait for reset

    // Configure gyroscope
    printf("Configuring gyroscope: Full scale ±1000dps, ODR 1kHz...\n");
    uint8_t gyro_config = 0b00000110;
    result = reg_write(i2c, ICM42688_I2C_L_ADDR, ICM42688_GYRO_CONFIG0, &gyro_config, 1);

    if (result < 0) {
        printf("Failed to configure gyroscope. Error code: %d\n", result);
        return; // Early return on error
    } else {
        printf("Gyroscope configured successfully.\n");
    }

    // Configure accelerometer
    printf("Configuring accelerometer: Full scale ±4g, ODR 1kHz...\n");
    uint8_t accel_config = 0b01000110;
    result = reg_write(i2c, ICM42688_I2C_L_ADDR, ICM42688_ACCEL_CONFIG0, &accel_config, 1);

    if (result < 0) {
        printf("Failed to configure accelerometer. Error code: %d\n", result);
        return; // Early return on error
    } else {
        printf("Accelerometer configured successfully.\n");
    }


    printf("ICM-42688P initialized.\n");
}


void icm42688_read_accel(i2c_inst_t *i2c, float *ax, float *ay, float *az) {
    uint8_t rawData[6];
    // Read 6 bytes from the accelerometer x0, x1, y0, y1, z0, z1
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_ACCEL_DATA_X1, rawData, 6);

    // Scaling factor based on ±4g full-scale range
    float accelScale = 4.0f / 32768.0f;

    *ax = (float)((int16_t)(rawData[0] << 8 | rawData[1])) * accelScale;
    *ay = (float)((int16_t)(rawData[2] << 8 | rawData[3])) * accelScale;
    *az = (float)((int16_t)(rawData[4] << 8 | rawData[5])) * accelScale;

    // printf("Accel: ax=%.2f, ay=%.2f, az=%.2f\n", *ax, *ay, *az);
}

void icm42688_read_gyro(i2c_inst_t *i2c, float *gx, float *gy, float *gz) {
    uint8_t rawData[6];
    // Read 6 bytes from the gyroscope x0, x1, y0, y1, z0, z1
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_GYRO_DATA_X1, rawData, 6);

    // Scaling factor based on ±2000 dps full-scale range
    float gyroScale = 2000.0f / 32768.0f;

    *gx = (float)((int16_t)(rawData[0] << 8 | rawData[1])) * gyroScale;
    *gy = (float)((int16_t)(rawData[2] << 8 | rawData[3])) * gyroScale;
    *gz = (float)((int16_t)(rawData[4] << 8 | rawData[5])) * gyroScale;

    // printf("Gyro: gx=%.2f, gy=%.2f, gz=%.2f\n", *gx, *gy, *gz);
}