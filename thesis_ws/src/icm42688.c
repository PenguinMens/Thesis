#include "icm42688.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define ICM42688_I2C_ADDR 0x68


void i2c_init_icm42688() {
    printf("Initializing I2C...\n");
    i2c_init(i2c_default, 100 * 1000);  // Initialize I2C at 100kHz

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("I2C initialized.\n");
}

void icm42688_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    int result = i2c_write_blocking(i2c_default, ICM42688_I2C_ADDR, data, 2, false);
    if (result < 0) {
        printf("Failed to write to register 0x%02X\n", reg);
    } else {
        printf("Wrote 0x%02X to register 0x%02X\n", value, reg);
    }
}

uint8_t icm42688_read_register(uint8_t reg) {
    uint8_t value;
    int result = i2c_write_blocking(i2c_default, ICM42688_I2C_ADDR, &reg, 1, true);
    if (result < 0) {
        printf("Failed to write register address 0x%02X\n", reg);
        return 0xFF;  // Return an error value
    }

    result = i2c_read_blocking(i2c_default, ICM42688_I2C_ADDR, &value, 1, false);
    if (result < 0) {
        printf("Failed to read from register 0x%02X\n", reg);
        return 0xFF;  // Return an error value
    }

    printf("Read 0x%02X from register 0x%02X\n", value, reg);
    return value;
}

void icm42688_read_registers(uint8_t reg, uint8_t* buf, uint8_t len) {
    int result = i2c_write_blocking(i2c_default, ICM42688_I2C_ADDR, &reg, 1, true);
    if (result < 0) {
        printf("Failed to write start register address 0x%02X\n", reg);
        return;
    }

    result = i2c_read_blocking(i2c_default, ICM42688_I2C_ADDR, buf, len, false);
    if (result < 0) {
        printf("Failed to read %d bytes from register 0x%02X\n", len, reg);
    } else {
        printf("Read %d bytes starting from register 0x%02X\n", len, reg);
    }
}

void icm42688_init() {
    printf("Initializing ICM-42688P...\n");

    // Reset the device
    icm42688_write_register(0x4B, 0x80);  // Assuming 0x4B is the reset register
    sleep_ms(100);  // Wait for reset

    // Configure accelerometer and gyroscope
    icm42688_write_register(0x1B, 0x18);  // Set gyroscope full scale range to ±2000dps
    icm42688_write_register(0x1C, 0x10);  // Set accelerometer full scale range to ±8g

    printf("ICM-42688P initialized.\n");
}

void icm42688_read_accel(float *ax, float *ay, float *az) {
    uint8_t rawData[6];
    icm42688_read_registers(0x3B1, rawData, 6);  // Assuming 0x3B is the accel data start register

    *ax = (float)((int16_t)(rawData[0] << 8 | rawData[1])) / 4096.0;
    *ay = (float)((int16_t)(rawData[2] << 8 | rawData[3])) / 4096.0;
    *az = (float)((int16_t)(rawData[4] << 8 | rawData[5])) / 4096.0;

    printf("Accel: ax=%.2f, ay=%.2f, az=%.2f\n", *ax, *ay, *az);
}

void icm42688_read_gyro(float *gx, float *gy, float *gz) {
    uint8_t rawData[6];
    icm42688_read_registers(0x43, rawData, 6);  // Assuming 0x43 is the gyro data start register

    *gx = (float)((int16_t)(rawData[0] << 8 | rawData[1])) / 16.4;
    *gy = (float)((int16_t)(rawData[2] << 8 | rawData[3])) / 16.4;
    *gz = (float)((int16_t)(rawData[4] << 8 | rawData[5])) / 16.4;

    printf("Gyro: gx=%.2f, gy=%.2f, gz=%.2f\n", *gx, *gy, *gz);
}
