#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define ICM42688_ADDR 0x68
#define WHO_AM_I_REG 0x75
#define WHO_AM_I_EXPECTED 0x47

#define PWR_MGMT0_REG 0x4E
#define ACCEL_CONFIG0_REG 0x14
#define GYRO_CONFIG0_REG 0x11
#define TEMP_DATA1_REG 0x1D
#define ACCEL_DATA_REG 0x1F
#define GYRO_DATA_REG 0x25

#define ACCEL_FS_SEL 0x03  // ±16g
#define GYRO_FS_SEL  0x03  // ±2000dps
#define ACCEL_ODR    0x07  // 1.6 kHz
#define GYRO_ODR     0x07  // 1.6 kHz

// Default I2C instance
i2c_inst_t *i2c = i2c_default;

static void icm42688_reset() {
    uint8_t buf[] = {PWR_MGMT0_REG, 0x01};  // Reset the device
    i2c_write_blocking(i2c, ICM42688_ADDR, buf, 2, false);
    sleep_ms(100);  // Wait for reset to complete

    // Set PWR_MGMT0 to enable accelerometer and gyroscope in LN mode
    buf[0] = PWR_MGMT0_REG;
    buf[1] = 0x0F;  // Set GYRO_MODE and ACCEL_MODE to LN mode, enable temp sensor
    i2c_write_blocking(i2c, ICM42688_ADDR, buf, 2, false);

    // Additional sensor configuration can follow here...
}


static void icm42688_configure() {
    uint8_t buf[2];

    // Configure Accelerometer
    buf[0] = ACCEL_CONFIG0_REG;
    buf[1] = (ACCEL_FS_SEL << 5) | ACCEL_ODR;  // Set full scale and ODR
    int ret = i2c_write_blocking(i2c, ICM42688_ADDR, buf, 2, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Failed to write to ACCEL_CONFIG0_REG\n");
    }
    else{
        printf("Wrote to ACCEL_CONFIG0_REG\n");
    }
    sleep_ms(10);  // Small delay to ensure register write completion

    // Configure Gyroscope
    buf[0] = GYRO_CONFIG0_REG;
    buf[1] = (GYRO_FS_SEL << 5) | GYRO_ODR;  // Set full scale and ODR
    ret = i2c_write_blocking(i2c, ICM42688_ADDR, buf, 2, false);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Failed to write to GYRO_CONFIG0_REG\n");
    }
    else{
        printf("Wrote to ACCEL_CONFIG0_REG\n");
    }
    sleep_ms(10);  // Small delay to ensure register write completion
}

static uint8_t icm42688_who_am_i() {
    uint8_t reg = WHO_AM_I_REG;
    uint8_t id;
    i2c_write_blocking(i2c, ICM42688_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, ICM42688_ADDR, &id, 1, false);
    return id;
}

static void icm42688_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    // Read accelerometer data
    uint8_t val = ACCEL_DATA_REG;
    i2c_write_blocking(i2c, ICM42688_ADDR, &val, 1, true); 
    i2c_read_blocking(i2c, ICM42688_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Read gyroscope data
    val = GYRO_DATA_REG;
    i2c_write_blocking(i2c, ICM42688_ADDR, &val, 1, true);
    i2c_read_blocking(i2c, ICM42688_ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Read temperature data
    val = TEMP_DATA1_REG;
    i2c_write_blocking(i2c, ICM42688_ADDR, &val, 1, true);
    i2c_read_blocking(i2c, ICM42688_ADDR, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

static uint8_t icm42688_read_register(uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(i2c, ICM42688_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c, ICM42688_ADDR, &val, 1, false);
    return val;
}

int main() {
    stdio_init_all();

    i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    printf("Initializing ICM42688...\n");

    icm42688_reset();
    icm42688_configure();

    uint8_t who_am_i = icm42688_who_am_i();
    printf("WHO_AM_I: 0x%02X\n", who_am_i);
    if (who_am_i != WHO_AM_I_EXPECTED) {
        printf("ICM42688 not found!\n");
    }


    int16_t accel[3], gyro[3], temp;

    while (1) {
            // Read back configuration registers
        icm42688_configure();
        uint8_t accel_config = icm42688_read_register(ACCEL_CONFIG0_REG);
        uint8_t gyro_config = icm42688_read_register(GYRO_CONFIG0_REG);
        printf("ACCEL_CONFIG0: 0x%02X\n", accel_config);
        printf("GYRO_CONFIG0: 0x%02X\n", gyro_config);

        icm42688_read_raw(accel, gyro, &temp);

        printf("Acc. X = %d, Y = %d, Z = %d\n", accel[0], accel[1], accel[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        printf("Temp. = %f\n", (temp / 132.48) + 25.0);

        sleep_ms(1000);
    }
}
