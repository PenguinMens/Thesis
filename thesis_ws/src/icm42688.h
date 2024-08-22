#ifndef ICM42688_H
#define ICM42688_H

#include <stdint.h>

#define ROS_MODE 0  // Set to 1 for ROS mode, 0 for non-ROS

void i2c_init_icm42688();
void icm42688_init();

void icm42688_write_register(uint8_t reg, uint8_t value);
uint8_t icm42688_read_register(uint8_t reg);
void icm42688_read_registers(uint8_t reg, uint8_t* buf, uint8_t len);

void icm42688_read_accel(float *ax, float *ay, float *az);
void icm42688_read_gyro(float *gx, float *gy, float *gz);



#endif
