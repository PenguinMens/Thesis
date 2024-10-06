#include "icm42688.h"
#include "pico/stdlib.h"

#define GYRO_HISTORY_SIZE 10

float gyro_bias_x = -1.35;
float gyro_bias_y = -0.19;
float gyro_bias_z = -0.36;

float gyro_x_history[GYRO_HISTORY_SIZE];
float gyro_y_history[GYRO_HISTORY_SIZE];
float gyro_z_history[GYRO_HISTORY_SIZE];
int idx = 0;

float accel_scale = 4.0f;
float gyro_scale = 1000.0f;

void moving_average(float *gx, float *gy, float *gz) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    gyro_x_history[idx] = *gx;
    gyro_y_history[idx] = *gy;
    gyro_z_history[idx] = *gz;
    idx = (idx + 1) % GYRO_HISTORY_SIZE;

    for (int i = 0; i < GYRO_HISTORY_SIZE; i++) {
        sum_x += gyro_x_history[i];
        sum_y += gyro_y_history[i];
        sum_z += gyro_z_history[i];
    }

    *gx = sum_x / GYRO_HISTORY_SIZE;
    *gy = sum_y / GYRO_HISTORY_SIZE;
    *gz = sum_z / GYRO_HISTORY_SIZE;

    
}

void icm42688_calibrate_gyro(i2c_inst_t *i2c, int num_samples) {
    //printf("Calibrating gyroscope...\n");
    float gx, gy, gz;
    
    for (int i = 0; i < num_samples; i++) {
        icm42688_read_gyro(i2c, &gx, &gy, &gz);
        gyro_bias_x += gx;
        gyro_bias_y += gy;
        gyro_bias_z += gz;
        sleep_ms(5);  // Adjust based on your IMU's ODR (Output Data Rate)
    }

    gyro_bias_x /= num_samples;
    gyro_bias_y /= num_samples;
    gyro_bias_z /= num_samples;

    ////printf("Gyroscope biases: gx=%.5f, gy=%.5f, gz=%.5f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

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
       //printf("I2C write failed with error code: %d\n", result);
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

void get_accel_cfg(i2c_inst_t *i2c, uint8_t *buf) {
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_ACCEL_CONFIG0, buf, 1);
}

void get_gyro_cfg(i2c_inst_t *i2c, uint8_t *buf) {
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_GYRO_CONFIG0, buf, 1);
}



void set_accel_FS(i2c_inst_t *i2c, uint8_t fs) {
    
    //get current config
    uint8_t reg;
    get_accel_cfg(i2c, &reg);
   //printf("Setting accel fs to %u fs was %u\n", fs, reg);
    //set new fs
    reg =  (fs << 5) | (reg & 0x1F) ;

   //printf("after shift%u\n", reg);
    uint8_t buf[] = {ICM42688_ACCEL_CONFIG0, reg};
    reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);

    accel_scale = ((1 << (4 - fs)))/32768.0f;
}

void set_gyro_FS(i2c_inst_t *i2c, uint8_t fs) {

    // Get current configuration of the GYRO_CONFIG0 register
    uint8_t reg;
    get_gyro_cfg(i2c, &reg);
    // Set the new full-scale (FS) value, preserving other bits
    reg = (fs << 5) | (reg & 0x1F);  // Clear FS bits (7:5) and set new FS value
    // Prepare buffer for writing
    uint8_t buf[] = {ICM42688_GYRO_CONFIG0, reg};
    // Write the modified register value back to GYRO_CONFIG0
    int result = reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);

    gyro_scale = (2000.0f / (1 << fs))/32768.0f;  // Update the gyro scale based on new FS value

}


void set_gyro_ODR(i2c_inst_t *i2c, uint8_t odr) {
    uint8_t reg;

    // Get the current value of the GYRO_CONFIG0 register
    get_gyro_cfg(i2c, &reg);
    // Mask the ODR bits (3:0) to preserve other bits, then set new ODR
    reg = (odr & 0x0F) | (reg & 0xF0);  // Clear ODR bits, then set new ODR

    // Prepare buffer for writing
    uint8_t buf[] = {ICM42688_GYRO_CONFIG0, reg};

    // Write the modified register value back to GYRO_CONFIG0
    reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);
}


void set_accel_ODR(i2c_inst_t *i2c, uint8_t odr) {
    uint8_t reg; // get current config
    get_accel_cfg(i2c, &reg);
    reg = odr | (reg & 0xF0); // only change the odr bits
    uint8_t buf[] = {ICM42688_ACCEL_CONFIG0, reg};
    reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);
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
   //printf("Initializing ICM-42688P...\n");

    // WHO AM I
    uint8_t id = icm42688_who_am_i(i2c);
    if (id != ICM42688_ID) {
       //printf("WHO_AM_I failed. Expected 0x49, got 0x%02X\n", id);
        return ; // Early return on error
    } else {
       //printf("WHO_AM_I successful. Device ID: 0x%02X\n", id);
    }


    // Reset the device
   //printf("Attempting to reset the device...\n");
    uint8_t buf[] = {ICM42688_SIGNAL_PATH_RESET, 0x80};
    int result = reg_write(i2c, ICM42688_I2C_L_ADDR, buf[0], &buf[1], 1);
    
    if (result < 0) {
       //printf("Failed to reset the device. Error code: %d\n", result);
        return; // Early return on error
    } else {
       //printf("Device reset successful.\n");
    }

    sleep_ms(100);  // Wait for reset
    uint8_t ODRCfg = odr200; // ODR 200hz
    uint8_t gyro_FS = dps1000;
    uint8_t accel_FS = gpm4;
   //printf("Configuring gyroscope: Full scale ±1000dps, ODR 200Hz...\n");
   //printf("Configuring accelerometer: Full scale ±4g, ODR 200Hz...\n");
    set_gyro_ODR(i2c, ODRCfg); // set speed at 200hz
    set_accel_ODR(i2c, ODRCfg);
    set_gyro_FS(i2c, gyro_FS); // set full scale at 1000dps
    set_accel_FS(i2c, accel_FS); //     4g
    
    //verify
    uint8_t gyro_cg;
    uint8_t accel_cg;
    get_gyro_cfg(i2c, &gyro_cg);
    get_accel_cfg(i2c, &accel_cg);
    if(gyro_cg != ((gyro_FS << 5) + ODRCfg)){
       //printf("Failed to set gyro config returned %u not %u\n", gyro_cg, (gyro_FS << 5) + ODRCfg);
    }
    else{
       //printf("Gyro success config set to %u\n", gyro_cg);
    }

    if(accel_cg != ((accel_FS << 5  )+ ODRCfg)){
       //printf("Failed to set accel config returned %u not %u\n", accel_cg, (accel_FS << 5 ) + ODRCfg);
    }
    else{
       //printf("Accel success config set to %u\n", accel_cg);
    }
    
    
    
    







   //printf("ICM-42688P initialized.\n");
}


void icm42688_read_accel(i2c_inst_t *i2c, float *ax, float *ay, float *az) {
    uint8_t rawData[6];
    // Read 6 bytes from the accelerometer x0, x1, y0, y1, z0, z1
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_ACCEL_DATA_X1, rawData, 6);

    // Scaling factor based on ±4g full-scale range


    *ax = (float)((int16_t)(rawData[0] << 8 | rawData[1])) * accel_scale;
    *ay = (float)((int16_t)(rawData[2] << 8 | rawData[3])) * accel_scale;
    *az = (float)((int16_t)(rawData[4] << 8 | rawData[5])) * accel_scale;

    ////printf("Accel: ax=%.2f, ay=%.2f, az=%.2f\n", *ax, *ay, *az);
}

void icm42688_read_gyro(i2c_inst_t *i2c, float *gx, float *gy, float *gz) {
    uint8_t rawData[6];
    // Read 6 bytes from the gyroscope x0, x1, y0, y1, z0, z1
    reg_read(i2c, ICM42688_I2C_L_ADDR, ICM42688_GYRO_DATA_X1, rawData, 6);

    // Scaling factor based on ±2000 dps full-scale range


    *gx = (float)((int16_t)(rawData[0] << 8 | rawData[1])) * gyro_scale;
    *gy = (float)((int16_t)(rawData[2] << 8 | rawData[3])) * gyro_scale;
    *gz = (float)((int16_t)(rawData[4] << 8 | rawData[5])) * gyro_scale;

    ////printf("Gyro: gx=%.2f, gy=%.2f, gz=%.2f\n", *gx, *gy, *gz);
}

void icm42688_read_gyro_corrected(i2c_inst_t *i2c, float *gx, float *gy, float *gz) {
    icm42688_read_gyro(i2c, gx, gy, gz);
    
    // Subtract the bias from the raw readings
    *gx -= gyro_bias_x;
    *gy -= gyro_bias_y;
    *gz -= gyro_bias_z;
    
    ////printf("Corrected Gyro: gx=%.2f, gy=%.2f, gz=%.2f\n", *gx, *gy, *gz);
}

void icm42688_read_gyro_average(i2c_inst_t *i2c, float *gx, float *gy, float *gz) {
    icm42688_read_gyro_corrected(i2c, gx, gy, gz);
    moving_average(gx, gy, gz);
}

