#include <stdio.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Custom motor control includes
#include "motor_control.h"
#include "encoder.h"
#include "motor.h"
#include "motor_calcs.h"
#include "icm42688.h"
#define PWM_MAX 50.0f
#define ROSMODE 0

#include "tusb.h"  // TinyUSB header for USB CDC support

#define I2C_FREQUENCY 100 * 1000  // I2C frequency set to 100kHz
#define CALIBRATION_SAMPLES 2000  // Number of samples for calibration
#define LOOP_DELAY_MS 10000        // Loop delay in milliseconds

void init_i2c()
{
    i2c_init(i2c_default, I2C_FREQUENCY);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void display_header(const char* title)
{
    printf("\n===============================\n");
    printf("    %s\n", title);
    printf("===============================\n");
}

void read_and_display_data(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint32_t curr_time = to_ms_since_boot(get_absolute_time());
    icm42688_read_accel(i2c_default, ax, ay, az);
    icm42688_read_gyro(i2c_default, gx, gy, gz);
    float temp;
    icm42688_get_temperature(i2c_default, &temp);

    printf("A: %.4f %.4f %.4f G: %.4f %.4f %.4f %u %.4f\n", *ax, *ay, *az,  *gx, *gy, *gz, curr_time, temp);

}

void read_and_display_data_calibrated(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint32_t curr_time = to_ms_since_boot(get_absolute_time());
    icm42688_read_average(i2c_default,ax,ay,az, gx, gy, gz);
    float temp;
    icm42688_get_temperature(i2c_default, &temp);

    printf("A: %.4f %.4f %.4f G: %.4f %.4f %.4f %u %.4f\n", *ax, *ay, *az,  *gx, *gy, *gz, curr_time, temp);

}

void read_and_display_data_average(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint32_t curr_time = to_ms_since_boot(get_absolute_time());
    icm42688_read_average(i2c_default,ax,ay,az, gx, gy, gz);
    float temp;
    icm42688_get_temperature(i2c_default, &temp);

    printf("A: %.4f %.4f %.4f G: %.4f %.4f %.4f %u %.4f\n", *ax, *ay, *az,  *gx, *gy, *gz, curr_time, temp);

}

void continuous_read_imu(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    printf("Continuous IMU reading started. Press 'x' to stop.\n");
    while (true) {
      
        int stop = getchar_timeout_us(0);  // Non-blocking check for the stop signal
        if (stop == 'x' || stop == 'X') {
            printf("Stopping continuous IMU reading.\n");
            break;
        }
        
        // Read and display IMU data
        read_and_display_data(ax, ay, az, gx, gy, gz);
        
        sleep_ms(5);  // Adjust this delay based on how often you want to print data
    }
}



void continuous_read_imu_calibrated(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    printf("Continuous IMU reading started. Press 'x' to stop.\n");
    while (true) {
      
        int stop = getchar_timeout_us(0);  // Non-blocking check for the stop signal
        if (stop == 'x' || stop == 'X') {
            printf("Stopping continuous IMU reading.\n");
            break;
        }
        
        // Read and display IMU data
        read_and_display_data_calibrated(ax, ay, az, gx, gy, gz);
        
        sleep_ms(5);  // Adjust this delay based on how often you want to print data
    }
}

void continuous_read_imu_average(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    printf("Continuous IMU reading started. Press 'x' to stop.\n");
    while (true) {
      
        int stop = getchar_timeout_us(0);  // Non-blocking check for the stop signal
        if (stop == 'x' || stop == 'X') {
            printf("Stopping continuous IMU reading.\n");
            break;
        }
        
        // Read and display IMU data
        read_and_display_data_average(ax, ay, az, gx, gy, gz);
        
        sleep_ms(5);  // Adjust this delay based on how often you want to print data
    }
}

int main()
{
    stdio_init_all(); // Initialize all configured stdio types
    sleep_ms(2000);   // Allow some time for USB connection initialization

    display_header("Initializing I2C and IMU");
    init_i2c();  // Initialize I2C
    icm42688_init(i2c_default);  // Initialize IMU
    icm42688_reset(i2c_default); // Reset IMU

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

    // display_header("IMU Test and Calibration");

    printf("Starting IMU calibration with %d samples...\n", CALIBRATION_SAMPLES);
    icm42688_calibrate(i2c_default, CALIBRATION_SAMPLES);
    printf("Calibration complete.\n\n");
    int counter = 0;
    while (true)
    {
        // Use non-blocking input with timeout
        int ch = getchar_timeout_us(0);  // Returns PICO_ERROR_TIMEOUT if no character is available

        if (ch != PICO_ERROR_TIMEOUT)
        {
            if (ch == 's' || ch == 'S') // Start reading data once
            {
                display_header("Reading Sensor Data Before Calibration");
                read_and_display_data(&ax, &ay, &az, &gx, &gy, &gz);
                
                
                printf("\nApplying Gyro Calibration...\n");
                icm42688_calibrate(i2c_default, CALIBRATION_SAMPLES);
                

                display_header("Reading Sensor Data After Calibration");
                read_and_display_data_calibrated(&ax, &ay, &az, &gx, &gy, &gz);
                
                printf("Calibration applied successfully.\n");
            }
            else if (ch == 'c' || ch == 'C')  // Start continuous reading
            {
                display_header("Starting Continuous IMU Data Read");
                continuous_read_imu(&ax, &ay, &az, &gx, &gy, &gz);
            }
            else if (ch == 'b' || ch == 'B')  // Start continuous reading
            {
                display_header("Starting Continuous IMU Data Read");
                continuous_read_imu_calibrated(&ax, &ay, &az, &gx, &gy, &gz);
            }
            else if (ch == 'a' || ch == 'A')  // Start continuous reading
            {
                display_header("Starting Continuous IMU Data Read");
                continuous_read_imu_average(&ax, &ay, &az, &gx, &gy, &gz);
            }
            else if (ch == 'q' || ch == 'Q')  // Quit the program
            {
                printf("\nExiting program.\n");
                break;
            }
        }
        else
        {
            if(counter == 20)
            {
                printf("Waiting for user input (press 'S' to start, 'C' for continuous, 'Q' to quit)...\n");
                uint8_t i = -1;
                get_accel_cfg(i2c_default, &i);
                printf("fsTest%d\n", i);
                get_gyro_cfg(i2c_default, &i);
                printf("fsTest%d\n", i);
                counter = 0;
            }
            else
            {
                counter++;
            }
           
        }

        sleep_ms(1000); // Delay to prevent the loop from running too fast
    }

    return 0;
}
