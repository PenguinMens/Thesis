#include "motor_calcs.h"  // Include motor calculation functions and structures
#include <math.h>          // Include math functions
#include "motor_control.h" // Include motor control functions and structures

// Global variables struct for encoder initialization
encoder_setup_t encoder_setup = {
    .PULSES_PER_REV = 12400,       // Number of encoder pulses per revolution (default: 12400)
    .PULSES_PER_REV_GEAR = 3100,   // Number of encoder pulses per revolution with gear (default: 3100)
    .FRAME_TIME_MS = 100,          // Frame time in milliseconds (default: 100)
    .WHEEL_DIAMETER = 0.2,         // Diameter of the wheel in meters (default: 0.2)
    .WHEEL_BASE = 0.20             // Distance between wheels in meters (default: 0.20)
};

#define WINDOW_SIZE 10  // Size of the moving average window

// Variables for moving average calculation of motor velocities
int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;

int INDEX2 = 0;
float VALUE2 = 0;
float SUM2 = 0;
float READINGS2[WINDOW_SIZE];
float AVERAGED2 = 0;


// float calc_stats(float time, Odemtry_values *vals){
//     int32_t pulse_count_1 = get_encoder_count(ENCODER1);
//     int32_t pulse_count_2 = get_encoder_count(ENCODER2);
//     float rmp_1 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV *time );
//     float rmp_2 = (pulse_count_2 * 60 *1000) /  (encoder_setup.PULSES_PER_REV *time );
//     vals->linear_velocity = ((rmp_1  + rmp_2) * (M_PI * encoder_setup.WHEEL_DIAMETER)) / (2 * 60);
//     vals->angular_velocity = ((rmp_1 - rmp_2) * (M_PI * encoder_setup.WHEEL_DIAMETER)) / (encoder_setup.WHEEL_BASE * 60);
//     float rpm3 = (pulse_count_1 * 60 *1000 )/ (encoder_setup.PULSES_PER_REV_GEAR *time );
//     // x position calculation (m)
//     vals->x += vals->linear_velocity * cos(vals->theta) * time/1000.0f;
//     // y position calculation (m)
//     vals->y += vals->linear_velocity * sin(vals->theta) * time/1000.0f;
//     // theta position calculation (rad) frame time in ms
//     vals->theta += vals->angular_velocity * time/1000.0f;
//     reset_encoders();
//     return ((rpm3) * (M_PI * 2 * .025)) / ( 60);
// }

// Function to calculate motor statistics and update odometry values
void calc_stats(float time, Odometry_values *vals, int32_t ENCODER1_TICKS, int32_t ENCODER2_TICKS, MotorStats *motorStatsA, MotorStats *motorStatsB) {
    // Calculate rotations per minute (RPM) for each motor
    int32_t pulse_count_1 = ENCODER1_TICKS;
    int32_t pulse_count_2 = ENCODER2_TICKS;
    float rmp_1 = (pulse_count_1 * 60 * 1000) / (encoder_setup.PULSES_PER_REV * time);
    float rmp_2 = (pulse_count_2 * 60 * 1000) / (encoder_setup.PULSES_PER_REV * time);

    // Update motor statistics with calculated RPM values
    motorStatsA->rps = rmp_1;
    motorStatsB->rps = rmp_2;

    // Calculate linear velocity for each motor

    float v1 = (rmp_1 * 0.10472f * encoder_setup.WHEEL_DIAMETER);
    float v2 = (rmp_2 * 0.10472f * encoder_setup.WHEEL_DIAMETER);

    // Moving average calculation for motor velocities
    // SUM = SUM - READINGS[INDEX];        // Remove the oldest entry from the sum
    // VALUE = v1;                         // Read the next sensor value
    // READINGS[INDEX] = VALUE;            // Add the newest reading to the window
    // SUM = SUM + VALUE;                  // Add the newest reading to the sum
    // INDEX = (INDEX + 1) % WINDOW_SIZE;  // Increment the index, and wrap to 0 if it exceeds the window size
    //AVERAGED = SUM / WINDOW_SIZE;       // Calculate the average velocity
    AVERAGED = v1;

    // SUM2 = SUM2 - READINGS2[INDEX2];       // Remove the oldest entry from the sum
    // VALUE2 = v2;                           // Read the next sensor value
    // READINGS2[INDEX2] = VALUE2;            // Add the newest reading to the window
    // SUM2 = SUM2 + VALUE2;                  // Add the newest reading to the sum
    // INDEX2 = (INDEX2 + 1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size
    // //AVERAGED2 = SUM2 / WINDOW_SIZE;        // Calculate the average velocity
    AVERAGED2 = v2;

    // Update motor statistics with averaged velocity values
    motorStatsA->velocity = AVERAGED;
    motorStatsB->velocity = AVERAGED2;

    // Calculate linear and angular velocities for odometry
    vals->linear_velocity = (( v1  + v2) ) / (2.0f);
    vals->angular_velocity = ((v2 - v1  ) ) / (encoder_setup.WHEEL_BASE);


    // Calculate new position and orientation based on velocities
    // vals->x += vals->linear_velocity * cos(vals->theta) * time / 1000.0f;
    // vals->y += vals->linear_velocity * sin(vals->theta) * time / 1000.0f;
    // vals->theta += vals->angular_velocity * time / 1000.0f;
    vals->x = 0;
    vals->y = 0;
    vals->theta =0;
}
