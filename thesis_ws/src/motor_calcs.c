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

void calc_stats(float time, Odometry_values *vals, int32_t ENCODER1_TICKS, int32_t ENCODER2_TICKS, MotorStats *motorStatsA, MotorStats *motorStatsB) {
    // Calculate the current position (angle) of each wheel
    float pos_1 = (float)ENCODER1_TICKS * (2 * (22/7) / encoder_setup.PULSES_PER_REV);
    float pos_2 = (float)ENCODER2_TICKS * (2 * (22/7) / encoder_setup.PULSES_PER_REV);

    // Calculate the velocity for each wheel
    float vel_1 = (pos_1 - motorStatsA->last_position) / time;
    float vel_2 = (pos_2 - motorStatsB->last_position) / time;

    // Update motor statistics with the current position and velocity
    motorStatsA->last_position = pos_1;
    motorStatsB->last_position = pos_2;
    motorStatsA->velocity = vel_1;
    motorStatsB->velocity = vel_2;

    // Calculate linear and angular velocities for odometry
    vals->linear_velocity = (vel_1 * encoder_setup.WHEEL_DIAMETER / 2.0f + vel_2 * encoder_setup.WHEEL_DIAMETER / 2.0f) / 2.0f;
    vals->angular_velocity = (vel_2 * encoder_setup.WHEEL_DIAMETER / 2.0f - vel_1 * encoder_setup.WHEEL_DIAMETER / 2.0f) / encoder_setup.WHEEL_BASE;

    // Update odometry (x, y, theta) if needed
    vals->x += vals->linear_velocity * cos(vals->theta) * time;
    vals->y += vals->linear_velocity * sin(vals->theta) * time;
    vals->theta += vals->angular_velocity * time;
}
