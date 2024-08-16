#include "motor_calcs.h"  // Include motor calculation functions and structures
#include <math.h>          // Include math functions
#include "motor_control.h" // Include motor control functions and structures

// Global variables struct for encoder initialization
encoder_setup_t encoder_setup = {
    .PULSES_PER_REV = 12400,       // Number of encoder pulses per revolution (default: 12400)
    .PULSES_PER_REV_GEAR = 3120,   // Number of encoder pulses per revolution with gear (default: 3100)      // Frame time in milliseconds (default: 100)
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
    // Start of function debug
    // - printf("calc_stats called with:\n");
    // - printf("    time: %f\n", time);
    // - printf("    ENCODER1_TICKS: %d    ENCODER2_TICKS: %d\n", ENCODER1_TICKS, ENCODER2_TICKS);
    // - printf("    Last angular_pos_1: %f    Last angular_pos_2: %f\n", motorStatsA->last_angular_position, motorStatsB->last_angular_position);
    // - printf("    Last angular_vel_1: %f    Last angular_vel_2: %f\n", motorStatsA->angular_velocity, motorStatsB->angular_velocity);

    // Calculate the current angular position (in radians) of each wheel
    float angular_pos_1 = (float)ENCODER1_TICKS * (2.0f * M_PI / encoder_setup.PULSES_PER_REV_GEAR);
    float angular_pos_2 = (float)ENCODER2_TICKS * (2.0f * M_PI / encoder_setup.PULSES_PER_REV_GEAR);

    // - printf("    Calculated angular_pos_1: %f    Calculated angular_pos_2: %f\n", angular_pos_1, angular_pos_2);

    // Calculate the angular velocity for each wheel in rad/s
    float angular_vel_1 = (angular_pos_1 - motorStatsA->last_angular_position) / time;
    float angular_vel_2 = (angular_pos_2 - motorStatsB->last_angular_position) / time;

    // - printf("    Calculated angular_vel_1: %f rad/s    Calculated angular_vel_2: %f rad/s\n", angular_vel_1, angular_vel_2);

    // Calculate RPM for each wheel
    float rpm_1 = (angular_vel_1 * 60.0f) / (2.0f * M_PI);
    float rpm_2 = (angular_vel_2 * 60.0f) / (2.0f * M_PI);

    // - printf("    Calculated RPM_1: %f RPM    Calculated RPM_2: %f RPM\n", rpm_1, rpm_2);

    // Update motor statistics with the current position, velocity, and RPM
    motorStatsA->last_angular_position = angular_pos_1;
    motorStatsB->last_angular_position = angular_pos_2;
    motorStatsA->angular_velocity = angular_vel_1;
    motorStatsB->angular_velocity = angular_vel_2;
    motorStatsA->rpm = rpm_1;
    motorStatsB->rpm = rpm_2;

    // - printf("    Updated last_angular_position pos_1: %f    pos_2: %f\n", motorStatsA->last_angular_position, motorStatsB->last_angular_position);
    // - printf("    Updated angular_velocity vel_1: %f rad/s    vel_2: %f rad/s\n", motorStatsA->angular_velocity, motorStatsB->angular_velocity);
    // - printf("    Updated RPM_1: %f RPM    RPM_2: %f RPM\n", motorStatsA->rpm, motorStatsB->rpm);

    // Calculate linear and angular velocities for odometry
    float linear_velocity_1 = angular_vel_1 * encoder_setup.WHEEL_DIAMETER/2;
    float linear_velocity_2 = angular_vel_2 * encoder_setup.WHEEL_DIAMETER/2;

    vals->linear_velocity = (linear_velocity_1 + linear_velocity_2) / 2.0f;
    vals->angular_velocity = (linear_velocity_2 - linear_velocity_1) / encoder_setup.WHEEL_BASE;

    // - printf("    Calculated linear_velocity: %f m/s    angular_velocity: %f rad/s\n", vals->linear_velocity, vals->angular_velocity);

    // Update odometry (x, y, theta) if needed
    vals->x += vals->linear_velocity * cos(vals->theta) * time;
    vals->y += vals->linear_velocity * sin(vals->theta) * time;
    vals->theta += vals->angular_velocity * time;

    // - printf("    Updated odometry x: %f    y: %f    theta: %f\n", vals->x, vals->y, vals->theta);

    // End of function debug
    // - printf("calc_stats completed.\n");
}
