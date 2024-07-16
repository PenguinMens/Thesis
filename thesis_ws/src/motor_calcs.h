// encoder_init.h

#ifndef MOTOR_CALCS_H
#define MOTOR_CALCS_H

#include <stdint.h>
#include "motor_pid.h" // Include header file for motor PID control

// Function declarations

// TODO: You might want to add a copyright notice here

// Structure to hold encoder setup parameters
typedef struct {
    int RESOLUTION;
    int PULSES_PER_REV;
    int PULSES_PER_REV_GEAR;
    int FRAME_TIME_MS;
    float WHEEL_DIAMETER;
    float WHEEL_BASE;
} encoder_setup_t;

// Structure to hold odometry values
typedef struct {
    float x;                  // X-coordinate of the car's position (in meters)
    float y;                  // Y-coordinate of the car's position (in meters)
    float theta;              // Orientation of the car (in radians)
    float linear_velocity;    // Linear velocity of the motor (in m/s)
    float angular_velocity;   // Angular velocity of the car (in rad/s)
} Odometry_values;

// Structure to hold motor statistics
typedef struct MotorStats {
    float velocity;           // Current velocity of the motor
    float rps;                // Rotations per second of the motor
    float PWM;           // PWM value for motor control
    PIDController pid;        // PID controller for motor control
} MotorStats;

// Function prototype to calculate motor statistics
void calc_stats(float time, Odometry_values *vals, int32_t ENCODER1_TICKS, int32_t ENCODER2_TICKS, MotorStats *motorStatsA, MotorStats *motorStatsB);

#endif // ENCODER_INIT_H
