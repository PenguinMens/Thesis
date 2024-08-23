#ifndef MOTOR_H
#define MOTOR_H
#include "pico/stdlib.h"    // Include Pico standard library functions
#include "motor_pid.h"
#include "motor_calcs.h"
#include "utils.h"       // Include utility functions
#include "custom_pwm.h"  // Include custom PWM functions
// Define any necessary constants or macros here

#define breakout 0

#if breakout

#define MOTOR1_PWM 15
#define MOTOR1_IN1 17
#define MOTOR1_IN2 16
#define MOTOR1_ENCODER 12

/*### MOTOR 2(right?) pin assignment   */
#define MOTOR2_PWM 14
#define MOTOR2_IN1 19
#define MOTOR2_IN2 18
#define MOTOR2_ENCODER 20

#else  // pico
/*### MOTOR 1(left?) pin assignment   */
#define MOTOR1_PWM 15
#define MOTOR1_IN1 16
#define MOTOR1_IN2 17
#define MOTOR1_ENCODER 20

/*### MOTOR 2(right?) pin assignment   */
#define MOTOR2_PWM 14
#define MOTOR2_IN1 18
#define MOTOR2_IN2 19
#define MOTOR2_ENCODER 12 
#endif // breakout








// // Structure to hold motor pins and PWM values
// @@@@ OLD CODE might still be worth seperating mins from motor
// typedef struct MotorPins {
    //  unsigned int PWM_PIN;  // Pin for PWM signal
    // unsigned int CHANNEL;  // PWM channel (shared with slice)
    // unsigned int SLICE;    // PWM slice (specific to Pico)
    // unsigned int IN1_PIN;  // Motor input pin 1
    // unsigned int IN2_PIN;  // Motor input pin 2
    // unsigned int ENCODER_PINS;
// } MotorPins;

typedef struct Motor {
    unsigned int PWM_PIN;  // Pin for PWM signal
    unsigned int CHANNEL;  // PWM channel (shared with slice)
    unsigned int SLICE;    // PWM slice (specific to Pico)
    unsigned int IN1_PIN;  // Motor input pin 1
    unsigned int IN2_PIN;  // Motor input pin 2
    unsigned int ENCODER_PINS;
    MotorStats motorStats;
} Motor;
/**
 * Initializes the motor pins and PWM settings.
 * Parameters:
 *  motor: Motor structure containing motor pins and PWM settings.
 * PWM: Pin for PWM signal
 * IN1: Motor input pin 1
 * IN2: Motor input pin 2
 * ENCODER_PINS: Motor encoder pins
 * kp: Proportional gain for PID controller
 * ki: Integral gain for PID controller
 * kd: Derivative gain for PID controller
 * setpoint: Setpoint for PID controller
 */
void init_motor(Motor *motor, uint PWM, uint IN1, uint IN2, uint ENCODER_PINS, float kp, float ki, float kd, float setpoint);
// Declare function prototypes


#endif // MOTOR_H
