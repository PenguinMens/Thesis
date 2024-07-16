#include "motor_control.h"  // Include motor control functions and structures
#include <math.h>           // Include math functions
#include "pico/stdlib.h"    // Include Pico standard library functions
#include "motor.h"
// Constants for GPIO
#define HIGH 1
#define LOW 0
void set_motor_dir(float speed, Motor motor); 
// Define Motor Pins structure


// Function to initialize motors

// // Function to control right motor
// int controlRightMotor(float dir, float pwm) {
//     set_motor_dir(dir, motorA);  // Set direction of Motor A
//     pwm_set_freq_duty(motorA.SLICE, motorA.CHANNEL, 50000, pwm);  // Set PWM duty cycle for Motor A
//      printf("RIGHT MOTORO %f,%f\n", dir,pwm);
// }

// // Function to control left motor
// int controlLeftMotor(float dir, float pwm) {
//     set_motor_dir(dir, motorB);  // Set direction of Motor B
//     pwm_set_freq_duty(motorB.SLICE, motorB.CHANNEL, 50000, pwm);  // Set PWM duty cycle for Motor B
//      printf("LEFT MOTORO %f,%f\n", dir,pwm);
// }

// Function to control left motor
int control_motor(Motor motor, float dir, float pwm) {
    set_motor_dir(dir, motor);  // Set direction of Motor B
    pwm_set_freq_duty(motor.SLICE, motor.CHANNEL, 50000, pwm);  // Set PWM duty cycle for Motor B
}

// Function to set motor direction based on speed
void set_motor_dir(float speed, Motor motor) {

    if (speed < 0.000001f && speed > -0.000001f) {
        gpio_put(motor.IN1_PIN, HIGH);  // Set input 1 of motor to HIGH
        gpio_put(motor.IN2_PIN, HIGH);  // Set input 2 of motor to HIGH
        printf("STOP\n");
    } else if (speed > 0.000001f) {
        gpio_put(motor.IN1_PIN, HIGH);  // Set input 1 of motor to HIGH
        gpio_put(motor.IN2_PIN, LOW);   // Set input 2 of motor to LOW
    } else {
        gpio_put(motor.IN1_PIN, LOW);   // Set input 1 of motor to LOW
        gpio_put(motor.IN2_PIN, HIGH);  // Set input 2 of motor to HIGH
    }
}
