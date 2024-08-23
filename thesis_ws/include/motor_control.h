/** motor_control.h */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "utils.h"       // Include utility functions
#include "custom_pwm.h"  // Include custom PWM functions
#include "motor.h"       // Include motor control functions and structures
/**
 * Explanation of motor_control.h:
 * This header file defines functions and structures related to motor control.
 * It provides an interface for initializing motors and controlling their speed and direction.
 */


/**
 * Controls the left motor direction and PWM value.
 * Parameters:
 *   motor: Motor structure containing motor pins and PWM settings.   
 *   dir: Direction of the motor rotation (positive or negative).
 *   pwm: PWM value for controlling motor speed.
 * Returns:
 *   Returns the status of the motor control operation.
 */
int control_motor(Motor motor, float dir, float pwm);



#endif // MOTOR_CONTROL_H
