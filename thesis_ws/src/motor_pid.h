#ifndef MOTOR_PID_H
#define MOTOR_PID_H

// Include any necessary libraries or headers here
#include <stdint.h>  // Example: Include standard integer types if needed
#include <control_msgs/msg/pid_state.h>

// Define any constants here
#define PID_TOLERANCE 1e-6  // Example: Define a small tolerance value

// Declare the PIDController structure
typedef struct {
    double Kp;           // Proportional gain
    double Ki;           // Integral gain
    double Kd;           // Derivative gain
    double setpoint;     // Desired setpoint
    double error;        // Current error
    double integral;     // Integral term
    double derivative;   // Derivative term
    double previous_error;  // Previous error for derivative term calculation
    double output;
} PIDController;

// Declare function prototypes

/**
 * @brief Initialize the PID controller with the given parameters.
 * 
 * @param pid Pointer to the PIDController structure to initialize.
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 * @param setpoint Desired setpoint. 
 */
void pid_init(PIDController* pid, double Kp, double Ki, double Kd, double setpoint);

/**
 * @brief Update the PID controller and compute the control output.
 * 
 * @param pid Pointer to the PIDController structure.
 * @param input Current input value.
 * @param dt Time step (in seconds) since the last update.
 * @return Control output.
 */
double pid_update(PIDController* pid, double input, double dt);

/**
 * @brief Set a new setpoint for the PID controller.
 * 
 * @param pid Pointer to the PIDController structure.
 * @param setpoint New desired setpoint.
 */
void pid_set_setpoint(PIDController* pid, double setpoint);

/**
 * @brief Reset the state of the PID controller.
 * 
 * @param pid Pointer to the PIDController structure.
 */
void pid_reset(PIDController* pid);

void pid_set_message(PIDController* pid, control_msgs__msg__PidState* msg);
#endif // MOTOR_PID_H
