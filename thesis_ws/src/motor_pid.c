#include "motor_pid.h"

// PWM range constants
int PWM_MOTOR_MIN = 0;  // Minimum PWM value for motors
int PWM_MOTOR_MAX = 100;  // Maximum PWM value for motors


// Set PID controller gains
// This function can be used to change the PID gains during runtime
void pid_set_gains(PIDController* pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

// Update PID controller and compute control output
double pid_update(PIDController* pid, double input, double dt_ms) {
    double dt = dt_ms/1000.0f;
    double error = pid->setpoint - input;
    pid->error = error;
    pid->integral = pid->integral + error * dt;
    double derivative = (error - pid->previous_error) / dt;
    pid->derivative = derivative;

    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    if( output > PWM_MOTOR_MAX){
        output = PWM_MOTOR_MAX;
    }
    pid->output = output;
    pid->previous_error = error;
   
    return output;
}
// Set new setpoint for PID controller
void pid_set_setpoint(PIDController* pid, double setpoint) {
    pid->setpoint = setpoint;
    //pid_reset(pid);
}

// Reset PID controller state
void pid_reset(PIDController* pid) {
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->previous_error = 0.0;
}

// Initialize PID controller with given parameters
void pid_init(PIDController* pid, double Kp, double Ki, double Kd, double setpoint) {
    pid_set_gains(pid, Kp, Ki, Kd);
    pid_set_setpoint(pid, setpoint);
    pid_reset(pid);
}

void pid_set_message(PIDController* pid, control_msgs__msg__PidState* msg){
    msg->p_term = pid->Kp;
    msg->i_term = pid->Ki;
    msg->d_term = pid->Kd;
    msg->p_error = pid->error;
    msg->i_error = pid->integral;
    msg->d_error = pid->derivative;
    msg->error = pid->error;
    msg->error_dot = pid->previous_error;
    msg->output = pid->output;

}