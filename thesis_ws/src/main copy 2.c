#include <math.h>
#include <inttypes.h>
#include <stdio.h>
// ROS 2
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/vector3.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Custom motor control includes
#include "motor_control.h"
#include "encoder.h"
#include "motor.h"
#include "motor_calcs.h"

// Define constants
#define PWM_MAX 50.0f
#define ROS_MODE 1

// Global Variables
rcl_publisher_t left_enc_publisher, right_enc_publisher,test_publisher, publisher_pid;
rcl_subscription_t left_wheel_cmd_subscriber, right_wheel_cmd_subscriber, pid_terms_subscriber;
std_msgs__msg__Int32 left_enc_msg, right_enc_msg;
std_msgs__msg__Float32 test_msg;
geometry_msgs__msg__Vector3 left_wheel_cmd_msg, right_wheel_cmd_msg;

control_msgs__msg__PidState pid_msg ,pid_in;

rcl_timer_t timer, timer2;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

Motor leftMotor, rightMotor;
Odometry_values odo_vals;

// Encoder pins
const uint ENCODERA = 0;
const uint ENCODERB = 1;


// Function prototypes
void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void pid_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void pid_callback(const void *msgin);
void left_wheel_cmd_callback(const void * msgin);
void right_wheel_cmd_callback(const void * msgin);


void motor_iteration(int32_t dt);




int main(){
    stdio_init_all(); // Initialize all configured stdio types

    // Initialize motors
    float kp = 10, ki = 1, kd = 0;
    init_motor(&leftMotor, MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENCODER, kp, ki, kd, 0);
    init_motor(&rightMotor, MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCODER, kp, ki, kd, 0);
    
    // Initialize encoders
    init_PIO_encoder(MOTOR1_ENCODER, MOTOR2_ENCODER, ENCODERA, ENCODERB);

    // ROS 2 Initialization
    #if ROS_MODE
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
        );
        allocator = rcl_get_default_allocator();

        const int timeout_ms = 1000;
        const uint8_t attempts = 120;

        rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
        if (ret != RCL_RET_OK){
            // Unreachable agent, exiting program.
            return ret;
        }

        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "pico_node", "", &support);

        // Initialize publishers for encoder values
        rclc_publisher_init_default(
            &left_enc_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_wheel_enc"
        );

        rclc_publisher_init_default(
            &right_enc_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_wheel_enc"
        );

        rclc_publisher_init_default(
            &test_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "test"
        );

        // Initialize subscribers for wheel commands
        rclc_subscription_init_best_effort(
            &left_wheel_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "/left_wheel_cmd"
        );  

        rclc_subscription_init_best_effort(
            &right_wheel_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "/right_wheel_cmd"
        );


        rclc_publisher_init_default(
            &publisher_pid,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs,msg, PidState),
            "pico_pid");

        rclc_subscription_init_best_effort(
            &pid_terms_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs,msg, PidState),
            "/pid_terms");



        // Initialize timer for encoder publishing
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(100),
            encoder_timer_callback
        );



        rclc_timer_init_default(
            &timer2,
            &support,
            RCL_MS_TO_NS(100),
            pid_timer_callback
        );
        // Initialize executor and add handles
        rclc_executor_init(&executor, &support.context, 5, &allocator);
        rclc_executor_add_timer(&executor, &timer);
        rclc_executor_add_timer(&executor, &timer2);
        rclc_executor_add_subscription(&executor, &left_wheel_cmd_subscriber, &left_wheel_cmd_msg, &left_wheel_cmd_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &right_wheel_cmd_subscriber, &right_wheel_cmd_msg, &right_wheel_cmd_callback, ON_NEW_DATA);
        if (rclc_executor_add_subscription(&executor, &pid_terms_subscriber, &pid_msg, &pid_callback, ON_NEW_DATA) != RCL_RET_OK)
        {
            //test_msg.data = -1.0f;
        }else
        {
            //test_msg.data = 1.0f;
        }

        
    #else
        leftMotor.motorStats.pid.setpoint = 1.0f;
    #endif
    
    // Main lo
    while (true) {
        #if ROS_MODE
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        #else

            calc_stats(1, &odo_vals, get_encoder_count_A(), get_encoder_count_B(), &leftMotor.motorStats, &rightMotor.motorStats);
            // After updating motor statistics
            printf("    Updated angular_pos_1: %f    angular_pos_2: %f\n", leftMotor.motorStats.last_angular_position, rightMotor.motorStats.last_angular_position);    
            printf("    Updated angular_vel_1: %f rad/s    angular_vel_2: %f rad/s\n", leftMotor.motorStats.angular_velocity, rightMotor.motorStats.angular_velocity);
            printf("    RPM 1: %f    RPM 2: %f\n", leftMotor.motorStats.rpm, rightMotor.motorStats.rpm);

            motor_iteration(1000);
            // printf("LEFT MOTOR COUNT: %d \nRIGHT MOTOR COUNT %d \n",get_encoder_count_A(), get_encoder_count_B() );
            sleep_ms(1000);
        #endif
    }

    return 0;
}

// Callback for the encoder timer
void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time){    
    left_enc_msg.data = get_encoder_count_A();
    right_enc_msg.data = get_encoder_count_B();

    // rcl_publish(&left_enc_publisher, &left_enc_msg, NULL);
    // rcl_publish(&right_enc_publisher, &right_enc_msg, NULL);


    calc_stats(0.1, &odo_vals, get_encoder_count_A(), get_encoder_count_B(), &leftMotor.motorStats, &rightMotor.motorStats);


    pid_in.p_term = leftMotor.motorStats.pid.Kp;
    pid_in.i_term = leftMotor.motorStats.pid.Ki;
    pid_in.d_term = leftMotor.motorStats.pid.Kd;
    pid_in.error = leftMotor.motorStats.pid.error;
    pid_in.output = leftMotor.motorStats.pid.output;
    pid_in.i_max = leftMotor.motorStats.pid.setpoint;
    

    
    
    rcl_publish(&publisher_pid, &pid_in, NULL);
    //test_msg.data = leftMotor.motorStats.angular_velocity;
    //rcl_publish(&test_publisher, &test_msg, NULL);

}

void pid_callback(const void *msgin) {
    const control_msgs__msg__PidState * msg = (const control_msgs__msg__PidState *)msgin;

    // Update the PID values for both motors
    pid_set_gains(&leftMotor.motorStats.pid, msg->p_term, msg->i_term, msg->d_term);
    pid_set_gains(&rightMotor.motorStats.pid, msg->p_term, msg->i_term, msg->d_term);

}


void motor_iteration(int32_t dt)
{


    double outputA = pid_update(&leftMotor.motorStats.pid, leftMotor.motorStats.angular_velocity, dt);
    double outputB = pid_update(&rightMotor.motorStats.pid, rightMotor.motorStats.angular_velocity, dt);
    
    float pwmA = fabs(outputA);
    float pwmB = fabs(outputB);
    test_msg.data =                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
    rcl_publish(&test_publisher, &test_msg, NULL);


    if (pwmA > PWM_MAX) {
        pwmA = PWM_MAX;
    }
    if (pwmB > PWM_MAX) {
        pwmB = PWM_MAX;
    }


    control_motor(rightMotor, outputB, pwmB);
    control_motor(leftMotor, outputA, pwmA);


    rightMotor.motorStats.PWM = outputB;
    leftMotor.motorStats.PWM = outputA;

}
// PID loop timer callback
void pid_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    motor_iteration(100);

}

// Callback for left wheel command subscriber
void left_wheel_cmd_callback(const void * msgin){
    

    // Control left motor based on the received command
    pid_set_setpoint(&rightMotor.motorStats.pid, left_wheel_cmd_msg.x);
}

// Callback for right wheel command subscriber
void right_wheel_cmd_callback(const void * msgin){
    const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;    

    // Control right motor based on the received command
    pid_set_setpoint(&rightMotor.motorStats.pid, msg->x);
}

