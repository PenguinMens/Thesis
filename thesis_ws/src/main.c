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
#define ROS_MODE 0

// Global Variables
rcl_publisher_t left_enc_publisher, right_enc_publisher,test_publisher;
rcl_subscription_t left_wheel_cmd_subscriber, right_wheel_cmd_subscriber;

std_msgs__msg__Int32 left_enc_msg, right_enc_msg;
std_msgs__msg__Float32 left_wheel_cmd_msg, right_wheel_cmd_msg, test_msg;

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
void left_wheel_cmd_callback(const void * msgin);
void right_wheel_cmd_callback(const void * msgin);



int main(){
    stdio_init_all(); // Initialize all configured stdio types

    // Initialize motors
    float kp = 0, ki = 0, kd = 0;
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
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/left_wheel_cmd"
        );  

        rclc_subscription_init_best_effort(
            &right_wheel_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/right_wheel_cmd"
        );

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
        rclc_executor_init(&executor, &support.context, 6, &allocator);
        rclc_executor_add_timer(&executor, &timer);
        rclc_executor_add_timer(&executor, &timer2);
        rclc_executor_add_subscription(&executor, &left_wheel_cmd_subscriber, &left_wheel_cmd_msg, &left_wheel_cmd_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &right_wheel_cmd_subscriber, &right_wheel_cmd_msg, &right_wheel_cmd_callback, ON_NEW_DATA);

    #endif
    leftMotor.motorStats.pid.setpoint = 0.5;
    // Main lo
    while (true) {
        #if ROS_MODE
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        #else

            calc_stats(1, &odo_vals, get_encoder_count_A(), get_encoder_count_B(), &leftMotor.motorStats, &rightMotor.motorStats);
            // After updating motor statistics
            printf("    Updated angular_pos_1: %f    angular_pos_2: %f\n", leftMotor.motorStats.last_angular_position, rightMotor.motorStats.last_angular_position);    
            printf("    Updated angular_vel_1: %f rad/s    angular_vel_2: %f rad/s\n", leftMotor.motorStats.angular_velocity, rightMotor.motorStats.angular_velocity);
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

    rcl_publish(&left_enc_publisher, &left_enc_msg, NULL);
    rcl_publish(&right_enc_publisher, &right_enc_msg, NULL);


    calc_stats(0.1, &odo_vals, get_encoder_count_A(), get_encoder_count_B(), &leftMotor.motorStats, &rightMotor.motorStats);
    test_msg.data = leftMotor.motorStats.angular_velocity;
    rcl_publish(&test_publisher, &test_msg, NULL);

}

void motor_iteration(int32_t dt)
{
        // Example PID control code
    // Assuming you have a control_motor_PID function

    // Print the last call time for debugging
   // printf("PID Timer Callback: last_call_time = %" PRIu64 "\n", last_call_time);

    calc_stats(1000, &odo_vals, get_encoder_count_A(), get_encoder_count_B(), &leftMotor.motorStats, &rightMotor.motorStats);

    // Print the updated angular velocities after calling calc_stats
    printf("After calc_stats:\n");
    printf("    leftMotor angular_velocity: %f rad/s\n", leftMotor.motorStats.angular_velocity);
    printf("    rightMotor angular_velocity: %f rad/s\n", rightMotor.motorStats.angular_velocity);

    double outputA = pid_update(&leftMotor.motorStats.pid, leftMotor.motorStats.angular_velocity, dt);
    double outputB = pid_update(&rightMotor.motorStats.pid, rightMotor.motorStats.angular_velocity, dt);

    // Print the PID output values
    printf("PID Outputs:\n");
    printf("    leftMotor PID output: %f\n", outputA);
    printf("    rightMotor PID output: %f\n", outputB);

    float pwmA = fabs(outputA);
    float pwmB = fabs(outputB);

    // Print the PWM values before limiting
    printf("PWM Values before limiting:\n");
    printf("    leftMotor PWM: %f\n", pwmA);
    printf("    rightMotor PWM: %f\n", pwmB);

    if (pwmA > PWM_MAX) {
        pwmA = PWM_MAX;
    }
    if (pwmB > PWM_MAX) {
        pwmB = PWM_MAX;
    }

    // Print the PWM values after limiting
    printf("PWM Values after limiting:\n");
    printf("    leftMotor PWM: %f\n", pwmA);
    printf("    rightMotor PWM: %f\n", pwmB);

    control_motor(rightMotor, outputB, pwmB);
    control_motor(leftMotor, outputA, pwmA);

    // Print the final PWM values being applied to the motors
    printf("Final PWM Values applied:\n");
    printf("    leftMotor PWM: %f\n", pwmA);
    printf("    rightMotor PWM: %f\n", pwmB);

    rightMotor.motorStats.PWM = outputB;
    leftMotor.motorStats.PWM = outputA;

    // // Debug statement for test publishing
    // test_msg.data = 1.0f;
    // printf("Test message data set to %f\n", test_msg.data);
    //rcl_publish(&test_publisher, &test_msg, NULL);
}
// PID loop timer callback
void pid_timer_callback(rcl_timer_t * timer, int64_t last_call_time){


}

// Callback for left wheel command subscriber
void left_wheel_cmd_callback(const void * msgin){
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Control left motor based on the received command
    pid_set_setpoint(&rightMotor.motorStats.pid, msg->data);
}

// Callback for right wheel command subscriber
void right_wheel_cmd_callback(const void * msgin){
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

    // Control right motor based on the received command
    pid_set_setpoint(&rightMotor.motorStats.pid, msg->data);
}

float calculate_velocity(int32_t current_encoder_count, int32_t previous_encoder_count, float time_interval, float pulses_per_rev, float wheel_radius) {
    // Calculate the change in encoder counts
    int32_t delta_count = current_encoder_count - previous_encoder_count;

    // Calculate the angular displacement in radians
    float angular_displacement = delta_count * (2.0f * M_PI / pulses_per_rev);

    // Calculate angular velocity (radians per second)
    float angular_velocity = angular_displacement / time_interval;

    // Convert angular velocity to linear velocity (meters per second)
    float linear_velocity = angular_velocity * wheel_radius;

    return linear_velocity;
}
