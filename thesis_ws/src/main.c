#include <stdio.h>

// ROS 2
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <control_msgs/msg/pid_state.h>
#include <nav_msgs/msg/odometry.h>

// Pico SDK
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

// Custom motor control includes
#include "motor_control.h"
#include "encoder.h"
#include "motor.h"
#include "motor_calcs.h"
#include "icm42688.h"
#define PWM_MAX 50.0f
#define ROSMODE 0
const uint LED_PIN = 25;

rcl_publisher_t pico_string_publisher, pico_float_publisher, publisher_odometer, pico_int_publisher, left_encoder, right_encoder;
std_msgs__msg__String msg_string_pid, msg_string_test;
std_msgs__msg__Float32 msg_float_left, msg_float_right, float_test;
std_msgs__msg__Int32 msg_int_test, left_encoder_msg, right_encoder_msg;
control_msgs__msg__PidState pid_msg, pid_in;
nav_msgs__msg__Odometry odo_msg;
Motor leftMotor, rightMotor;
Odometry_values odo_vals;
const uint ENCODERA = 0;
const uint ENCODERB = 1;

float test = -1.0f;

// Callback functions for ENCODER

void publish_odo()
{
    uint64_t curr_time = time_us_64();
    odo_msg.header.frame_id.data = "odom";
    odo_msg.child_frame_id.data = "base_link";
    odo_msg.header.stamp.sec = (int32_t)(curr_time / 1000000);
    odo_msg.header.stamp.nanosec = (int32_t)(curr_time % 1000000) * 1000;

    odo_msg.pose.pose.position.x = odo_vals.x; // x position
    odo_msg.pose.pose.position.y = odo_vals.y; // y position
    odo_msg.pose.pose.position.z = 0.0;        // z position
    // set the orientation
    odo_msg.pose.pose.orientation.x = 0.0; // x orientation
    odo_msg.pose.pose.orientation.y = 0.0; // y orientation
    odo_msg.pose.pose.orientation.z = 0;
    // odo_msg.pose.pose.orientation.w = 0.0; // w orientation
    // set the linear velocity
    odo_msg.twist.twist.linear.z = 0;
    odo_msg.twist.twist.linear.x = leftMotor.motorStats.angular_velocity;
    odo_msg.twist.twist.linear.y = rightMotor.motorStats.angular_velocity;

    // set the angular velocity
    odo_msg.twist.twist.angular.x = leftMotor.motorStats.pid.error;
    odo_msg.twist.twist.angular.y = leftMotor.motorStats.pid.setpoint;
    odo_msg.twist.twist.angular.z = leftMotor.motorStats.pid.output;
    rcl_publish(&publisher_odometer, &odo_msg, NULL);
}

void motor_iteration(double dt)
{
    double outputA = pid_update(&leftMotor.motorStats.pid, leftMotor.motorStats.angular_velocity, dt);
    double outputB = pid_update(&rightMotor.motorStats.pid, rightMotor.motorStats.angular_velocity, dt);

    float pwmA = fabs(outputA);
    float pwmB = fabs(outputB);

    if (pwmA > PWM_MAX)
    {
        pwmA = PWM_MAX;
    }
    if (pwmB > PWM_MAX)
    {
        pwmB = PWM_MAX;
    }

    control_motor(rightMotor, outputB, pwmB);
    control_motor(leftMotor, outputA, pwmA);

    rightMotor.motorStats.PWM = outputB;
    leftMotor.motorStats.PWM = outputA;
}

// Timer callback functions for encoder reading and motor calcs

void timer_callback1(rcl_timer_t *timer, int64_t last_call_time)
{
    float dt = last_call_time / 1000000000.0f; // ns to s
    int left_encoder_count = get_encoder_count_A();
    int right_encoder_count = get_encoder_count_B();
    left_encoder_msg.data = left_encoder_count;
    right_encoder_msg.data = right_encoder_count;

    calc_stats(dt, &odo_vals, left_encoder_count, right_encoder_count, &leftMotor.motorStats, &rightMotor.motorStats);

    rcl_publish(&left_encoder, &left_encoder_msg, NULL);
    rcl_publish(&right_encoder, &right_encoder_msg, NULL);
    publish_odo();
}

void timer_callback2(rcl_timer_t *timer, int64_t last_call_time)
{
    // Publish the PID Kp value for debugging (if necessary)
    float dt = last_call_time / 1000000000.0f;
    float_test.data = dt;
    rcl_publish(&pico_float_publisher, &float_test, NULL);
    motor_iteration(dt);

    // Publish an example integer value (e.g., motor encoder count)
    msg_int_test.data = get_encoder_count_A(); // Or any other relevant integer value
    rcl_publish(&pico_int_publisher, &msg_int_test, NULL);
}

void print_pid_terms()
{
    char pid_string[50];
    sprintf(pid_string, "PID: P=%f, I=%f, D=%f", leftMotor.motorStats.pid.Kp, leftMotor.motorStats.pid.Ki, leftMotor.motorStats.pid.Kd);
    msg_string_pid.data.data = pid_string;
    msg_string_pid.data.size = strlen(pid_string);
    rcl_ret_t ret = rcl_publish(&pico_string_publisher, &msg_string_pid, NULL);
}

void left_wheel_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg_received = (const std_msgs__msg__Float32 *)msgin;
    // Publish received command as string
    msg_string_test.data.data = "Published from left_cmd_callback";
    msg_string_test.data.size = strlen(msg_string_test.data.data);
    rcl_ret_t ret = rcl_publish(&pico_string_publisher, &msg_string_test, NULL);

    // Set the PID setpoint for the left motor
    pid_set_setpoint(&leftMotor.motorStats.pid, msg_received->data);
}

void right_wheel_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32 *msg_received = (const std_msgs__msg__Float32 *)msgin;
    // Publish received command as string
    msg_string_test.data.data = "Published from right_cmd_callback";
    msg_string_test.data.size = strlen(msg_string_test.data.data);
    rcl_ret_t ret = rcl_publish(&pico_string_publisher, &msg_string_test, NULL);

    // Set the PID setpoint for the right motor
    pid_set_setpoint(&rightMotor.motorStats.pid, msg_received->data);
}

void pid_state_callback(const void *msgin)
{
    const control_msgs__msg__PidState *msg_received = (const control_msgs__msg__PidState *)msgin;

    // Variables to store P, I, D values from the PidState message
    double p_term = msg_received->p_term;
    double i_term = msg_received->i_term;
    double d_term = msg_received->d_term;

    // Set the PID gains using the parsed values
    pid_set_gains(&leftMotor.motorStats.pid, p_term, i_term, d_term);
    pid_set_gains(&rightMotor.motorStats.pid, p_term, i_term, d_term);

    // Optionally publish a confirmation message
    char pid_string[50];
    sprintf(pid_string, "R PID: P=%lf, I=%lf, D=%lf", p_term, i_term, d_term);

    msg_string_test.data.data = pid_string;
    msg_string_test.data.size = strlen(pid_string);
    rcl_ret_t ret = rcl_publish(&pico_string_publisher, &msg_string_test, NULL);
    print_pid_terms();
}

int main()
{
    stdio_init_all(); // Initialize all configured stdio types


    sleep_ms(2000);
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    icm42688_init(i2c_default);
    icm42688_reset(i2c_default);


    
    float ax = 0, ay =0 , az = 0 , gx= 0 , gy= 0 , gz=  0;

    // Initialize motors
    #if ROSMODE
        float kp = 10, ki = 1, kd = 0;
        init_motor(&leftMotor, MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2, MOTOR1_ENCODER, kp, ki, kd, 0);
        init_motor(&rightMotor, MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCODER, kp, ki, kd, 0);

        // Initialize encoders
        init_PIO_encoder(MOTOR1_ENCODER, MOTOR2_ENCODER, ENCODERA, ENCODERB);

        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read);

        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);

        rcl_timer_t timer1, timer2;
        rcl_node_t node;
        rcl_allocator_t allocator;
        rclc_support_t support;
        rclc_executor_t executor;
        rcl_subscription_t left_wheel_cmd_sub, right_wheel_cmd_sub, pid_state_sub;

        msg_string_test.data.data = "init_string_test";
        msg_string_test.data.capacity = 50;
        msg_string_pid.data.data = "init_string_pid";
        msg_string_pid.data.capacity = 50;
        msg_float_left.data = 0.0f;
        msg_float_right.data = 0.0f;
        msg_int_test.data = 0;

        allocator = rcl_get_default_allocator();

        // Wait for agent successful ping for 2 minutes.
        const int timeout_ms = 1000;
        const uint8_t attempts = 120;

        rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

        if (ret != RCL_RET_OK)
        {
            // Unreachable agent, exiting program.
            return ret;
        }

        rclc_support_init(&support, 0, NULL, &allocator);

        rclc_node_init_default(&node, "pico_node", "", &support);

        // Initialize the string publisher
        rclc_publisher_init_default(
            &pico_string_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "pico_string_publisher");

        // Initialize the float publisher for debugging
        rclc_publisher_init_default(
            &pico_float_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "pico_float_publisher");

        // Initialize the float publisher for left_cmd
        rclc_publisher_init_default(
            &left_encoder,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_wheel_encoder");

        // Initialize the float publisher for right_cmd
        rclc_publisher_init_default(
            &right_encoder,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_wheel_encoder");

        // Initialize the int publisher
        rclc_publisher_init_default(
            &pico_int_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico_int_publisher");

        // Initialize the int publisher
        rclc_publisher_init_default(
            &pico_int_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico_int_publisher");
            
        // Initialize the int publisher
        rclc_publisher_init_default(
            &pico_int_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico_int_publisher");

        rclc_publisher_init_default(
            &publisher_odometer,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "pico_odometry");

        rclc_subscription_init_default(
            &left_wheel_cmd_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_cmd");

        rclc_subscription_init_default(
            &right_wheel_cmd_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_cmd");

        rclc_subscription_init_default(
            &pid_state_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, PidState),
            "pid_state_cmd");

        rclc_timer_init_default(
            &timer1,
            &support,
            RCL_MS_TO_NS(50),
            timer_callback1);

        rclc_timer_init_default(
            &timer2,
            &support,
            RCL_MS_TO_NS(50),
            timer_callback2);

        rclc_executor_init(&executor, &support.context, 6, &allocator); // Updated executor count to 6
        rclc_executor_add_timer(&executor, &timer1);
        rclc_executor_add_timer(&executor, &timer2);
        rclc_executor_add_subscription(&executor, &pid_state_sub, &pid_in, &pid_state_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &left_wheel_cmd_sub, &msg_float_left, &left_wheel_cmd_callback, ON_NEW_DATA);
        rclc_executor_add_subscription(&executor, &right_wheel_cmd_sub, &msg_float_right, &right_wheel_cmd_callback, ON_NEW_DATA);
    #else
    
        gpio_put(LED_PIN, 1);    
    #endif

    gpio_put(LED_PIN, 1);

    while (true)
    {
        #if ROSMODE
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
        #else
            icm42688_read_accel(i2c_default, &ax, &ay, &az);
            icm42688_read_gyro(i2c_default, &gx, &gy, &gz);
            printf("Accel: ax=%.2f ay=%.2f az=%.2f\n", ax, ay, az);
            printf("Gyro: gx=%.2f gy=%.2f gz=%.2f\n", gx, gy, gz);
            sleep_ms(500);
        #endif
    }

    // Clean up resources (not strictly necessary for an embedded system like Pico, but good practice)
    #if ROSMODE
    
    rclc_executor_fini(&executor);
    rclc_timer_fini(&timer1);
    rclc_timer_fini(&timer2);
    rclc_publisher_fini(&pico_string_publisher, &node);
    rclc_publisher_fini(&pico_float_publisher, &node);
    rclc_publisher_fini(&pico_int_publisher, &node); // Clean up int publisher
    rclc_subscription_fini(&left_wheel_cmd_sub, &node);
    rclc_subscription_fini(&right_wheel_cmd_sub, &node);
    rclc_subscription_fini(&pid_state_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    
    #endif
    return 0;
}
