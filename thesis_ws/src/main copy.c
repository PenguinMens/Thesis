#include <stdio.h>
#include <math.h>
#include <inttypes.h>

// ros
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point32.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <control_msgs/msg/pid_state.h>

// pico
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"

// custom
#include "custom_pwm.h"
#include "motor_control.h"
#include "motor_calcs.h"
#include "motor_pid.h"
#include "quadrature_encoder.pio.h"
#include "encoder.h"
#include "motor.h"

#define ROS_MODE 1
#define PWM_MAX 50.0f


float time_test = 0.0f;
float left_speed_target = 0;
float right_speed_target =0;


int32_t test_A = 0;
int32_t test_B = 0;

const uint LED_PIN = 25;

Odometry_values odo_vals;
// incrimenting puiblisher
rcl_publisher_t publisher, publisher_imu, publisher_twist, publisher_odomoter, publisher_pid;
rcl_subscription_t cmd_vel_subscriber, pid_terms_subscriber;


std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist geo_msg;
nav_msgs__msg__Odometry odo_msg;
control_msgs__msg__PidState pid_msg,pid_in;

//subscriber for input

rcl_timer_t timer,timer2,timer3; // general timer current used for motor


rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;




void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_callback(const void * msgin);
int controlMotorsPID(float dt);
void update_setpoint(double x, double z);
void motor_iteration(uint64_t last_call_time);
int init_i2c();
void publish_imu_raw();
void timer2_callback(rcl_timer_t * timer, int64_t last_call_time);
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time);
void sendMotorDataCSV();
void publish_odo();
void pid_in_callback(const void * msgin);



//flag for mtoor direction tests

Motor leftMotor, rightMotor;
uint64_t last_trigger_time = 0;
uint32_t duration = 0;

const uint ENCODERA =0;
const uint ENCODERB = 1;
bool generic_flag = 0;



int main(){
    stdio_init_all(); // Initialize all configured stdio types


    gpio_init(6);
    gpio_set_dir(6, GPIO_OUT);
    gpio_put(6, 1);
    init_i2c();
    float kp =  0;
    float ki =0;  
    float kd = 0;

    init_motor(&leftMotor,MOTOR1_PWM, MOTOR1_IN1, MOTOR1_IN2,  MOTOR1_ENCODER, kp, ki, kd, 0);
    init_motor(&rightMotor,MOTOR2_PWM, MOTOR2_IN1, MOTOR2_IN2, MOTOR2_ENCODER, kp, ki, kd, 0);
    // init_motors(15,16,17,14,18,19);
    const uint PIN_AB = MOTOR1_ENCODER;
    const uint PIN_CD = MOTOR2_ENCODER;
    // @todo fix this shit, needs to be put into motor.c
    init_PIO_encoder(PIN_AB, PIN_CD, ENCODERA,ENCODERB);


    msg.data = 0;
    int new_value, delta, old_value = 0;

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

        // Wait for agent successful ping for 2 minutes.
        const int timeout_ms = 1000; 
        const uint8_t attempts = 120;

        rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
        if (ret != RCL_RET_OK){
            // Unreachable agent, exiting program.
            return ret;
        }

        rclc_support_init(&support, 0, NULL, &allocator);
        // std msg currently used for debugging : motor speeds
        rclc_node_init_default(&node, "pico_node", "", &support);

        rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "pico_publisher");


        rclc_publisher_init_default(
            &publisher_imu,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),

            "pico_imu");

        rclc_publisher_init_default(
            &publisher_twist,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg, Twist),
            "pico_twist");
        
        rclc_publisher_init_default(
            &publisher_odomoter,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg, Odometry),
            "pico_odometry");

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

        // subscriber for cmdvel
        rclc_subscription_init_best_effort(&cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg, Twist),
            "/cmd_vel");


        rclc_executor_init(&executor, &support.context,
            6,
            &allocator);

        // timer for mtoor controll
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(20),
            timer_callback);

        rcl_ret_t rc = rclc_timer_init_default(
            &timer2,
            &support,
            RCL_MS_TO_NS(1000),
            timer2_callback);

        rclc_timer_init_default(
            &timer3,
            &support,
            RCL_MS_TO_NS(100),
            timer3_callback);

        rclc_executor_add_timer(&executor, &timer);
        rclc_executor_add_timer(&executor, &timer2);
        rclc_executor_add_timer(&executor, &timer3);
        rclc_executor_add_subscription(&executor,
            &cmd_vel_subscriber,
            &geo_msg,
            &cmd_vel_callback,
            ON_NEW_DATA);
        rclc_executor_add_subscription(&executor,
            &pid_terms_subscriber,
            &pid_in,
            &pid_in_callback,
            ON_NEW_DATA);

    #else    
     float setpoint = 0.05f;
     int i =0;


    

    #endif
   
    while (true)
    {

        #if ROS_MODE
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        #else
        uint64_t current_time = time_us_64();
        if (current_time - last_trigger_time >= timer_interval_us) {
            if(generic_flag)
            {
                // i++;
                // if(i > 400)
                // {
                //     // update_setpoint(setpoint,0);
                //     // update_setpoint(setpoint,0);
                //     // setpoint = setpoint + 0.01;
                //     i = 0;
                //     update_setpoint(0,0);
                //     generic_flag = 0;
                //     printf("end\n");
                // }
                motor_iteration(timer_interval_us);
                last_trigger_time = current_time;
                duration = duration+( timer_interval_us / 1000);

               
            }


      

        }
         if (tud_cdc_available()) { // Check if data is available to read
            uint8_t buf[64];
            uint32_t count = tud_cdc_read(buf, sizeof(buf)); // Read the data into buf
            printf("start\n");
            
            last_trigger_time = current_time;
            if( generic_flag == 0)
            {
                printf("start\n");
                update_setpoint(setpoint,0);
                generic_flag = 1;
            }
            else
            {
                printf("end\n");
                generic_flag = 0;
                update_setpoint(0,0);
                motor_iteration(timer_interval_us);
            }

        }
        
        #endif
        
    }
    return 0;
}

int init_i2c(){
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Don't forget the pull ups! | Or use external ones
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    return 0;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){	

    pid_set_message(&leftMotor.motorStats.pid, &pid_msg);
    rcl_publish(&publisher_pid, &pid_msg, NULL);
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time){	

    rcl_publish(&publisher, &msg, NULL);
}

void pid_in_callback(const void * msgin)
{
    pid_set_gains(&leftMotor.motorStats.pid, pid_in.p_term, pid_in.i_term, pid_in.d_term);
    pid_set_gains(&rightMotor.motorStats.pid, pid_in.p_term, pid_in.i_term, pid_in.d_term);

}

void motor_iteration( uint64_t last_call_time) // probaby makes more sense to call this a motor iteration
{
    
    test_A = get_encoder_count_A();
    test_B = get_encoder_count_B();
    reset_encoders();
    time_test =  last_call_time/1000.0f;

    //printf("%" PRIu64 "\n", last_call_time);
    calc_stats(time_test, &odo_vals,test_A,test_B, &leftMotor.motorStats, &rightMotor.motorStats ); 
    controlMotorsPID(time_test);
}
void timer3_callback(rcl_timer_t * timer, int64_t last_call_time){


    motor_iteration(last_call_time/1000.0f);
    msg.data = last_call_time;
    rcl_publish(&publisher,&msg,NULL);
    odo_msg.pose.pose.orientation.w = time_test;
    
    publish_odo(last_call_time);

    
}

void update_setpoint(double x, double z)
{
    left_speed_target =  x - 0.5f*z*0.2;
    right_speed_target = x + 0.5f*z*0.2;

    pid_set_setpoint(&leftMotor.motorStats.pid, left_speed_target);
    pid_set_setpoint(&rightMotor.motorStats.pid, right_speed_target);
    


}
void cmd_vel_callback(const void * msgin){
    
   rcl_publish(&publisher_twist, &geo_msg, NULL);

   update_setpoint(geo_msg.linear.x,geo_msg.angular.z);

}



void publish_odo(uint64_t current_time)
{
    uint64_t curr_time = time_us_64();
    odo_msg.header.frame_id.data = "odom";
    odo_msg.child_frame_id.data = "base_link";
    odo_msg.header.stamp.sec = (int32_t) (curr_time/1000000);
    odo_msg.header.stamp.nanosec = (int32_t) (curr_time%1000000)*1000;

    odo_msg.pose.pose.position.x = odo_vals.x; // x position
    odo_msg.pose.pose.position.y = odo_vals.y; // y position
    odo_msg.pose.pose.position.z = 0.0; // z position
    // set the orientation
    odo_msg.pose.pose.orientation.x = 0.0; // x orientation 
    odo_msg.pose.pose.orientation.y = 0.0; // y orientation
    odo_msg.pose.pose.orientation.z =0;
    // odo_msg.pose.pose.orientation.w = 0.0; // w orientation
    // set the linear velocity 
    odo_msg.twist.twist.linear.z =0;
    odo_msg.twist.twist.linear.x = odo_vals.linear_velocity;
    odo_msg.twist.twist.linear.y =0;

    // set the angular velocity 
    odo_msg.twist.twist.angular.x = 0;//leftMotor.motorStats.rps;
    odo_msg.twist.twist.angular.y = 0;// rightMotor.motorStats.rps;
    odo_msg.twist.twist.angular.z = odo_vals.angular_velocity;
    rcl_publish(&publisher_odomoter, &odo_msg, NULL);
}

void publish_imu_raw() {
    uint64_t current_time = time_us_64();

    imu_msg.header.frame_id.data = ("imu_link");
    imu_msg.header.stamp.sec = (int32_t) (current_time/1000000);

    imu_msg.angular_velocity.x =  0;
     imu_msg.angular_velocity.y = 0;
     imu_msg.angular_velocity.z = 0;
     imu_msg.linear_acceleration.x = 0;
     imu_msg.linear_acceleration.y = 0;
     imu_msg.linear_acceleration.z = 0;
    // publish imu data
    rcl_publish(&publisher_imu, &imu_msg, NULL);

}

int controlMotorsPID(float dt)
{
    static bool headersPrinted = false;
    // if (!headersPrinted) {
    //     printf("Time, Motor, PID Output, PWM, Velocity, KP, KI, KD\n");
    //     headersPrinted = true;
    // }

    double outputA = pid_update(&leftMotor.motorStats.pid, leftMotor.motorStats.velocity, dt);
    float pwmA =  fabs(outputA);
    if (pwmA > PWM_MAX) pwmA = PWM_MAX;
    
    double outputB = pid_update(&rightMotor.motorStats.pid, rightMotor.motorStats.velocity, dt);
    float pwmB = fabs(outputB);
    if (pwmB > PWM_MAX) pwmB = PWM_MAX;
    

    

//     printf("%f, A, %f, %f, %f, %f, %f ,%d\n", dt, outputA, pwmA, leftMotor.motorStats.velocity,left_speed_target,  odo_vals.linear_velocity, test_A);
//     // printf("%f, B, %f, %f, %f, %f, %f\n", dt, outputB, pwmB, rightMotor.motorStats.velocity, right_speed_target,  odo_vals.linear_velocity);
//    printf("%f, B, %f, %f, %f, %f, %f, %d\n", dt, outputB, pwmB, rightMotor.motorStats.velocity,  right_speed_target,  odo_vals.linear_velocity, test_B);

    if(left_speed_target == 0 && right_speed_target == 0){
        outputB = 0;
        pwmB = 0;
        outputA = 0;
        pwmA = 0;
        pid_reset(&rightMotor.motorStats.pid);
        pid_reset(&leftMotor.motorStats.pid);
    }

    control_motor(rightMotor,outputB, pwmB);
    control_motor(leftMotor,outputA, pwmA);
    rightMotor.motorStats.PWM = pwmB;
    leftMotor.motorStats.PWM = pwmA;
    #if !ros_mode
    {
        sendMotorDataCSV();
    }
    #endif

    
    return 0;
}

void sendMotorDataCSV() {
    // char buffer[128];
    // snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
    //          leftMotor.motorStats.velocity,
    //          leftMotor.motorStats.rps,
    //          leftMotor.motorStats.pid.Kp,
    //          leftMotor.motorStats.pid.Ki,
    //          leftMotor.motorStats.pid.Kd,
    //          leftMotor.motorStats.pid.setpoint,
    //          leftMotor.motorStats.pid.error,
    //          leftMotor.motorStats.pid.integral,
    //          leftMotor.motorStats.pid.derivative,
    //          leftMotor.motorStats.pid.previous_error);
    // Serial.print(buffer);

    printf("%c,%.4f,%.4f,%.2f,%.2f,%.2f\n",'L',
            leftMotor.motorStats.velocity,
            leftMotor.motorStats.rps,
            leftMotor.motorStats.PWM,
            leftMotor.motorStats.pid.error,
            leftMotor.motorStats.pid.previous_error            
            );
    printf("%c,%.4f,%.4f,%.2f,%.2f,%.2f\n",'R',
            rightMotor.motorStats.velocity,
            rightMotor.motorStats.rps,
            rightMotor.motorStats.PWM,
            rightMotor.motorStats.pid.error,
            rightMotor.motorStats.pid.previous_error            
            );
    
}