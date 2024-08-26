#ifndef DIFFBOT_COMMUNICATOR_HPP
#define DIFFBOT_COMMUNICATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>

class DiffBotCommunicator {
public:

 DiffBotCommunicator() = default;  // Default constructor

  DiffBotCommunicator(rclcpp::Node::SharedPtr node) {
    initialize(node);
  }


  void initialize(rclcpp::Node::SharedPtr node) {
    left_wheel_publisher_ = node->create_publisher<std_msgs::msg::Float32>("left_wheel_cmd", 10);
    right_wheel_publisher_ = node->create_publisher<std_msgs::msg::Float32>("right_wheel_cmd", 10);


  RCLCPP_INFO(rclcpp::get_logger("DiffBotCommunicator"), "Initializing DiffBotCommunicator");

  left_encoder_subscriber_ = node->create_subscription<std_msgs::msg::Int32>(
    "left_wheel_encoder", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      left_wheel_encoder_ = msg->data;
      received_left_encoder_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("DiffBotCommunicator"), "Left wheel encoder updated: %d", left_wheel_encoder_);
    });

  right_encoder_subscriber_ = node->create_subscription<std_msgs::msg::Int32>(
    "right_wheel_encoder", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      right_wheel_encoder_ = msg->data;
      received_right_encoder_ = true;
      // RCLCPP_INFO(rclcpp::get_logger("DiffBotCommunicator"), "Right wheel encoder updated: %d", right_wheel_encoder_);
    });

    imu_subscriber_ = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_orientation_[0] = msg->orientation.x;
        imu_orientation_[1] = msg->orientation.y;
        imu_orientation_[2] = msg->orientation.z;
        imu_orientation_[3] = msg->orientation.w;

        imu_angular_velocity_[0] = msg->angular_velocity.x;
        imu_angular_velocity_[1] = msg->angular_velocity.y;
        imu_angular_velocity_[2] = msg->angular_velocity.z;

        imu_linear_acceleration_[0] = msg->linear_acceleration.x;
        imu_linear_acceleration_[1] = msg->linear_acceleration.y;
        imu_linear_acceleration_[2] = msg->linear_acceleration.z;
        

      // RCLCPP_INFO(rclcpp::get_logger("DiffBotCommunicator"), "IMU data received");
    });



  RCLCPP_INFO(rclcpp::get_logger("DiffBotCommunicator"), "Subscriptions are set up");
  }


  void sendWheelCommands(float left_wheel_velocity, float right_wheel_velocity) {
    auto left_msg = std_msgs::msg::Float32();
    left_msg.data = left_wheel_velocity;
    left_wheel_publisher_->publish(left_msg);

    auto right_msg = std_msgs::msg::Float32();
    right_msg.data = right_wheel_velocity;
    right_wheel_publisher_->publish(right_msg);
  }





  float getLeftWheelEncoder() const { return left_wheel_encoder_; }
  float getRightWheelEncoder() const { return right_wheel_encoder_; }

  void getIMUData(double* orientation, double* angular_velocity, double* linear_acceleration) {
    orientation[0] = imu_orientation_[0];
    orientation[1] = imu_orientation_[1];
    orientation[2] = imu_orientation_[2];
    orientation[3] = imu_orientation_[3];

    angular_velocity[0] = imu_angular_velocity_[0];
    angular_velocity[1] = imu_angular_velocity_[1];
    angular_velocity[2] = imu_angular_velocity_[2];

    linear_acceleration[0] = imu_linear_acceleration_[0];
    linear_acceleration[1] = imu_linear_acceleration_[1];
    linear_acceleration[2] = imu_linear_acceleration_[2];
  }

  bool isReady() const {
    // Check if the agent has received at least one message from the encoders
    return received_left_encoder_ && received_right_encoder_;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  int32_t left_wheel_encoder_{0};  // Store encoder counts
  int32_t right_wheel_encoder_{0};  // Store encoder counts

  bool received_left_encoder_{false};  // Flag to indicate receipt of encoder data
  bool received_right_encoder_{false};  // Flag to indicate receipt of encoder data
    double imu_orientation_[4];  // Quaternion (x, y, z, w)
    double imu_angular_velocity_[3];  // Angular velocity (x, y, z)
    double imu_linear_acceleration_[3];  // Linear acceleration (x, y, z)

};


#endif // DIFFBOT_COMMUNICATOR_HPP
