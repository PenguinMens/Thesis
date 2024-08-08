#ifndef DIFFBOT_COMMUNICATOR_HPP
#define DIFFBOT_COMMUNICATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

class DiffBotCommunicator {
public:

 DiffBotCommunicator() = default;  // Default constructor

  DiffBotCommunicator(rclcpp::Node::SharedPtr node) {
    initialize(node);
  }


  void initialize(rclcpp::Node::SharedPtr node) {
    left_wheel_publisher_ = node->create_publisher<std_msgs::msg::Float32>("left_wheel_cmd", 10);
    right_wheel_publisher_ = node->create_publisher<std_msgs::msg::Float32>("right_wheel_cmd", 10);

    left_encoder_subscriber_ = node->create_subscription<std_msgs::msg::Int32>(
      "left_wheel_encoder", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        left_wheel_encoder_ = msg->data;
        received_left_encoder_ = true;
      });

    right_encoder_subscriber_ = node->create_subscription<std_msgs::msg::Int32>(
      "right_wheel_encoder", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        right_wheel_encoder_ = msg->data;
        received_right_encoder_ = true;
      });
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

  bool isReady() const {
    // Check if the agent has received at least one message from the encoders
    return received_left_encoder_ && received_right_encoder_;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_encoder_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_encoder_subscriber_;

  int32_t left_wheel_encoder_{0};  // Store encoder counts
  int32_t right_wheel_encoder_{0};  // Store encoder counts

  bool received_left_encoder_{false};  // Flag to indicate receipt of encoder data
  bool received_right_encoder_{false};  // Flag to indicate receipt of encoder data
};


#endif // DIFFBOT_COMMUNICATOR_HPP
