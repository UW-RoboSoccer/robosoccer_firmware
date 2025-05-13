#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "firmware_bridge_utils.hpp"

using namespace std::chrono_literals;

class FirmwareBridgeNode : public rclcpp::Node {
public:
  FirmwareBridgeNode()
  : Node("firmware_bridge_node")
  {
    imu_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/data", 10);
    imu_euler_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/euler_ori", 10);
    force_right_pub = this->create_publisher<std_msgs::msg::Float32>("/right/force", 10);
    force_right_pub = this->create_publisher<std_msgs::msg::Float32>("/left/force", 10);
    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    joint_cmd_sub =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_cmd", 10,
        std::bind(&FirmwareBridgeNode::set_joints_callback, this,
              std::placeholders::_1));

    // Setup Timer to publish at 50Hz (20ms)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&FirmwareBridgeNode::communication_loop, this));

    RCLCPP_INFO(this->get_logger(), "Firmware bridge node has been started.");
  }

private:
  void set_joints_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
      // TODO set joints
  }

  void communication_loop() {
    auto example_msg = std_msgs::msg::Float32();

    example_msg.data = 1.0;
    imu_pub->publish(example_msg);
    imu_euler_pub->publish(example_msg);
    force_right_pub->publish(example_msg);
    force_right_pub->publish(example_msg);

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();
    joint_state_msg.name.clear();
    joint_state_msg.position.clear();
    joint_state_msg.velocity.clear();
    joint_state_msg.effort.clear();

    for (size_t i = 0; i < 16; ++i) { // TODO # of actuators
      joint_state_msg.position.push_back(0.0);
      joint_state_msg.velocity.push_back(0.0);
      joint_state_msg.effort.push_back(0.0);
    }

    joint_pub->publish(joint_state_msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_euler_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_right_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_left_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_cmd_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirmwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}