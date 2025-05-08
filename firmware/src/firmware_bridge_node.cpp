#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/Float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "firmware_bridge_utils.hpp"

using namespace std::chrono_literals;

class FirmwareBridge : public rclcpp::Node {
public:
  FirmwareBridge()
  : Node("firmware_bridge")
  {
    imu_pub = this->create_publisher<std_msgs::msg::Float32>("/imu/data", 10);

    joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); // todo need one for each limb?

    // Setup Timer to publish at 50Hz (20ms)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&FirmwareBridge::communication_loop, this));

    RCLCPP_INFO(this->get_logger(), "Firmware bridge node has been started.");
  }

private:
  void communication_loop() {
    auto imu_msg = std_msgs::msg::Float32();

    imu_msg.data = 1.0;// todo read imu
    imu_pub->publish(imu_msg);

    // todo publish joint states
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FirmwareBridge>());
  rclcpp::shutdown();
  return 0;
}