// You can remove or add any includes you need
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>
#include <std_msgs/msg/u_int16.hpp>
#include <algorithm>

namespace solution {

class Solution final : public rclcpp::Node {
public:
  explicit Solution(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp::Node("solution", options) {
    // Short example you can remove it
    const auto preempt = declare_parameter<bool>("preempt", false);
    RCLCPP_INFO(get_logger(), "%s", preempt ? "Preempt" : "Queue");

    _motor1 = create_publisher<std_msgs::msg::Int8>("motor_1/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
    _motor2 = create_publisher<std_msgs::msg::Int8>("motor_2/control", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    _encoder1 = create_subscription<std_msgs::msg::UInt16>("motor_1/encoder", 10,[this](const std_msgs::msg::UInt16::SharedPtr msg) {_current_yaw = msg ->data;});
    _encoder2 = create_subscription<std_msgs::msg::UInt16>("motor_2/encoder", 10, [this](const std_msgs::msg::UInt16::SharedPtr msg) { _current_pitch = msg->data; }); 

    _target = create_subscription<geometry_msgs::msg::Point>("target", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      [this](const geometry_msgs::msg::Point::SharedPtr point) {
        RCLCPP_INFO(get_logger(), "New Target: (%g, %g, %g)", point->x, point->y, point->z);

        double yaw_rad = std::atan2(point->y, point->x);
        double distance_xy = std::sqrt(point->x * point->x + point->y * point->y);
        double pitch_rad = std::atan2(point->z, distance_xy);

        if(yaw_rad < 0) yaw_rad += 2.0 * M_PI;
        if(pitch_rad < 0) pitch_rad += 2.0 * M_PI;

        _target_yaw = static_cast<int>((yaw_rad / (2.0 *M_PI))*4096.0)%4096;
        _target_pitch = static_cast<int>((pitch_rad / (2.0 *M_PI))*4096.0)%4096;

        RCLCPP_INFO(get_logger(), "Converted Target Ticks: Yaw: %d, Pitch: %d", _target_yaw, _target_pitch);
      });

      _timer = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        double kp = 0.5;
        
        int error_yaw = _target_yaw - _current_yaw;
        if(error_yaw >2048) error_yaw -= 4096;
        else if (error_yaw < -2048) error_yaw += 4096;

        std_msgs::msg::Int8 cmd1;
        if (std::abs(error_yaw) <= 2) cmd1.data = 0; 
        else cmd1.data = std::clamp(static_cast<int>(error_yaw *kp), -128, 127);
        _motor1->publish(cmd1);;

        int error_pitch = _target_pitch - _current_pitch;
        if (error_pitch > 2048) error_pitch -= 4096;
        else if (error_pitch < -2048) error_pitch += 4096;

        std_msgs::msg::Int8 cmd2;
        if (std::abs(error_pitch) <= 2) cmd2.data = 0;
        else cmd2.data = std::clamp(static_cast<int>(error_pitch *kp), -128, 127);
        _motor2->publish(cmd2);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor1;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _motor2;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _target;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder1;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _encoder2;
  rclcpp::TimerBase::SharedPtr _timer;

  int _target_yaw =0;
  int _target_pitch =0;
  int _current_yaw =0;
  int _current_pitch =0;
};

}  // namespace solution

RCLCPP_COMPONENTS_REGISTER_NODE(solution::Solution)
