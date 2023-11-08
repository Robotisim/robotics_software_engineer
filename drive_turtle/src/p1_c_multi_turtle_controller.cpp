/**
 * Author: Muhammad Luqman
 * Organization: Robotisim
 *
 * This ROS2 node, "turtlesim_straight_line", is a general-purpose controller for a Turtlebot3 robot.
 * It takes the command velocity topic and the linear velocity as parameters,
 * and publishes geometry_msgs/Twist messages to make the robot move.
 *
 * The node can be customized for different robots by changing the parameters.
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtlesimStraightLine : public rclcpp::Node {
  public:
    TurtlesimStraightLine() : Node("turtlesim_straight_line") {
      this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
      this->declare_parameter<double>("linear_velocity", 2.0);

      std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
      linear_velocity_ = this->get_parameter("linear_velocity").as_double();

      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&TurtlesimStraightLine::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_velocity_;

    void timer_callback() {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = linear_velocity_;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimStraightLine>());
  rclcpp::shutdown();
  return 0;
}
