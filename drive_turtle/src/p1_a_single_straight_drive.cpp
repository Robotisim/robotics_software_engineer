/**
 * @file p1_a_single_straight_drive.cpp
 * @author Muhammad Luqman
 * @brief This ROS2 node continuously publishes Twist messages to a specified topic to make a turtle robot move in a straight line.
 * @organization Robotisim
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// The TurtlesimStraightLine node
class TurtlesimStraightLine : public rclcpp::Node {
  public:
    TurtlesimStraightLine() : Node("turtlesim_straight_line") {
      // Set up a publisher and a timer
      cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/turtle1/cmd_vel", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                       std::bind(&TurtlesimStraightLine::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
      // The callback function creates a Twist message with a linear velocity,
      // which will make the turtle move in a straight line, and publishes the message.
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 2.0;
      twist.angular.z = 0.0;
      cmd_vel_pub_->publish(twist);
    }
};

int main(int argc, char **argv) {
  // Initialize ROS, create the TurtlesimStraightLine node, and spin it
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlesimStraightLine>());
  rclcpp::shutdown();
  return 0;
}
