// Purpose:
// - Navigate a robot through a maze using LIDAR data
// - Make decisions based on obstacle proximity to maneuver through the maze
// Author: Robotisim

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum class RobotState {
  MOVING_STRAIGHT,
  TURNING_LEFT,
  TURNING_RIGHT,
  OUT_OF_MAZE
};

class MazeSolving : public rclcpp::Node {
public:
  MazeSolving() : Node("maze_solving"), _state(RobotState::MOVING_STRAIGHT) {
    _publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    _subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg) {
    // Analyze LIDAR data to navigate through the maze
    float rightObstacle = *std::min_element(lidarMsg->ranges.begin() + 260,
                                            lidarMsg->ranges.begin() + 280);
    float frontObstacle = *std::min_element(lidarMsg->ranges.begin() + 340,
                                            lidarMsg->ranges.begin() + 360);
    float leftObstacle = *std::min_element(lidarMsg->ranges.begin() + 80,
                                           lidarMsg->ranges.begin() + 100);

    // Log obstacle distances
    RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f,",
                frontObstacle, rightObstacle, leftObstacle);

    // Determine robot's state based on obstacle proximity
    if (frontObstacle > _frontThreshold && rightObstacle > _frontThreshold &&
        leftObstacle > _frontThreshold) {
      _state = RobotState::OUT_OF_MAZE;
    } else if (frontObstacle < _frontThreshold) {
      _state = leftObstacle < rightObstacle ? RobotState::TURNING_RIGHT
                                            : RobotState::TURNING_LEFT;
    }

    // Adjust robot's motion based on the state
    geometry_msgs::msg::Twist command;
    switch (_state) {
    case RobotState::MOVING_STRAIGHT:
      command.linear.x = _linearVel;
      command.angular.z = 0.0;
      break;
    case RobotState::TURNING_LEFT:
      command.linear.x = 0.0;
      command.angular.z = _angularVel;
      break;
    case RobotState::TURNING_RIGHT:
      command.linear.x = 0.0;
      command.angular.z = -_angularVel;
      break;
    case RobotState::OUT_OF_MAZE:
      command.linear.x = 0.0;
      command.angular.z = 0.0;
      break;
    }

    // Publish the command
    _publisher->publish(command);
  }

  float _frontThreshold = 1.5f;
  float _angularVel = 0.8f;
  float _linearVel = 0.6f;
  RobotState _state;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _subscription;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeSolving>());
  rclcpp::shutdown();
  return 0;
}
