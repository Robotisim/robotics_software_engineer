// This program publishes static transforms to tf2 in ROS 2
// It demonstrates broadcasting transforms from 'world' to 'base' frame and 'world' to '2ndframe'.
// Author: Robotisim

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

class StaticFramePublisher : public rclcpp::Node {
public:
    // Constructor: initializes the static frame publisher node
    StaticFramePublisher() : Node("static_frame_publisher"), _broadcaster(this), _xTrans(0.0) {
        // Set up a timer to call timerCallback every second
        _timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&StaticFramePublisher::timerCallback, this));
    }

private:
    // Timer callback: publishes transformations
    void timerCallback() {
        // First transform from 'world' to 'base'
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "base";
        _xTrans += 1.0; // Increment _xTrans to simulate motion
        t.transform.translation.x = _xTrans;
        t.transform.translation.y = 2.0;
        t.transform.translation.z = 1.0;

        // Set rotation to identity (no rotation)
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Second transform from 'world' to '2ndframe'
        geometry_msgs::msg::TransformStamped t1;

        t1.header.stamp = t.header.stamp; // Use the same timestamp for consistency
        t1.header.frame_id = "world";
        t1.child_frame_id = "2ndframe";
        t1.transform.translation.x = 1.0;
        t1.transform.translation.y = -2.0;
        t1.transform.translation.z = 1.0;

        q.setRPY(0, 0, 0);
        t1.transform.rotation.x = q.x();
        t1.transform.rotation.y = q.y();
        t1.transform.rotation.z = q.z();
        t1.transform.rotation.w = q.w(); // Same rotation as the first transform

        // Broadcast the transforms
        _broadcaster.sendTransform(t);
        _broadcaster.sendTransform(t1);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    tf2_ros::StaticTransformBroadcaster _broadcaster;
    double _xTrans; // Tracks the x translation of the 'base' frame
};

int main(int argc, char **argv) {
    // Initialize ROS 2 and spin the StaticFramePublisher node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticFramePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
