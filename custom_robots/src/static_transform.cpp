#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/LinearMath/Quaternion.h>

// The StaticFramePublisher node
class StaticFramePublisher : public rclcpp::Node {
  public:
    StaticFramePublisher() : Node("Static_Frame_Publisher") , broadcaster_(this) {
      // Set up a timer
      timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                       std::bind(&StaticFramePublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        geometry_msgs::msg::TransformStamped t;
        geometry_msgs::msg::TransformStamped t1;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "base";
        x_trans +=1;
        t.transform.translation.x = x_trans;
        t.transform.translation.y = 2.0;
        t.transform.translation.z = 1.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();


        t1.header.frame_id = "world";
        t1.child_frame_id = "2ndframe";

        t1.transform.translation.x = 1.0;
        t1.transform.translation.y = -2.0;
        t1.transform.translation.z = 1.0;

        tf2::Quaternion q1;
        q1.setRPY(0, 0, 0);
        t1.transform.rotation.x = q1.x();
        t1.transform.rotation.y = q1.y();
        t1.transform.rotation.z = q1.z();
        t1.transform.rotation.w = q1.w();


        broadcaster_.sendTransform(t);
        // broadcaster_.sendTransform(t1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;
    double x_trans;
};

int main(int argc, char **argv) {
  // Initialize ROS, create the StaticFramePublisher node, and spin it
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}
