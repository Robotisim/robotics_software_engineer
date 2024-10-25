#include "rclcpp/rclcpp.hpp"
#include "line_following/line_following.hpp"

using namespace std;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto line_follower = std::make_shared<LineFollowing>();
    rclcpp::spin(line_follower);
    rclcpp::shutdown();
    return 0;
}