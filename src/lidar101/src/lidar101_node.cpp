#include "lidar101/lidar101.h"


using namespace lidar101;


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
    return 0;
}