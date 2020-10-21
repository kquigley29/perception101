#ifndef EUFS_LIDAR101_H
#define EUFS_LIDAR101_H


// cpp
#include <string>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//// transforms
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2/time.h>

// perception_pcl
//#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>


namespace lidar101 {

    class Lidar : public rclcpp::Node {
    public:
        Lidar();

    private:
        // callback to receive and work on point cloud data
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        // publishers and subscribers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

//    // Transforms
//    rclcpp::Clock::SharedPtr clock;
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // params
        float RADIUS_;
        std::string base_footprint_frame_;

        void update_params();
    };

} // lidar101

#endif //EUFS_LIDAR101_H
