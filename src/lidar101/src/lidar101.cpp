#include "lidar101/lidar101.h"
#include <pcl/filters/extract_indices.h>
#include <memory>


using namespace lidar101;


Lidar::Lidar()
: Node("lidar101")
{
    RCLCPP_INFO(this->get_logger(),
                "Starting lidar101");

    // declare the parameters
    this->RADIUS_ = this->declare_parameter("radius", 10.0);
    this->base_footprint_frame_ = this->declare_parameter("frame_to_publish_in", "base_footprint");

    // publishers and subscribers
    rclcpp::QoS queue_size = 1;
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("radius_pc", queue_size);

    std::string sub_topic_ = this->declare_parameter("subscription_topic", "/velodyne_points");
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(sub_topic_,
                                                                    queue_size,
                                                                    std::bind(&Lidar::velodyne_callback, this, std::placeholders::_1));

    param_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>("/parameter_events",
                                                                                rclcpp::QoS(10),
                                                                                std::bind(&Lidar::parameter_callback, this, std::placeholders::_1));

    // transforms
    clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void Lidar::parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    RCLCPP_INFO(this->get_logger(),
                "parameter callback");

    for (const auto &param: event->changed_parameters) {
        if (param.name == "radius") this->get_parameter("radius", this->RADIUS_);
    }
}


void Lidar::velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // transform message into base footprint frame
    // convert the message to a point cloud
    sensor_msgs::msg::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud(this->base_footprint_frame_, *msg, transformed_msg, *tf_buffer_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_msg, *cloud);

    // get the points within the radius and add them to close_points
    pcl::PointCloud<pcl::PointXYZ>::Ptr close_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto point: *cloud) {
        float squared_range = (point.x * point.x) + (point.y * point.y);

        if (squared_range <= this->RADIUS_ * this->RADIUS_) {
            close_points->points.push_back(point);
        }
    }

    // publish the new point cloud within the radius
    sensor_msgs::msg::PointCloud2 close_points_msg;
    pcl::toROSMsg(*close_points, close_points_msg);
    close_points_msg.header.frame_id = this->base_footprint_frame_;
    pub_->publish(close_points_msg);
}
