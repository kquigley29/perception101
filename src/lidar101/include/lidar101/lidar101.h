/*
In this header file we create the class and declare its member functions which includes its
 constructor and callbacks. We also declare variables to hold the parameters.

 All necessary headers have already been included. Be sure to go over these and understand
 which functions each provides.

The instructions are found below numbered in square brackets eg. [1] Inherit from rclcpp::Node
*/


// These two lines ensure that when including the header in other files, is is not duplicated.
#ifndef EUFS_LIDAR101_H
#define EUFS_LIDAR101_H


// cpp
#include <string>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// transforms
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

// perception_pcl
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include <pcl/common/common.h>
#include <pcl/conversions.h>


namespace lidar101 {

    class Lidar : /* [1] Inherit from rclcpp::Node */ {
    public:
        // [2] Declare the class constructor. This function should take no arguments.

    private:
        // callbacks
        // [3] Declare a callback function called velodyne_callback to take a sensor_msgs::msg::PointCloud2::SharedPtr
        //      argument. Use the declaration below as a guide.

        void parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

        // publishers and subscribers
        // [4] Declare a publisher to publish your pointcloud. The message type should be able to take a pointcloud
        //      so be sure to use the correct type.
        //      Hint: This is very similar to declaring a subscriber as done below.

        // [5] Declare a subscriber to subscribe to the /velodyne_points messages. Make sure the message type is
        //      the right one to use. You can use the line below as a guide.
        rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

        // Transforms
        rclcpp::Clock::SharedPtr clock;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // params
        // [6] Declare variables to hold the radius and base footprint frame here.
        //      Call them RADIUS_ and base_footprint_frame_ respectively and be mindful of the types.
        //      You can always look in the config/params.yaml file to find out the types. ROS parameters
        //      are usually very sensitive to types. For example, if you try set a float to an int value
        //      using 'ros2 param set <node name> <param name> <value>' in the terminal, the node will crash.

    };

} // lidar101

#endif //EUFS_LIDAR101_H
