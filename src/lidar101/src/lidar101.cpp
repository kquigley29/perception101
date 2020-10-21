/*
In this source file we define the functions we declared in the header file and give the
 functionality to our node.

The instructions are provided the same as in the header file.
*/

#include "lidar101/lidar101.h"
#include <pcl/filters/extract_indices.h>
#include <memory>


using namespace lidar101;


Lidar::Lidar()
: /* [1] Call the constructor of the rclcpp::Node class here with the node name argument. */
{
    // This line provides information to the terminal. It is useful to know for debugging.
    // The first argument comes from the rclcpp::class we inherited from. It provides a
    //  logger to log all info, warnings, and errors.
    RCLCPP_INFO(this->get_logger(),
                "Starting lidar101");

    // Here we declare the parameters and assign the values to the variable declared in the header.
    // ROS2 required parameters to be declared in order to access their values (we provide these
    //  values in config/params.yaml)
    this->RADIUS_ = this->declare_parameter("radius", 10.0);
    this->base_footprint_frame_ = this->declare_parameter("frame_to_publish_in", "base_footprint");

    // publishers and subscribers
    rclcpp::QoS queue_size = 1;

    // [2] Use the rclcpp::Node::create_publisher function to give your publisher declared in the header
    //      a value. Give it a descriptive topic name and use the queue_size variable as the second argument.
    //      The queue_size limits the number of backed-up messages waiting to be published. We usually use a
    //      size of 1 to publish the most up-to-date messages since we rely on real-time data.

    std::string sub_topic_ = this->declare_parameter("subscription_topic", "/velodyne_points");

    // [3] Use the rclcpp::Node::create_subscriber function to assign a callback to the subscription declared in
    //      the header file. Use the sub_topic_ variable from above for the topic name and queue_size for the
    //      queue size. You can use param_sub_ below as a guide.

    // param_sub_ subscribes to the '/parameter_events' topic to keep track of when parameters are updated. It uses
    //  the Lidar::parameter_callback.
    param_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>("/parameter_events",
                                                                                rclcpp::QoS(10),
                                                                                std::bind(&Lidar::parameter_callback, this, std::placeholders::_1));

    // transforms
    // Here we start a node to listen transform messages so that we can transform the received pointcloud
    //  in the callback
    clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


// This is the callback which is called when param_sub_ receives a message from the '/parameter_events' topic.
//  It updates all the parameters that have been changed.
void Lidar::parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    // Here we print information to the terminal as we did earlier in the constructor
    RCLCPP_INFO(this->get_logger(),
                "Parameter callback");

    // Here we loop through the parameters that have been changed and assign them their new values.
    for (const auto &param: event->changed_parameters) {
        if (param.name == "radius") this->get_parameter("radius", this->RADIUS_);
    }
}


void Lidar::velodyne_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Here we transform the message into the base footprint frame using the transforms received from the
    //  transform listener node started in the constructor.
    sensor_msgs::msg::PointCloud2 transformed_msg;
    pcl_ros::transformPointCloud(this->base_footprint_frame_, *msg, transformed_msg, *tf_buffer_);

    // [4] Declare a new pointcloud and convert the ROS message in transformed_msg to a pointcloud.
    //      The point cloud should be a pointer of type pcl::PointCloud<pcl::PointXYZ>.

    // [5] Loop through the points in the point cloud and remove all the points that are further from the
    //      LiDAR than the radius parameter. You should create a new pointcloud to store the points within
    //      the radius. This step involves using the PointCloudLibrary. There are various methods of accessing
    //      the old point cloud and moving/copying the points to the new point cloud.

    // [6] Declare a message object and call it close_points_msg. It will be used to publish the pointcloud
    //      within the radius.

    // [7] Convert the pointcloud into the message.

    // Here we provide information about the content of the message being published. The header part of ROS
    //  messages tell you al lot about the message such as its frame id, timestamp etc.
    close_points_msg.header.frame_id = this->base_footprint_frame_;

    // [8] Publish the message.

}
