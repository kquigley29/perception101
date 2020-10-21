#Exercise 1
##Creating a node and processing a pointcloud

1) Clone the repository
2) Look through `CMakeLists.txt` and familiarise yourself with all the commands
3) Look at `include/lidar101.h`. This is a cpp header file for the `lidar101` node and contains the declarations for the Lidar class to use.
4) Follow the instructions in the comments of `lidar101.h`, ask any questions if you can't do anything
5) Look at `src/lidar101.cpp`, this contains the implementations of what you declared in the header file
6) Follow the instructions in the comments of `lidar101.cpp`, ask any questions if you're stuck
7) Finally initiate the node in the main function, follow the instructions in `lidar101_node.cpp`

    ####Overview of `Lidar101`
    - We create a Lidar class which inherits from a `rclcpp::Node`. 
    - The node must create a subscriber to the `/velodyne_points` topic - this contains pointcloud messages
    - The node must create a publisher to a topic name to publish the manipulated pointcloud
    - The node must have a callback function which is called when the subscriber receives a message
    - The callback takes a `sensor_msgs::msg::PointCloud2::SharedPtr` argument, this is the message type sent on `/velodyne_points` and contains a pointer to the pointcloud message
    - It then iterates through the points in the pointcloud (compare this to `lidar_grid` code) and removes anything not inside a preset radius
    - Finally it publishes a `sensor_msgs::msg::PointCloud2` containing the ammended pointcloud data
 
Useful Links:
- [PointCloudLibrary Documentation](https://pointclouds.org/documentation/)
- [CMake wiki](https://gitlab.kitware.com/cmake/community/-/wikis/Home)
- The lidar_grid source code! Lots of it is transferable to this exercise, make sure to understand the code you copy, ask about anything you don't understand
