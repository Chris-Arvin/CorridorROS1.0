cmake_minimum_required(VERSION 2.8.3)
project(fake_localization)

find_package(catkin REQUIRED
        COMPONENTS
            geometry_msgs
            rospy
            tf2_geometry_msgs
            tf2_ros
        )


find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        rospy
)

