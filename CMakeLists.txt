cmake_minimum_required(VERSION 3.0.0)
project(ars_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs tf_conversions tf2_ros python-rospkg ars_lib_helpers
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
