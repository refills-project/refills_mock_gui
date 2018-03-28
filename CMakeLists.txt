cmake_minimum_required(VERSION 2.8.3)
project(refills_mock_gui)

find_package(catkin REQUIRED COMPONENTS rospy refills_msgs actionlib json_prolog knowrob_refills)
catkin_python_setup()
catkin_package()
include_directories()
