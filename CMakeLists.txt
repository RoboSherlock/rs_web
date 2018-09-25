cmake_minimum_required(VERSION 2.8.3)
project(rs_web)

find_package(catkin REQUIRED COMPONENTS
  robosherlock_msgs
)

catkin_python_setup()

catkin_package()
