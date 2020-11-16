cmake_minimum_required(VERSION 3.0.2)
project(pid_controlled_nav)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        ackermann_msgs
        geometry_msgs
        nav_msgs
        )


catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(auto_nav_pid src/main.cpp src/LeftFollower.cpp include/pid_controlled_nav/LeftFollower.hpp include/pid_controlled_nav/PID.hpp src/PID.cpp)
target_link_libraries(auto_nav_pid ${catkin_LIBRARIES})

