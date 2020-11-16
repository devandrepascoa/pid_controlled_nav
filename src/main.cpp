//
// Created by andrepascoa on 08/11/20.
//
#include "ros/ros.h"
#include "pid_controlled_nav/LeftFollower.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "LeftFollower");
    car_brain::LeftFollower lefty;
    ros::spin();

    return 0;
}