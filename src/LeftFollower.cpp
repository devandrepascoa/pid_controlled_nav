//
// Created by andrepascoa on 08/11/20.
//

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include "pid_controlled_nav/LeftFollower.hpp"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <cmath>


const std::map<std::string, double> car_brain::velocities = {{"fast",   2.0},
                                                             {"medium", 1.5},
                                                             {"slow",   1}};

void car_brain::LeftFollower::scanCallback(const sensor_msgs::LaserScan &msg) {
    double b_index = ((PERPENDICULAR_ANGLE) - msg.angle_min) / msg.angle_increment;
    double a_index = ((PERPENDICULAR_ANGLE + WALL_ANGLE) - msg.angle_min) / msg.angle_increment;

    polarCoordinate b = {PERPENDICULAR_ANGLE, msg.ranges[msg.ranges.size() - b_index]};
    polarCoordinate a = {PERPENDICULAR_ANGLE + WALL_ANGLE, msg.ranges[msg.ranges.size() - a_index]};
    double angle_wall = atan((a.distance * cos(WALL_ANGLE) - b.distance) / (a.distance * sin(WALL_ANGLE)));
    double distance_wall = b.distance * cos(angle_wall);
    double future_distance_wall = distance_wall + LOOKAHEAD_DISTANCE * sin(angle_wall);

//    ROS_INFO("DISTANCE: %f", future_distance_wall);
    double current_time = ros::Time::now().toSec();
    double dt = current_time - prev_time;
    prev_time = current_time;
    double steeringAngle = pid.calculate(DESIRED_DISTANCE, future_distance_wall, dt);
    ROS_INFO("ANGLE: %f", steeringAngle);
    double speed = getSpeedFromSteeringAngle(steeringAngle);
    pubDriveData(speed, steeringAngle);
}


car_brain::LeftFollower::LeftFollower() : pid(PID(KP, KI, KD, 4.0, -4.0)),
                                          prev_time(ros::Time::now().toSec()) {
    //Initializes publisher for sending data to the /nav topic
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 1);

    //Initializes subscriber for retrieving data from the /scan topic, and calls the scanCallback accordingly
    //also binds the subscriber to the current instance of this object for transporting data around
    scan_sub = n.subscribe("/scan", 1, &car_brain::LeftFollower::scanCallback, this);
}

void car_brain::LeftFollower::pubDriveData(double speed, double steeringAngle) {
    ackermann_msgs::AckermannDriveStamped ack_pub_msg; //Data for stopping vehicle

    ack_pub_msg.header.stamp = ros::Time::now();
    ack_pub_msg.header.frame_id = "laser";
    ack_pub_msg.drive.speed = speed;
    ack_pub_msg.drive.steering_angle = steeringAngle;
    drive_pub.publish(ack_pub_msg);
}


double car_brain::LeftFollower::getSpeedFromSteeringAngle(double steeringAngle) {
    double steeringAngleDegrees = abs((180 / M_PI) * steeringAngle);

    if (steeringAngleDegrees > 0 && steeringAngleDegrees < 10) {
        return velocities.at("fast");
    } else if (steeringAngleDegrees >= 10 && steeringAngleDegrees < 20) {
        return velocities.at("medium");
    }
    return velocities.at("slow");
}
