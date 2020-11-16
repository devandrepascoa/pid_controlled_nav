//
// Created by andrepascoa on 08/11/20.
//

#ifndef CAR_BRAINZ_LEFT_FOLLOWER_HPP
#define CAR_BRAINZ_LEFT_FOLLOWER_HPP

#include "ros/ros.h"
#include "PID.hpp"
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>


namespace car_brain {
    /**
     * Safety class containing all the safety features of the car
     * TODO
     * Add offset based on lidar position and car dimensions
     */
    struct polarCoordinate {
        double theta;
        double distance;
    };

    const double WALL_ANGLE = 0.9; //50 degrees in radians
    const double PERPENDICULAR_ANGLE = -M_PI / 2.0; //Perpendicular angle relative to the car X axis, to the left
    const double LOOKAHEAD_DISTANCE = 0.5;
    const double DESIRED_DISTANCE = 0.75;
    const double KP = 1.0;
    const double KI = 0.0;
    const double KD = 0.0;
    extern const std::map<std::string, double> velocities;

    class LeftFollower {
    private:
        ros::Publisher drive_pub; /**< Braking publisher, for sending stop drive data */
        ros::Subscriber scan_sub; /**< Scan Subscriber, for retrieving LiDAR data */
        ros::NodeHandle n; /**< Interface to access this process Node and create the subscribers, publishers, etc..*/
        double prev_time;
        PID pid;

        /**
         *@brief This method will be called everytime someone publishes to the /scan topic
         *
         * @param msg, message containing our LiDAR data and it's specification
         */
        void scanCallback(const sensor_msgs::LaserScan &msg);


    public:
        /**
         * Class constructor, starts the data retrieval from topics using subscribers, and also initializes the publisher
         * types.
         */
        LeftFollower();

        void pubDriveData(double speed, double steeringAngle);

        static double getSpeedFromSteeringAngle(double steeringAngle);

    };

};
#endif //CAR_BRAINZ_LEFT_FOLLOWER_HPP
