#ifndef SLAM_H
#define SLAM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "fastslam/particle_filter.h"
#include "waveshare_alphabot2/Pan_Tilt.h"
#include "fiducial_msgs/FiducialTransform.h"

#define ROS_RATE 10 // Frequency of 10 Hz
#define MAX_SUBSCRIBER_QUEUE_SIZE 1000
#define MAX_PUBLISHER_QUEUE_SIZE 1000

extern ros::Publisher pub1;
extern ros::Publisher pub2;

// Function declarations for the callbacks
void motion_callback(const geometry_msgs::Twist::ConstPtr& msg);
void pantilt_callback(const waveshare_alphabot2::Pan_Tilt::ConstPtr& msg);
void observ_callback(const fiducial_msgs::FiducialTransform::ConstPtr& msg);

#endif // SLAM_H
