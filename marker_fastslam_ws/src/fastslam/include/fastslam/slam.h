#ifndef SLAM_H
#define SLAM_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "fastslam/particle_filter.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#define ROS_RATE 10 // Frequency of 10 Hz
#define MAX_SUBSCRIBER_QUEUE_SIZE 1000

// Function declarations for the callbacks
void motion_callback(const geometry_msgs::Twist::ConstPtr& msg);
void observ_callback(const fiducial_msgs::FiducialTransform::ConstPtr& msg);

#endif // SLAM_H
