#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

print ('Starting script...')

# Global variable to store the robot's current pose

current_pose = None


# Callback function to update the current pose of the robot

def update_pose(data):
    global current_pose
    current_pose = data.pose.pose


# Function to execute motion and return new pose

def execute_motion(u, delta_t):

    global cmd_vel_publisher
    vel_msg = Twist()

    # Initial wake-up command

    cmd_vel_publisher.publish(Twist())
    rospy.sleep(0.5)  # Short pause before sending the actual command

    vel_msg.linear.x = u[0]
    vel_msg.angular.z = u[1]
    cmd_vel_publisher.publish(vel_msg)
    rospy.loginfo('Published velocity command')

    # Wait for the motion duration

    time.sleep(delta_t)

    # Stop the robot

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    cmd_vel_publisher.publish(vel_msg)
    rospy.loginfo('Published stop command')

    # Wait for the pose to update

    time.sleep(1)  # Adjust time as necessary for your system

    print (current_pose)

    return current_pose


# Function to collect data and estimate parameters

def collect_data_and_estimate_parameters():

    print ('Setting up variables...')
    num_samples = 2  # Number of samples to collect
    N_move = 10  # Number of motion samples

    commanded_vs = np.linspace(-0.5, 0.5, num_samples)  # Linear velocities
    commanded_omegas = np.linspace(-0.5, 0.5, num_samples)  # Angular velocities

    delta = 2 #(0.5 - -0.5) / num_samples  # Time for motion execution

    measured_vs = []
    measured_omegas = []
    arr_sigma_v = []
    arr_sigma_w = []
  
    print ('Variables set. fetching init pose...')
    initial_pose = current_pose  # Starting pose

    print ('Entering estimation loop...')
    for v in commanded_vs:
        for omega in commanded_omegas:
            sigma_v = 0
            sigma_w = 0
            for k in range(N_move):

            # Execute motion for 2 seconds

                final_pose = execute_motion((v, omega), delta)

            # Find distance

                dx = final_pose.position.x - initial_pose.position.x
                dy = final_pose.position.y - initial_pose.position.y

            # Calculate velocity and orientation

                v_x = dx / delta
                v_y = dy / delta
                theta = math.atan2(v_y, v_x)

                estimated_v = np.sqrt(v_y ** 2 + v_x ** 2)
                estimated_omega = theta / delta

            # Calculate error squared

                sigma_v += (estimated_v - v) ** 2
                sigma_w += (estimated_omega - omega) ** 2

            # Update initial pose for next iteration

                initial_pose = final_pose

            arr_sigma_v.append(1 / (N_move) * sigma_v)
            arr_sigma_w.append(1 / (N_move) * sigma_w)

    print ('Estimated v variance parameters:', np.sum(arr_sigma_v)/len(arr_sigma_v))
    print ('Estimated w variance parameters:', np.sum(arr_sigma_w)/len(arr_sigma_w))


try:
    print ('Initializing node...')
    rospy.init_node('alphabot2_motion_tester')

    print ('Setting up publisher...')
    cmd_vel_publisher = rospy.Publisher('/alphabot2/control', Twist,
            queue_size=2)

    print ('Setting up subscriber...')
    odom_subscriber = rospy.Subscriber('/odom', Odometry, update_pose)

    print ('Node setup complete. Starting parameter estimation...')
    rospy.wait_for_message('/odom', Odometry, timeout=None)
    print (current_pose)
    collect_data_and_estimate_parameters()
except (exception, e):

    print ('Encountered an error: ', e)
finally:
    print ('Shutting down node.')

