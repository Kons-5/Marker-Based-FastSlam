#include "fastslam/slam.h"

// Global ParticleFilter object
ParticleFilter particle_filter;

// Global pan and tilt angles
double pan = 0.0, tilt = 0.0; 

void motion_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received motion control data");
    
    // Extract linear and angular velocities from the message
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    double timestep = 1.0 / ROS_RATE;
    
    // Call the predict function of the particle filter with motion data
    particle_filter.predict(linear_vel, angular_vel, timestep);
}

void pantilt_callback(const waveshare_alphabot2::Pan_Tilt::ConstPtr& msg) {
    ROS_INFO("Received pan and tilt data");
    
    pan = msg->pan;
    tilt = msg->tilt;
}

void observ_callback(const fiducial_msgs::FiducialTransform::ConstPtr& msg) {
    ROS_INFO("Received fiducial transforms data");
    
    // Create a LandmarkObs object from the received fiducial transform
    LandmarkObs observation;
    
    // Attribute the unique identifier
    observation.id = msg->fiducial_id;
    
    // Create the pose vector
    observation.pose << 
        msg->transform.translation.x,
        msg->transform.translation.y,
        msg->transform.translation.z,
        msg->transform.rotation.w,
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z;

    // Call the update function of the particle filter with observation data
    particle_filter.update(observation, pan, tilt);
    
    // Perform the resampling step after updating
    particle_filter.resample();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle nh;

    // Initialize the particle filter
    particle_filter.init();

    // Subscribe and create publishers for all relevant topics
    ros::Subscriber sub1 = nh.subscribe("/cmd_vel", MAX_SUBSCRIBER_QUEUE_SIZE, motion_callback);
    ros::Subscriber sub2 = nh.subscribe("/pan_tilt", MAX_SUBSCRIBER_QUEUE_SIZE, pantilt_callback);
    ros::Subscriber sub3 = nh.subscribe("/fiducial_transforms", MAX_SUBSCRIBER_QUEUE_SIZE, observ_callback);

    // pub1 = nh.advertise<geometry_msgs::PoseArray>("/particles", MAX_PUBLISHER_QUEUE_SIZE);
    // pub2 = nh.advertise<geometry_msgs::PoseArray>("/landmarks", MAX_PUBLISHER_QUEUE_SIZE);

    ROS_INFO("Waiting for data...");

    ros::Rate rate(ROS_RATE); // 30 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
