#include "fastslam/slam.h"

// Global ParticleFilter object
ParticleFilter particle_filter;

void motion_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received motion control data");
    
    // Extract linear and angular velocities from the message
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    double timestep = 1.0 / ROS_RATE;
    
    // Call the predict function of the particle filter with motion data
    particle_filter.predict(linear_vel, angular_vel, timestep); // Assuming variances are 0.1 for both
}

void observ_callback(const fiducial_msgs::FiducialTransform::ConstPtr& msg) {
    ROS_INFO("Received fiducial transforms data");
    
    // Create a LandmarkObs object from the received fiducial transform
    LandmarkObs observation;
    observation.id = msg->fiducial_id;
    observation.position = Eigen::Vector3d(
        msg->transform.translation.x,
        msg->transform.translation.y,
        msg->transform.translation.z
    );
    observation.orientation = Eigen::Quaterniond(
        msg->transform.rotation.w,
        msg->transform.rotation.x,
        msg->transform.rotation.y,
        msg->transform.rotation.z
    );

    // Call the update function of the particle filter with observation data
    particle_filter.update(observation);
    
    // Perform the resampling step after updating
    particle_filter.resample();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_node");
    ros::NodeHandle nh;

    // Initialize the particle filter
    particle_filter.init();

    // Subscribe and create publishers for all relevant topics
    ros::Subscriber sub1 = nh.subscribe("/alphabot2/control", MAX_SUBSCRIBER_QUEUE_SIZE, motion_callback);
    ros::Subscriber sub2 = nh.subscribe("/fiducial_transforms", MAX_SUBSCRIBER_QUEUE_SIZE, observ_callback);

    ROS_INFO("Waiting for data...");

    ros::Rate rate(ROS_RATE); // 30 Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
