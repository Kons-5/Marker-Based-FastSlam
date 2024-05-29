#include "fastslam/particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter() {}

void ParticleFilter::init() {
    // Initialize particles, weights, and any necessary parameters
    particles.resize(NUM_PARTICLES);

    // Initialize each particle with initial state
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].weight = 1.0 / NUM_PARTICLES; // Initialize all particles with a normalized weight
        particles[i].pose = Eigen::Matrix<double, 7, 1>::Zero();
        particles[i].landmarks.clear();
    }
}

// Function to compute yaw from quaternion
double ParticleFilter::get_yaw(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    return yaw;
}

void ParticleFilter::predict(const double linear_vel, const double angular_vel, const double timestep) {
    // Implement the prediction step based on motion model
    // Update each particle's position and orientation
    for (auto& particle : particles) {
        // Modify particle.pose based on the motion model
        double old_x = particle.pose[0];
        double old_y = particle.pose[1];
        double old_z = particle.pose[2];
        
        Eigen::Quaterniond q(particle.pose[3], particle.pose[4], particle.pose[5], particle.pose[6]);
        double old_theta = ParticleFilter::get_yaw(q);
        
        // Calculate the approximate change after the elapsed time
        double new_x, new_y, new_theta;
        if (fabs(angular_vel) > 1e-5) {
            // General case
            new_x = old_x - (linear_vel / angular_vel) * sin(old_theta) + (linear_vel / angular_vel) * sin(old_theta + angular_vel * timestep);
            new_y = old_y + (linear_vel / angular_vel) * cos(old_theta) - (linear_vel / angular_vel) * cos(old_theta + angular_vel * timestep);
            new_theta = angular_vel * timestep;
        } else {
            // Straight-line motion case
            new_x = old_x + linear_vel * cos(old_theta) * timestep;
            new_y = old_y + linear_vel * sin(old_theta) * timestep;
            new_theta = old_theta;  // Does not change
        }
        
        // Create a quaternion representing the yaw rotation
        Eigen::Quaterniond new_q(Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ()));

        // Update the particle's position and orientation
        particle.pose << new_x, new_y, old_z, new_q.w(), new_q.x(), new_q.y(), new_q.z(); // The robot has planar motion, dz/dt = 0
    }
}

// Function to convert a quaternion into a 4x4 quaternion multiplication matrix
Eigen::Matrix4d ParticleFilter::quaternion_mult_matrix(const Eigen::Quaterniond& q) {
    // Ensure the quaternion is normalized
    Eigen::Quaterniond q_normalized = q.normalized();

    // Extract the quaternion components
    double w = q_normalized.w();
    double x = q_normalized.x();
    double y = q_normalized.y();
    double z = q_normalized.z();

    // Compute the quaternion multiplication matrix
    Eigen::Matrix4d Q;
    Q << w, -x, -y, -z,
         x,  w, -z,  y,
         y,  z,  w, -x,
         z, -y,  x,  w;

    return Q;
}

// Computes the transform from the camera's coordinate frame to the global coordinate frame
Eigen::Matrix<double, 7, 1> ParticleFilter::transform_to_global(const Eigen::Matrix<double, 7, 1>& particle, const Eigen::Matrix<double, 7, 1>& landmark, double pan, const double tilt) {
    // Robot orientation rotation matrix
    Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Quaterniond q_robot(particle[3], particle[4], particle[5], particle[6]);
    A.block<3, 3>(0, 0) = q_robot.toRotationMatrix();
    A.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_robot);
    
    // Pan rotation matrix
    Eigen::Matrix<double, 7, 7> B = Eigen::Matrix<double, 7, 7>::Zero();
    B.block<3, 3>(0, 0) << cos(pan), -sin(pan), 0,
                           sin(pan),  cos(pan), 0,
                           0,         0,        1;
                           
    Eigen::Quaterniond q_pan(Eigen::AngleAxisd(pan, Eigen::Vector3d::UnitZ()));
    B.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_pan);
    
    // Tilt rotation matrix
    Eigen::Matrix<double, 7, 7> C = Eigen::Matrix<double, 7, 7>::Zero();
    C.block<3, 3>(0, 0) << cos(tilt), 0, sin(tilt),
                           0,         1,         0,
                          -sin(tilt), 0, cos(tilt);
             
    Eigen::Quaterniond q_tilt(Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()));             
    C.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_tilt);
    
    Eigen::Matrix<double, 7, 1> translation;
    translation << particle[0], particle[1], particle[2], 0, 0, 0, 0;
    
    // Final transformation (translation + rotations)
    return translation + A * B * C * landmark;
}

// Computes the jacobian of the measurement function
Eigen::Matrix<double, 7, 7> ParticleFilter::compute_jacobian(const Eigen::Matrix<double, 7, 1>& particle, const double pan, const double tilt) {
    // Robot orientation rotation matrix
    Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Quaterniond q_robot(particle[3], particle[4], particle[5], particle[6]);
    A.block<3, 3>(0, 0) = q_robot.toRotationMatrix();
    A.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_robot);
    
    // Pan rotation matrix
    Eigen::Matrix<double, 7, 7> B = Eigen::Matrix<double, 7, 7>::Zero();
    B.block<3, 3>(0, 0) << cos(pan), -sin(pan), 0,
                           sin(pan),  cos(pan), 0,
                           0,         0,        1;
                           
    Eigen::Quaterniond q_pan(Eigen::AngleAxisd(pan, Eigen::Vector3d::UnitZ()));
    B.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_pan);
    
    // Tilt rotation matrix
    Eigen::Matrix<double, 7, 7> C = Eigen::Matrix<double, 7, 7>::Zero();
    C.block<3, 3>(0, 0) << cos(tilt), 0, sin(tilt),
                           0,         1,         0,
                          -sin(tilt), 0, cos(tilt);
             
    Eigen::Quaterniond q_tilt(Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()));             
    C.block<4, 4>(3, 3) = ParticleFilter::quaternion_mult_matrix(q_tilt);
    
    return C.transpose() * B.transpose() * A.transpose();
}

// Computes the transform from the global coordinate frame to the camera's coordinate frame
// This function effectively outputs the "expected" observation based on the stored belief
Eigen::Matrix<double, 7, 1> ParticleFilter::measurement_function(const Eigen::Matrix<double, 7, 1>& particle, const Eigen::Matrix<double, 7, 1>& landmark, const double pan, const double tilt) {
    Eigen::Matrix<double, 7, 1> translation;
    translation << particle[0], particle[1], particle[2], 0, 0, 0, 0;
    
    return ParticleFilter::compute_jacobian(particle, pan, tilt) * (landmark - translation);
}

void ParticleFilter::update(const LandmarkObs& observation, const double pan, const double tilt) {
    // Implement the update step based on sensor measurements
    // Update each particle's weight based on the likelihood of observed measurements
    for (auto& particle : particles) {
        auto& landmarks = particle.landmarks;
        // TODO: Change!!!
        Eigen::Matrix<double, 7, 7> R = Eigen::Matrix<double, 7, 7>::Identity();

        if (landmarks.find(observation.id) == landmarks.end()) {
            // Initialize the landmark if it has never been seen
            Eigen::Matrix<double, 7, 1> landmark_global = ParticleFilter::transform_to_global(particle.pose, observation.pose, pan, tilt);

            Eigen::Matrix<double, 7, 7> G = ParticleFilter::compute_jacobian(particle.pose, pan, tilt);
            Eigen::Matrix<double, 7, 7> G_inv = G.inverse();
            Eigen::Matrix<double, 7, 7> covariance = G_inv * R * G_inv.transpose();

            // Use emplace to construct the object in place
            landmarks.emplace(observation.id, LandmarkEst{observation.id, landmark_global, covariance});
        } else {
            // Update seen landmarks
            auto& landmark = landmarks[observation.id];

            // Calculate the Kalman gain
            Eigen::Matrix<double, 7, 7> G = ParticleFilter::compute_jacobian(particle.pose, pan, tilt);
            Eigen::Matrix<double, 7, 7> Q = G * landmark.covariance * G.transpose() + R;
            Eigen::Matrix<double, 7, 7> Q_inv = Q.inverse();
            Eigen::Matrix<double, 7, 7> K = landmark.covariance * G.transpose() * Q_inv;

            // Update the landmark position (innovation)
            Eigen::Matrix<double, 7, 1> expected_observation = measurement_function(particle.pose, landmark.pose, pan, tilt);
            Eigen::Matrix<double, 7, 1> residual = observation.pose - expected_observation;
            Eigen::Matrix<double, 7, 1> scaled_residual = K * residual;
            landmark.pose += scaled_residual;

            // Update the covariance of this landmark
            Eigen::Matrix<double, 7, 7> I = Eigen::Matrix<double, 7, 7>::Identity();
            landmark.covariance = (I - K * G) * landmark.covariance;

            // Update the weight of the particle
            double det_Q = Q.determinant();
            double norm_factor = pow(2 * M_PI * det_Q, -0.5);
            double exponent = -0.5 * residual.transpose() * Q_inv * residual;
            particle.weight *= norm_factor * exp(exponent);
        }
    }
}

void ParticleFilter::resample() {
    // Create a new set of particles
    vector<Particle> new_particles(NUM_PARTICLES);

    // Calculate the sum of all weights
    double weight_sum = 0.0;
    for (const auto& particle : particles) {
        weight_sum += particle.weight;
    }

    // Normalize weights
    for (auto& particle : particles) {
        particle.weight /= weight_sum;
    }

    // Initialize random number generator
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dist(0.0, 1.0);

    // Low variance resampling
    double r = dist(gen) / NUM_PARTICLES;
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; ++m) {
        double U = r + m * (1.0 / NUM_PARTICLES);
        while (U > c) {
            i = (i + 1) % NUM_PARTICLES;
            c += particles[i].weight;
        }
        new_particles[m] = particles[i];
        new_particles[m].weight = 1.0 / NUM_PARTICLES;  // Reset the weight of the new particle
    }

    // Replace old particles with the new resampled particles
    particles = new_particles;
}
