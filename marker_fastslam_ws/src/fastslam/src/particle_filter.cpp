#include "fastslam/particle_filter.h"

using namespace std;

ParticleFilter::ParticleFilter() {}

void ParticleFilter::init() {
    // Initialize particles, weights, and any necessary parameters
    particles.resize(NUM_PARTICLES);

    // Initialize each particle with initial state
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].weight = 1.0 / NUM_PARTICLES; // Initialize all particles with a normalized weight
        particles[i].position = Eigen::Vector3d::Zero();
        particles[i].orientation = Eigen::Quaterniond::Identity();
        particles[i].landmarks.clear();
    }
}

// Helper function to get yaw from Eigen::Quaterniond
double getYaw(const Eigen::Quaterniond& q) {
    // Conversion to roll-pitch-yaw (ZYX order)
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();
    double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    return yaw;
}

void ParticleFilter::predict(double linear_vel, double angular_vel, double timestep) {
    // Implement the prediction step based on motion model
    // Update each particle's position and orientation
    for (auto& particle : particles) {
        // Modify particle.position and particle.orientation based on the motion model
        double old_x = particle.position.x();
        double old_y = particle.position.y();
        double old_z = particle.position.z();
        double old_theta = getYaw(particle.orientation);
        
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
        
        // Update the particle's position and orientation
        particle.position = Eigen::Vector3d(new_x, new_y, old_z); // The robot has planar motion, dz/dt = 0
        particle.orientation = Eigen::AngleAxisd(new_theta, Eigen::Vector3d::UnitZ());
    }
}

void ParticleFilter::update(const LandmarkObs& observation) {
    // TODO: Change!!!
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d G = Eigen::Matrix3d::Identity();

    // Implement the update step based on sensor measurements
    // Update each particle's weight based on the likelihood of observed measurements
    for (auto& particle : particles) {
        auto& landmarks = particle.landmarks;

        if (landmarks.find(observation.id) == landmarks.end()) {
            // Initialize the landmark if it has never been seen
            Eigen::Vector3d lm_pos = particle.position + observation.position;

            Eigen::Matrix3d G_inv = G.inverse();
            Eigen::Matrix3d covariance = G_inv * R * G_inv.transpose();

            // Use emplace to construct the object in place
            landmarks.emplace(observation.id, LandmarkEst{observation.id, lm_pos, observation.orientation, covariance});
        } else {
            // Update seen landmarks
            auto& landmark = landmarks[observation.id];
            Eigen::Vector3d residual = observation.position - (landmark.position - particle.position);

            // Calculate the Kalman gain
            Eigen::Matrix3d Q = G * landmark.covariance * G.transpose() + R;
            Eigen::Matrix3d Q_inv = Q.inverse();
            Eigen::Matrix3d K = landmark.covariance * G.transpose() * Q_inv;

            // Update the landmark position
            Eigen::Vector3d scaled_residual = K * residual;
            landmark.position += scaled_residual;

            // Update the covariance of this landmark
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
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
    std::vector<Particle> new_particles(NUM_PARTICLES);

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
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

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
