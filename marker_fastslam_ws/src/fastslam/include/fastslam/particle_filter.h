#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <cmath>
#include <map>

#define NUM_PARTICLES 100

struct LandmarkObs {
    int id;
    Eigen::Matrix<double, 7, 1> pose;       // 7D pose vector (x, y, z, quaternion)
};

struct LandmarkEst {
    int id;
    Eigen::Matrix<double, 7, 1> pose;       // 7D pose vector (x, y, z, quaternion)
    Eigen::Matrix<double, 7, 7> covariance; // 7x7 covariance matrix

    // Constructor for convinience
    LandmarkEst() : id(0), pose(Eigen::Matrix<double, 7, 1>::Zero()), covariance(Eigen::Matrix<double, 7, 7>::Zero()) {}
    LandmarkEst(int id, const Eigen::Matrix<double, 7, 1>& pose, const Eigen::Matrix<double, 7, 7>& cov)
        : id(id), pose(pose), covariance(cov) {}
};

struct Particle {
    double weight;
    Eigen::Matrix<double, 7, 1> pose;       // 7D pose vector (x, y, z, quaternion)
    std::map<int, LandmarkEst> landmarks;
};

class ParticleFilter {
public:
    ParticleFilter();
    void init();
    void predict(const double linear_vel, const double angular_vel, const double timestep);
    void update(const LandmarkObs& observation, const double pan, const double tilt);
    void resample();

private:
    int num_particles;
    std::vector<Particle> particles;
    double get_yaw(const Eigen::Quaterniond& q);
    Eigen::Matrix4d quaternion_mult_matrix(const Eigen::Quaterniond& q);
    Eigen::Matrix<double, 7, 1> transform_to_global(const Eigen::Matrix<double, 7, 1>& particle, const Eigen::Matrix<double, 7, 1>& landmark, double pan, const double tilt);
    Eigen::Matrix<double, 7, 1> measurement_function(const Eigen::Matrix<double, 7, 1>& particle, const Eigen::Matrix<double, 7, 1>& landmark, const double pan, const double tilt);
    Eigen::Matrix<double, 7, 7> compute_jacobian(const Eigen::Matrix<double, 7, 1>& particle, const double pan, const double tilt);
};

#endif // PARTICLE_FILTER_H
