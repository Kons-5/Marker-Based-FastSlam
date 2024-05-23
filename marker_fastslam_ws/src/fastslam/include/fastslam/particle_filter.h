#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Eigen/Dense>
#include <tf2/utils.h>
#include <vector>
#include <random>
#include <cmath>
#include <tuple>
#include <map>

#define NUM_PARTICLES 100

struct LandmarkObs {
    int id;
    Eigen::Vector3d position;         // 3D position vector (x, y, z)
    Eigen::Quaterniond orientation;   // Orientation quaternion (x, y, z, w)
};

struct LandmarkEst {
    int id;
    Eigen::Vector3d position;         // 3D position vector (x, y, z)
    Eigen::Quaterniond orientation;   // Orientation quaternion (x, y, z, w)
    Eigen::Matrix3d covariance;       // 3x3 covariance matrix

    // Constructor for convinience
    LandmarkEst() : id(0), position(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()), covariance(Eigen::Matrix3d::Zero()) {}
    LandmarkEst(int id, const Eigen::Vector3d& pos, const Eigen::Quaterniond& orient, const Eigen::Matrix3d& cov)
        : id(id), position(pos), orientation(orient), covariance(cov) {}
};

struct Particle {
    double weight;
    Eigen::Vector3d position;         // 3D position vector (x, y, z)
    Eigen::Quaterniond orientation;   // Orientation quaternion (x, y, z, w)
    std::map<int, LandmarkEst> landmarks;
};

class ParticleFilter {
public:
    ParticleFilter();
    void init();
    void predict(double linear_vel, double angular_vel, double timestep);
    void update(const LandmarkObs& observation);
    void resample();

private:
    int num_particles;
    std::vector<Particle> particles;
};

#endif // PARTICLE_FILTER_H
