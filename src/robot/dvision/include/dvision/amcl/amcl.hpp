#pragma once
#include <array>
#include "dmsgs/VisionInfo.h"
#include "geometry_msgs/Vector3.h"
#include "dvision/amcl/particle.hpp"
#include "dvision/amcl/map.hpp"
#include "dvision/amcl/types.hpp"
#include "dvision/amcl/tools.hpp"
#include "dvision/amcl/kdtree.hpp"

#include <vector>

namespace dvision {

struct Cluster {
    // Number of samples
    int count;

    // Total weight of samples in this cluster
    double weight;
    double meanWeight;

    // Cluster statistics
    Pose mean;
    Matrix cov;

    // Workspace
    double m[4], c[2][2];
};

struct SampleSet {
    ~SampleSet() {
        delete kdtree;
    }

    std::vector<Particle> samples;
    KdTree* kdtree = nullptr;

    // Clusters
    int cluster_count = 0;
    int cluster_max_count;
    std::vector<Cluster> clusters;

    // Filter statistics
    Pose mean;
    Matrix cov;
    bool converged = false;
};

class AMCL {
public:
    AMCL();
    void Init();

    void Process(Measurement& z, Control& u, double imu_yaw_degree, dmsgs::VisionInfo& visionInfo);
    std::vector<Particle>& partices();
    Map& map();
    Pose GetEstimatedPose();
    bool IsConverged();

    void ResetParticlesLeftTouch();
    void ResetParticlesRightTouch();
    void ResetParticlesPoint(geometry_msgs::Vector3 point);
    void falldownGauss();

private:
    void update(Measurement& z, Control& u, double imu_yaw_degree);
    void SampleMotionModel(Control& u);
    void MeasurementModel(Measurement& z);
    void Resample(Measurement& z);
//    void SensorReset(Measurement& z);
    void Estimate();
    bool CheckConverged();
    Pose RandomPose();
    void ClusterStats(SampleSet& set);

    int m_num_particles;
    int m_cycle = 0;
    SampleSet m_sets[2];
    int m_current_set = 0;
    Map m_map;

    Pose m_pose;

    double w_slow = 0.0;
    double w_fast = 0.0;

    double alpha_slow = 0.0;
    double alpha_fast = 0.0;

    double z_hit = 0.95;
    double z_rand = 0.95;
    double sigma_hit = 0.2;

    int m_resample_interval = 5;

    double dist_threshold = 50;
    bool m_converged = false;

    double  m_yaw = 0;

    Gaussian m_yGauss;
    Gaussian m_xGauss;
    Gaussian m_resampleGauss;
    Uniform  m_uniform;
};

} // namespace dvision