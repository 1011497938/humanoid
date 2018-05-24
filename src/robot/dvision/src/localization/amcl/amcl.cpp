#include "dvision/amcl/amcl.hpp"
#include "dvision/utils.hpp"
#include <dvision/timer.hpp>
#include "dvision/parameters.hpp"
using namespace std;

// FIXME(MWX): if all particles converged, but weight are small, then there's no difference between using 10 samples or 10000 samples

namespace dvision {

AMCL::AMCL()
    :
      m_current_set(0),
      m_converged(false),
      // don't change this!!!
      m_yGauss(0, .3),
      m_xGauss(0, .5),
      m_resampleGauss(0, 5),
      m_uniform(-1, 1)
{
}


void AMCL::Init() {
    m_num_particles = 300;
    m_resample_interval = 30;

//    m_num_particles = parameters.amcl.num_particles;
//    m_resample_interval = parameters.amcl.resample_interval;
    dist_threshold = parameters.amcl.dist_threshold;
    z_hit = parameters.amcl.z_hit;
    z_rand = parameters.amcl.z_rand;
    sigma_hit = parameters.amcl.sigma_hit;
    m_map.Init();

    for(int i = 0; i < 2; ++i) {
        auto& set = m_sets[i];

        set.samples.resize(m_num_particles);
        set.kdtree = new KdTree(3 * m_num_particles);
        set.cluster_max_count = m_num_particles;
        set.clusters.resize((size_t)set.cluster_max_count);

        set.mean = Pose(0, 0, 0);
        set.cov = Matrix();
    }

    auto& set = m_sets[m_current_set];

    set.kdtree->clear();

    for(auto& sample : set.samples) {
        sample.weight = 1.0 / set.samples.size();
        sample.pose = RandomPose();

        set.kdtree->InsertPose(sample.pose, sample.weight);
    }
    ClusterStats(set);
}

void AMCL::Process(Measurement &z, Control &u, double imu_yaw_degree, dmsgs::VisionInfo &visionInfo) {
    Timer t;
    update(z, u, imu_yaw_degree);

    // put result into visionInfo
    auto& particles = this->partices();
    visionInfo.particles.resize(particles.size());

    for (uint32_t i = 0; i < particles.size(); ++i) {
        visionInfo.particles[i].pose.x = particles[i].pose.x();
        visionInfo.particles[i].pose.y = particles[i].pose.y();
        visionInfo.particles[i].pose.z = particles[i].pose.heading();
        visionInfo.particles[i].weight = particles[i].weight;
    }

    visionInfo.loc_ok = m_converged;
    visionInfo.robot_pos.x = m_pose.x();
    visionInfo.robot_pos.y = m_pose.y();
    visionInfo.robot_pos.z = m_pose.heading();
    ROS_DEBUG("AMCL used: %lf ms", t.elapsedMsec());
}

void AMCL::update(Measurement& z, Control& u, double imu_yaw_degree) {
    m_yaw = imu_yaw_degree;
    SampleMotionModel(u);
    MeasurementModel(z);

    if (!(++m_cycle % m_resample_interval) || !z.centerPoints.empty() || (z.goalPosts.size() == 2)) {
        Resample(z);
//        Estimate();
//        CheckConverged();
        m_cycle = 0;
    }

    if(!(m_cycle % 5)) {
        Estimate();
        CheckConverged();
    }
}

void AMCL::SampleMotionModel(Control& u) {
    double dx = u.dx;
    double dy = u.dy;

    double w = dconstant::geometry::fieldLength / 2 + dconstant::geometry::borderStripWidth;
    double h = dconstant::geometry::fieldWidth / 2 + dconstant::geometry::borderStripWidth;

    for (auto &particle : this->partices()) {
        Pose &p = particle.pose;
        double ddx = dx + m_xGauss.sample();
        double ddy = dy + m_yGauss.sample();

        double heading = Degree2Radian(p.heading());
        if(parameters.simulation) {
            p.setX(p.x() + ddx * std::cos(heading) - ddy * std::sin(heading));
            p.setY(p.y() + ddx * std::sin(heading) + ddy * std::cos(heading));
        } else {
            p.setX(p.x() + ddx);
            p.setY(p.y() + ddy);
        }
        p.setHeading(m_yaw);

        p.setX(std::min(p.x(), w));
        p.setX(std::max(p.x(), -w));
        p.setY(std::min(p.y(), h));
        p.setY(std::max(p.y(), -h));
    }
}

void AMCL::MeasurementModel(Measurement& z) {
    double total_weight = 0;
    auto &set = m_sets[m_current_set];
    for (auto &sample : set.samples) {
        double prob = sample.weight;
        double fuck = 0.0;
        cv::Point3d gParticle(sample.pose.x(), sample.pose.y(), sample.pose.heading());

        if(z.whitePoints.size() > 10) {
            for (auto& whiteP: z.whitePoints) {
                cv::Point2d gWhitePoint = getOnGlobalCoordinate(gParticle, cv::Point2d(whiteP.x, whiteP.y));
                double dist = m_map.getDist(gWhitePoint.x, gWhitePoint.y);
                double pz = normal_pdf(dist, 0.0, sigma_hit);
                fuck += pz * pz * pz;
            }
            prob += fuck;
        }

        for(auto& center: z.centerPoints) {
            cv::Point2d gCenter = getOnGlobalCoordinate(gParticle, cv::Point2d(center.x, center.y));
            // divide by 5.0, like in occ dist map
            auto dist = sqrt(gCenter.x * gCenter.x + gCenter.y * gCenter.y) / 5.0;
            dist = std::min(dist, 5.0);
            double pz = normal_pdf(dist, 0.0, sigma_hit);
            prob += pz * pz * pz * z.whitePoints.size();
        }

//        auto goalX = dconstant::geometry::fieldLength / 2;
//        auto goalY = dconstant::geometry::goalWidth / 2;
//        int xx[4] = {1, 1, -1, -1};
//        int yy[4] = {1, -1, -1, 1};
//
//        for(auto& goal: z.goalPosts) {
//            cv::Point2d gCenter = getOnGlobalCoordinate(gParticle, cv::Point2d(goal.x, goal.y));
//            // Get dist to closest goal
//            double dist = 5.0;
//            for(int i = 0; i < 4; ++i) {
//                auto dx = goalX * xx[i] - gCenter.x;
//                auto dy = goalY * yy[i] - gCenter.y;
//                dist = std::min(sqrt(dx * dx + dy * dy) / 5.0, dist);
//            }
//            // divide by 5.0, like in occ dist map
//            dist /= 5.0;
//            double pz = normal_pdf(dist, 0.0, sigma_hit);
//            prob += pz * pz * pz * z.whitePoints.size() / 10;
//        }

        sample.weight = prob;
        total_weight += sample.weight;
    }

    if(total_weight > 0) {
        for(auto& sample : set.samples) {
            sample.weight /= total_weight;
        }

        double w_avg = total_weight / set.samples.size();
        if(w_slow == 0.0) {
            w_slow = w_avg;
        } else {
            w_slow += alpha_slow * (w_avg - w_slow);
        }

        if (w_fast == 0.0) {
            w_fast = w_avg;
        } else {
            w_fast += alpha_fast * (w_avg - w_fast);
        }
    } else {
        for(auto& sample : set.samples)
            sample.weight = 1.0 / set.samples.size();
    }
}

void AMCL::Resample(Measurement& z) {
    auto &set_a = m_sets[m_current_set];
    auto &set_b = m_sets[(m_current_set + 1) % 2];

    double c[set_a.samples.size() + 1];
    c[0] = 0.0;
    for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
        c[i + 1] = c[i] + set_a.samples[i].weight;
    }

    size_t cnt = 0;
    if(!z.centerPoints.empty()) {
        // calc pose with max likelihood observing the center
        // and put some samples there
        size_t reset_for_center_size = set_b.samples.size() / 3;
        auto& center = z.centerPoints[0];
        double x = center.x;
        double y = center.y;
        double r = m_pose.headingR();

        auto dx = x * cos(r) - y * sin(r);
        auto dy = x * sin(r) + y * cos(r);

        auto xx = 0.0 - dx;
        auto yy = 0.0 - dy;
        for(; cnt < reset_for_center_size; ++cnt) {
            set_b.samples[cnt].pose = Pose(xx + m_resampleGauss.sample(), yy, m_pose.heading());
            set_b.samples[cnt].weight = 1.0;
        }
    }

#if 0
    if(!z.goalPosts.empty()) {
        // calc pose with maxlikelihood observing the goal post
        // and push some samples there
        size_t reset_size = set_b.samples.size() / 10;
        auto goalX = dconstant::geometry::fieldLength / 2;
        auto goalY = dconstant::geometry::goalWidth / 2;

        if(z.goalPosts.size() == 1) {
//            auto& goal = z.goalPosts[0];
//            double x = goal.x;
//            double y = goal.y;
//            double r = m_pose.headingR();
//
//            auto dx = x * cos(r) - y * sin(r);
//            auto dy = x * sin(r) + y * cos(r);
//
//            double xx;
//            if(dx > 0) {
//                xx = goalX - dx;
//            } else {
//                xx = -goalX - dx;
//            }
//
//            for(int i = 0; i < reset_size; i+=2) {
//                set_b.samples[cnt].pose = Pose(xx + m_resampleGauss.sample(), goalY - dy + m_resampleGauss.sample(), m_pose.heading());
//                set_b.samples[cnt].weight = 1.0;
//                ++cnt;
//
//                set_b.samples[cnt].pose = Pose(xx + m_resampleGauss.sample(), -goalY - dy + m_resampleGauss.sample(), m_pose.heading());
//                set_b.samples[cnt].weight = 1.0;
//                ++cnt;
//            }
        } else {
            auto& goalLeft = z.goalPosts[0];
            auto& goalRight = z.goalPosts[1];

            {
                double x = goalLeft.x;
                double y = goalLeft.y;
                double r = m_pose.headingR();

                auto dx = x * cos(r) - y * sin(r);
                auto dy = x * sin(r) + y * cos(r);

                double xx;
                bool leftSide;
                if(dx > 0) {
                    xx = goalX - dx;
                    leftSide = false;
                } else {
                    xx = -goalX - dx;
                    leftSide = true;
                }

                for(size_t i = 0; i < reset_size; ++i) {
                    double newY;
                    if(leftSide)
                        newY = -goalY - dy;
                    else
                        newY = goalY - dy;
                    set_b.samples[cnt].pose = Pose(xx + m_resampleGauss.sample(), newY + m_resampleGauss.sample(), m_pose.heading());
                    set_b.samples[cnt].weight = 1.0;
                    ++cnt;
                }
            }
            {
                double x = goalRight.x;
                double y = goalRight.y;
                double r = m_pose.headingR();

                auto dx = x * cos(r) - y * sin(r);
                auto dy = x * sin(r) + y * cos(r);

                double xx;
                bool leftSide;
                if(dx > 0) {
                    xx = goalX - dx;
                    leftSide = false;
                } else {
                    xx = -goalX - dx;
                    leftSide = true;
                }
                for(size_t i = 0; i < reset_size; ++i) {
                    double newY;
                    if(leftSide)
                        newY = goalY - dy;
                    else
                        newY = -goalY - dy;

                    set_b.samples[cnt].pose = Pose(xx + m_resampleGauss.sample(), newY + m_resampleGauss.sample(), m_pose.heading());
                    set_b.samples[cnt].weight = 1.0;
                    ++cnt;
                }
            }
        }
    }
#endif
    double w_diff = 1.0 - w_fast / w_slow;
    w_diff = max(0.0, w_diff);

    for (; cnt < set_b.samples.size(); ++cnt) {
        auto &sample_b = set_b.samples[cnt];

        if(drand48() < w_diff) {
            sample_b.pose = RandomPose();
        } else {
            double r = drand48();
            for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
                if ((c[i] <= r) && (r < c[i + 1])) {
                    sample_b = set_a.samples[i];
                    sample_b.pose.setX(sample_b.pose.x() + m_resampleGauss.sample());
                    sample_b.pose.setY(sample_b.pose.y());
                    break;
                }
            }
        }
        sample_b.weight = 1.0;
    }

    m_current_set = (m_current_set + 1) % 2;

    CheckConverged();
}

void AMCL::Estimate() {
//    double x_sum = 0, y_sum = 0, z_sum = 0;
//    for(auto& sample : this->partices()) {
//        x_sum += sample.pose.x();
//        y_sum += sample.pose.y();
//        z_sum += sample.pose.heading();
//    }
//
//    auto num_particles = this->partices().size();
//    m_pose.setX(x_sum / num_particles);
//    m_pose.setY(y_sum / num_particles);
//    m_pose.setHeading(z_sum / num_particles);

    // Create the kd tree for adaptive sampling
    auto& set_b = m_sets[m_current_set];
    set_b.kdtree->clear();

    for(auto& sample : set_b.samples) {
        set_b.kdtree->InsertPose(sample.pose, sample.weight);
    }

    // Re-compute cluster statistics
    ClusterStats(set_b);


    auto& set = m_sets[m_current_set];
    double max_weight = 0.0;
    for(auto& cluster : set.clusters) {
        if(cluster.weight > max_weight) {
            max_weight = cluster.weight;
            m_pose = cluster.mean;
        }
    }
}

bool AMCL::CheckConverged() {
    auto& set = m_sets[m_current_set];

    double mean_x = 0, mean_y = 0;
    for(auto& sample : set.samples) {
        mean_x += sample.pose.x();
        mean_y += sample.pose.y();
    }
    mean_x /= set.samples.size();
    mean_y /= set.samples.size();

    for(auto& sample : set.samples) {
        if(fabs(sample.pose.x() - mean_x) > this->dist_threshold ||
            fabs(sample.pose.y() - mean_y) > this->dist_threshold) {
            set.converged = false;
            m_converged = false;
            return false;
        }
    }
    m_converged = true;
    set.converged = true;
    return true;
}

void AMCL::ClusterStats(SampleSet &set) {
    // Workspace
    double m[4], c[2][2];

    set.kdtree->Cluster();

    // Initialize cluster stats
    set.cluster_count = 0;

    for(int i = 0; i < set.cluster_max_count; ++i) {
        Cluster& cluster = set.clusters[i];
        cluster.count = 0;
        cluster.weight = 0;
        cluster.mean = Pose();
        cluster.cov = Matrix();

        for(int j = 0; j < 4; ++j)
            cluster.m[j] = 0.0;

        for(int j = 0; j < 2; ++j)
            for(int k = 0; k < 2; ++k)
                cluster.c[j][k] = 0.0;
    }

    // Initialize overall filter stats
    size_t count = 0;
    double weight = 0.0;
    set.mean = Pose();
    set.cov = Matrix();

    for(int j = 0; j < 4; ++j)
        m[j] = 0.0;

    for(int j = 0; j < 2; ++j)
        for(int k = 0; k < 2; ++k)
            c[j][k] = 0.0;

    // Compute cluster stats
    for(auto& sample : set.samples) {
        // Get the cluster label for this sample
        int cidx = set.kdtree->GetCluster(sample.pose);
        //assert(cidx >= 0);

        if(cidx >= set.cluster_max_count)
            continue;
        if(cidx + 1 > set.cluster_count)
            set.cluster_count = cidx + 1;

        auto& cluster = set.clusters[cidx];

        cluster.count += 1;
        cluster.weight += sample.weight;

        count += 1;
        weight += sample.weight;


        // Compute mean
        cluster.m[0] += sample.weight * sample.pose.x();
        cluster.m[1] += sample.weight * sample.pose.y();
        cluster.m[2] += sample.weight * std::cos(sample.pose.headingR());
        cluster.m[3] += sample.weight * std::sin(sample.pose.headingR());

        m[0] += sample.weight * sample.pose.x();
        m[1] += sample.weight * sample.pose.y();
        m[2] += sample.weight * std::cos(sample.pose.headingR());
        m[3] += sample.weight * std::sin(sample.pose.headingR());

        // Compute covariance in linear components
        for(int j = 0; j < 2; ++j)
            for(int k = 0; k < 2; ++k) {
                cluster.c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
                c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
            }
    }

    // Normalize
    for(int i = 0; i < set.cluster_count; ++i) {
        auto& cluster = set.clusters[i];
        cluster.mean.setX(cluster.m[0] / cluster.weight);
        cluster.mean.setY(cluster.m[1] / cluster.weight);
        cluster.mean.setHeadingR(atan2(cluster.m[3], cluster.m[2]));

        cluster.cov = Matrix();

        // Covariance in linear components
        for(int j = 0; j < 2; ++j)
            for(int k = 0; k < 2; ++k) {
                cluster.cov.m[j][k] = cluster.c[j][k] / cluster.weight -
                    cluster.mean[j] * cluster.mean[k];
            }
        // Covariance in angular components;
        cluster.cov.m[2][2] = -2 * log(sqrt(cluster.m[2] * cluster.m[2] +
                                            cluster.m[3] * cluster.m[3]));
        // Cluster mean weight
        cluster.meanWeight = cluster.weight / cluster.count;
    }

    // Compute overall filter stats
    set.mean[0] = m[0] / weight;
    set.mean[1] = m[1] / weight;
    set.mean.setHeadingR(atan2(m[3], m[2]));

    // Covariance in linear components
    for(int j = 0; j < 2; ++j)
        for(int k = 0; k < 2; ++k) {
            set.cov.m[j][k] = c[j][k] / weight - set.mean[j] * set.mean[k];
        }


    set.cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}

std::vector<Particle> &AMCL::partices() {
    return m_sets[m_current_set].samples;
}

Pose AMCL::RandomPose() {
    double w = dconstant::geometry::fieldLength / 2;
    double h = dconstant::geometry::fieldWidth / 2;

    Pose p;
    p.setX(m_uniform.sample() * w);
    p.setY(m_uniform.sample() * h);
    p.setHeading(m_uniform.sample() * 180);
    return p;
}

Map &AMCL::map() {
    return m_map;
}

Pose AMCL::GetEstimatedPose() {
    return m_pose;
}

bool AMCL::IsConverged() {
    return m_converged;
}

void AMCL::ResetParticlesLeftTouch() {
    // from (-450, 300) to (0, 300)
    // from (-450, -300) to (0, -300)
    auto xUniform = Uniform(-450, 0);
    auto& cur_set = m_sets[m_current_set];
    size_t i;
    for(i = 0; i < cur_set.samples.size() / 2; ++i) {
        cur_set.samples[i].pose = Pose(xUniform.sample(), 300, m_yaw);
    }
    for(; i < cur_set.samples.size(); ++i) {
        cur_set.samples[i].pose = Pose(xUniform.sample(), -300, m_yaw);
    }
}

void AMCL::ResetParticlesRightTouch() {
    // from (450, 300) to (0, 300)
    // from (450, -300) to (0, -300)
    auto xUniform = Uniform(450, 0);
    auto& cur_set = m_sets[m_current_set];
    size_t i;
    for(i = 0; i < cur_set.samples.size() / 2; ++i) {
        cur_set.samples[i].pose = Pose(xUniform.sample(), 300, m_yaw);
    }
    for(; i < cur_set.samples.size(); ++i) {
        cur_set.samples[i].pose = Pose(xUniform.sample(), -300, m_yaw);
    }
}

void AMCL::ResetParticlesPoint(geometry_msgs::Vector3 point) {
    Gaussian fuck(0, 3);
    auto& cur_set = m_sets[m_current_set];
    for(auto& sample : cur_set.samples) {
        sample.pose = Pose(point.x + fuck.sample(), point.y + fuck.sample(), m_yaw);
    }
}

void AMCL::falldownGauss() {
    Gaussian g(0, 10);
    for(auto& sample : m_sets[m_current_set].samples) {
        auto x = sample.pose.x();
        auto y = sample.pose.y();

        sample.pose.setX(x + g.sample());
        sample.pose.setY(y + g.sample());
    }
}

} // namespace dvision
