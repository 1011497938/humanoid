#pragma once
#include "dvision/amcl/pose.hpp"

namespace dvision {
class Particle {
public:
    Particle();
    Pose pose;
    double weight;
};
}