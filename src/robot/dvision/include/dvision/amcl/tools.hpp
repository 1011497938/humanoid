#pragma once
#include <random>

namespace dvision {

template <typename T>
class Distribution {
public:
    Distribution(float a, float b) {
        m_dist = T(a, b);

        std::random_device rd;
        m_generator = std::default_random_engine(rd());
    }
    float sample() {
        return m_dist(m_generator);
    }

private:
    std::default_random_engine m_generator;
    T m_dist;
};

using Gaussian = Distribution<std::normal_distribution<float>>;
using Uniform = Distribution<std::uniform_real_distribution<float>>;


// Copied from https://stackoverflow.com/a/10848293
template <typename T>
T normal_pdf(T x, T m, T s)
{
    static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
}

} // namespace dvision