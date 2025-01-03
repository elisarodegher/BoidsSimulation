#ifndef STATISTICS_HPP
#define STATISTICS_HPP
#include "graphics.hpp"

namespace bds {
    double GetMeanDistance(std::vector<boid> boid_vector);
    double GetMeanVelocity(std::vector<boid> boid_vector);
    double GetStdDevDistance(std::vector<boid> boid_vector);
    double GetStdDevVelocity(std::vector<boid> boid_vector);
}
#endif