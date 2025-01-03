#include "statistics.hpp"

double bds::GetMeanDistance(std::vector<boid> boid_vector) {
if (boid_vector.size() <= 1) {
        return 0.;
    }

    double dist_sum = 0.;
    int n_events = 0;

    for (size_t i = 0; i < boid_vector.size(); ++i) {
        for (size_t j = i + 1; j < boid_vector.size(); ++j) {
            dist_sum += sqrt(squaresum(boid_vector[i].get_pos_value() - boid_vector[j].get_pos_value()));
            ++n_events;
        }
    }

    return dist_sum / n_events;
}

double bds::GetMeanVelocity(std::vector<boid> boid_vector) {
    if (boid_vector.empty()) {
        return 0.;
    }

    double vel_sum = 0.;
    for (const auto& boid : boid_vector) {
        vel_sum += sqrt(squaresum(boid.get_vel_value()));
    }

    return vel_sum / boid_vector.size();
}

double bds::GetStdDevDistance(std::vector<boid> boid_vector) {
  if (boid_vector.size() <= 1) {
        return 0.;
    }

    double meandist = GetMeanDistance(boid_vector);
    double sum = 0.;
    int n_events = 0;

    for (size_t i = 0; i < boid_vector.size(); ++i) {
        for (size_t j = i + 1; j < boid_vector.size(); ++j) {
            double dist = sqrt(squaresum(boid_vector[i].get_pos_value() - boid_vector[j].get_pos_value()));
            sum += (dist - meandist) * (dist - meandist);
            ++n_events;
        }
    }

    return sqrt(sum / n_events);
} 

double bds::GetStdDevVelocity(std::vector<boid> boid_vector) {
  if (boid_vector.size() <= 1) {
        return 0.;
    }

    double meanvel = GetMeanVelocity(boid_vector);
    double sum = std::accumulate(
        boid_vector.begin(), boid_vector.end(), 0.0,
        [meanvel](double sm, const boid& i_boid) {
            double diff = sqrt(squaresum(i_boid.get_vel_value())) - meanvel;
            return sm + diff * diff;
        });

    return sqrt(sum / (boid_vector.size() - 1));
}  
