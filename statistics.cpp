#include "statistics.hpp"

double bds::GetMeanDistance(std::vector<boid> boid_vector) {
  double dist_sum{0.};
  int n_events{0};

  dist_sum = std::accumulate(
      boid_vector.begin(), boid_vector.end(), double{0.},
      [&](double& d_s, boid& i_boid) {
        double local_sum = std::accumulate(
            boid_vector.begin(), boid_vector.end(), double{0.},
            [&](double& ds, boid& j_boid) {
              ds += sqrt(squaresum(i_boid.get_pos_value() - j_boid.get_pos_value()));
              ++n_events;
              return ds;
            });
        return d_s + local_sum;
      });

  dist_sum = dist_sum / 2;  
  double meandist = dist_sum / n_events;
  return meandist;
}

double bds::GetMeanVelocity(std::vector<boid> boid_vector) {
  double vel_sum{0.};

  vel_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.},
                            [&](double& vs, boid& i_boid) {
                              vs += sqrt(squaresum(i_boid.get_vel_value()));
                              return vs;
                            });

  double meanvel = vel_sum / static_cast<double>(boid_vector.size());
  return meanvel;
}

double bds::GetStdDevDistance(std::vector<boid> boid_vector) {
  double meandist = GetMeanDistance(boid_vector);
  double sum = std::accumulate(
      boid_vector.begin(), boid_vector.end(), double{0.},
      [&](double& pass_sum, boid& i_boid) {
        double local_sum = std::accumulate(
            boid_vector.begin(), boid_vector.end(), double{0.},
            [&](double& dm, boid& j_boid) {
              dm = (sqrt(squaresum(i_boid.get_pos_value() - j_boid.get_pos_value())) - meandist);
              dm *= dm;
              return dm;
            });
        pass_sum += local_sum;
        return pass_sum;
      });
  
  double stddev =
      sqrt(sum / (static_cast<double>(boid_vector.size()) *
                  (static_cast<double>(boid_vector.size()) - 1) * 2));
  return stddev;
} 

double bds::GetStdDevVelocity(std::vector<boid> boid_vector) {
  double meanvel = GetMeanVelocity(boid_vector);
  double sum;

  sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.},
                        [&](double& sm, boid& i_boid) {
                          sm = (sqrt(squaresum(i_boid.get_vel_value())) - meanvel);
                          sm *= sm;
                          return sm;
                        });

  double stddev =
      sqrt(sum / (static_cast<double>(boid_vector.size()) *
                  (static_cast<double>(boid_vector.size()) - 1) * 2));
  return stddev;
}  
