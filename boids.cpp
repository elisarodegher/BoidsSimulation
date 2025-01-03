#include "boids.hpp"

bds::boid::boid(couple p, couple s) : pos_{p}, vel_{s} {}

void bds::boid::vel_mod(couple s) {
  vel_ += s;  
}  

void bds::boid::pos_mod(double deltat) {
  pos_ += (deltat * vel_);
}  
void bds::boid::pos_mod(couple p) { pos_ += p; }

couple bds::boid::get_pos_value() const { return pos_; }
couple bds::boid::get_vel_value() const { return vel_; }
couple& bds::boid::get_pos_ref() {
  return pos_;
} 
double bds::boid::get_angle() const {
  double angle = atan2(vel_[1], vel_[0]);
  return angle;
}


bds::wind::wind(double intensity, double angle) : intensity_{intensity}, angle_{angle} {}

void bds::wind::rotate(double rot_ang) {
    angle_ += rot_ang;
}

couple bds::wind::get_coordinates() {
    double x_comp = intensity_ * cos(angle_);
    double y_comp = intensity_ * sin(angle_);
    couple comp{x_comp, y_comp};
    return comp;
}

double bds::wind::get_angle_rad() { return angle_; }

/* FUNZIONI LIBERE */
void bds::periodize(couple& pos, double perx, double pery) {
  if ((pos[0]) > (fabs(perx) / 2)) {
    pos[0] -= perx;
  }
  if ((pos[0]) < (-fabs(perx) / 2)) {
    pos[0] += perx;
  }
  if ((pos[1]) > (fabs(pery) / 2)) {
    pos[1] -= pery;
  }
  if ((pos[1]) < (-fabs(pery) / 2)) {
    pos[1] += pery;
  }
}

bool bds::BoidsAreNear(bds::boid i, bds::boid j, double dist,
                       double field_width, double field_height)

{
  if (&i == &j) {
    return 0;
  }
  else {

  couple diff = (i.get_pos_value() - j.get_pos_value());
  periodize(diff, field_width, field_height);

  double diffdist = squaresum(diff);
  return diffdist < (dist * dist);
  }
}

couple bds::v_separation(lu_int i, double sep_dist, double sep_fact,
                         const std::vector<boid>& boid_vector, double field_width,
                         double field_height) {

  couple v_sep;
  v_sep = std::accumulate(
      boid_vector.begin(), boid_vector.end(), couple{0., 0.},
      [&](couple& rel, const boid& j_boid) {
        if (BoidsAreNear(boid_vector[i], j_boid, sep_dist, field_width, field_height)) {
          couple diff = (j_boid.get_pos_value() - boid_vector[i].get_pos_value());
          periodize(diff, field_width, field_height);
          rel += diff;
        }
        return rel;
      });

  v_sep = (-1 * sep_fact) * v_sep;
  return v_sep;
}

couple bds::v_alignment(lu_int i, double alig_fact,
                        const std::vector<boid>& boid_vector) {
  couple v_alig;
  assert(i < boid_vector.size());

  v_alig = std::accumulate(
      boid_vector.begin(), boid_vector.end(), couple{0., 0.},
      [&](couple& rvel, const boid& j_boid) {
        if (&boid_vector[i] !=
            &j_boid) { 
          rvel += j_boid.get_vel_value();
        }
        return rvel;
      });

  if (boid_vector.size() == 1) {
    v_alig = {0., 0.};
  } else {
    v_alig *= (1. / static_cast<double>(boid_vector.size() - 1));
    v_alig -= boid_vector[i].get_vel_value();
    v_alig *= alig_fact;
  }
  return v_alig;
}

couple bds::v_coesion(lu_int i, double dist_vic, double coes_fact,
                      const std::vector<boid>& boid_vector, double field_width,
                      double field_height) {
  couple c_mass;
  assert(i < boid_vector.size());

  int near_boids{1};

  c_mass = std::accumulate(
      boid_vector.begin(), boid_vector.end(), couple{0., 0.},
      [&](couple& cm, const boid& j_boid) {
        if (&boid_vector[i] != &j_boid && BoidsAreNear(boid_vector[i], j_boid, dist_vic, field_width, field_height)) {
            couple diff = j_boid.get_pos_value() - boid_vector[i].get_pos_value();
            periodize(diff, field_width, field_height);
            cm += (boid_vector[i].get_pos_value() + diff);
            ++near_boids;
        }
        return cm;
      });

  couple v_coes{0., 0.};
  if (near_boids == 1) {
    return v_coes;
  } else {
    c_mass *= (1. / static_cast<double>(near_boids - 1));
    c_mass -= boid_vector[i].get_pos_value();
    v_coes = coes_fact * c_mass;
    return v_coes;
  }
}

couple bds::v_random(double rndm_mod) {
  std::random_device rndm;
  std::default_random_engine eng(rndm());
  std::uniform_real_distribution<double> angle(0., 6.2830);
  std::uniform_real_distribution<double> vel(0., 1.);

  couple v_rndm{0., 0.};
  v_rndm[0] = rndm_mod * cos(angle(eng));
  v_rndm[1] =
      rndm_mod *
      sin(angle(
          eng)); 
  return v_rndm;
}

couple bds::v_wind(wind i_wind) {
    couple v_wind = 0.1 * i_wind.get_coordinates();
    return v_wind;
}

void bds::v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact,
                double dist_vic, double coes_fact,
                std::vector<boid>& boid_vector, double field_width,
                double field_height, wind gen_wind, double rndm_mod) {
  couple v_mod =
      v_separation(i, sep_dist, sep_fact, boid_vector, field_width,
                   field_height) +
      v_alignment(i, alig_fact, boid_vector) +
      v_coesion(i, dist_vic, coes_fact, boid_vector, field_width, field_height);
  v_mod += v_random(rndm_mod);
  v_mod += v_wind(gen_wind);
  boid_vector[i].vel_mod(v_mod);
}
void bds::p_mod(lu_int i, std::vector<boid>& boid_vector, double deltat) {
  boid_vector[i].pos_mod(deltat);
}

void bds::Pacman(std::vector<bds::boid>& boid_vector, lu_int i,
                 double field_width, double field_height) {
  periodize(boid_vector[i].get_pos_ref(), field_width, field_height);
}

bds::GraphicBoids::GraphicBoids() {
  sup.setPointCount(3);
  sup.setPoint(0, sf::Vector2f(0.f, 0.f));
  sup.setPoint(1, sf::Vector2f(9.f, 3.f));
  sup.setPoint(2, sf::Vector2f(2.f, 3.f));

  inf.setPointCount(3);
  inf.setPoint(0, sf::Vector2f(0.f, 6.f));
  inf.setPoint(1, sf::Vector2f(2.f, 3.f));
  inf.setPoint(2, sf::Vector2f(9.f, 3.f));

  sup.setFillColor(sf::Color::Red);
  inf.setFillColor(sf::Color::Red);

  sup.setOrigin(sf::Vector2f(3., 3.));
  inf.setOrigin(sf::Vector2f(3., 3.));
}

void bds::GraphicBoids::move(double x, double y) {
  inf.move(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
  sup.move(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
}

void bds::GraphicBoids::rotate(double ang) {
  inf.rotate(static_cast<float>(ang));
  sup.rotate(static_cast<float>(ang));
}

void bds::GraphicBoids::setPosition(double x, double y) {
  inf.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
  sup.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
}

void bds::GraphicBoids::draw(sf::RenderWindow& window) {
  window.draw(sup);
  window.draw(inf);
}

bds::GraphicWind::GraphicWind(wind w) : wind_line{sf::RectangleShape(sf::Vector2f(60., 3.))} {
    wind_arrow.setPointCount(5);
    wind_arrow.setPoint(0, sf::Vector2f(60.f, -1.f));
    wind_arrow.setPoint(1, sf::Vector2f(66.f, 1.5f));
    wind_arrow.setPoint(2, sf::Vector2f(60.f, 4.f));
    wind_arrow.setPoint(3, sf::Vector2f(57.f, 3.f));
    wind_arrow.setPoint(4, sf::Vector2f(57.f, 0.f));

    wind_line.setOrigin(sf::Vector2f(33.f, 1.5f));
    wind_arrow.setOrigin(sf::Vector2f(33.f, 1.5f));

    wind_line.setFillColor(sf::Color::Blue);
    wind_arrow.setFillColor(sf::Color::Blue);

    wind_line.setRotation(to_degrees(w.get_angle_rad()));
    wind_arrow.setRotation(to_degrees(w.get_angle_rad()));
}

void bds::GraphicWind::draw(sf::RenderWindow& window) {
    window.draw(wind_line);
    window.draw(wind_arrow);
}