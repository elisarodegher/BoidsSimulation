#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "statistics.hpp"

int main() {
  lu_int n_boids;

  double sep_fact;
  double align_fact;
  double coes_fact;

  double sep_dist;
  double dist_vic;

  double rndm_mod{0.3};
  double wind_intensity{1};

  double time_interval;
  double time_check;

  n_boids = 200;
  dist_vic = 1.;
  sep_dist = 0.1;
  sep_fact = 0.3;
  align_fact = 0.04;
  coes_fact = 0.009;
  time_interval = 60;
  time_check = 5.;

  /*std::cout << "Number of boids: \n";
  std::cin >> n_boids;
  std::cout << "Minimum distance for boids to be considered near \n";
  std::cin >> dist_vic;
  std::cout << "Minimum distance for boids to separate \n";
  std::cin >> sep_dist;
   std::cout << "Separation factor: \n";
  std::cin >> sep_fact;
   std::cout << "Alignment factor: \n";
  std::cin >> align_fact;
   std::cout << "Coesion factor: \n";
  std::cin >> coes_fact;
   std::cout << "Simulation's duration:\n";
  std::cin >> time_interval;
   std::cout << "This program prints statistic values of the group every x
  seconds. Please insert the number of seconds between each print: \n"; std::cin
  >> time_check;
  */

  std::default_random_engine eng(static_cast<lu_int>(std::time(nullptr)));
  std::uniform_real_distribution<double> dist(-10., 10.);

  std::vector<bds::boid> boid_vector;
  couple p;
  couple s;

  for (lu_int i = 0; i != n_boids; ++i) {
    p = {dist(eng), dist(eng)};
    s = {dist(eng), dist(eng)};

    boid_vector.push_back({p, s});
  }

  bds::wind b_wind{wind_intensity, 0.};

  double fieldwidth{30.};
  double fieldheight{20.};
  double Deltat{0.025};


  sf::RenderWindow sky(sf::VideoMode(900, 600), "boidsgraphic",
                       sf::Style::Default);
  sky.setFramerateLimit( static_cast<unsigned int>(
      1 / Deltat));
  sf::Clock clock;
  sf::Clock stat_clock;
  sf::Clock wind_clock;
  // conversion width-height rate to window: 60

  while (sky.isOpen()) {
    //EVENTS AND TIME MANAGING
    sf::Event event;
    while (sky.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        sky.close();
      }
    }

    if (clock.getElapsedTime().asSeconds() >= time_interval) {
      sky.close();
    }

    if (stat_clock.getElapsedTime().asSeconds() >= time_check) {
      std::cout << "Time passed: " << clock.getElapsedTime().asSeconds()
                << " s\n";
      std::cout << "Mean distance: " << bds::GetMeanDistance(boid_vector)
                << " +/- " << bds::GetStdDevDistance(boid_vector) << "\n";
      std::cout << "Mean velocity: " << bds::GetMeanVelocity(boid_vector)
                << " +/- " << bds::GetStdDevVelocity(boid_vector) << "\n";
      stat_clock.restart();
    }

    
    
    if (wind_clock.getElapsedTime().asSeconds() >= 9) {
    std::random_device wind_mod;
    std::default_random_engine eng2(wind_mod());
    std::uniform_real_distribution<double> wind_rotations(-180., 180.);
    double wind_angle = wind_rotations(eng);
    b_wind.rotate(to_radians(wind_angle));
    wind_clock.restart();
    }
    // DRAWING
    sky.clear(sf::Color(135, 206, 235));

    for (lu_int i = 0; i < boid_vector.size(); ++i) {
      bds::GraphicBoids GBoid{};
      GBoid.setPosition(450., 300.);

      v_mod(i, sep_fact, sep_dist, align_fact, dist_vic, coes_fact, boid_vector,
            fieldwidth, fieldheight, b_wind, rndm_mod);
      GBoid.rotate(to_degrees(boid_vector[i].get_angle()));

      p_mod(i, boid_vector, Deltat);
      Pacman(boid_vector, i, fieldwidth, fieldheight);

      couple gr_pos = boid_vector[i].get_pos_value();
      gr_pos = 30 * gr_pos;
      GBoid.move(gr_pos[0], gr_pos[1]);

      GBoid.draw(sky);
    }

    bds::GraphicWind GWind{b_wind};
    GWind.draw(sky);

    sky.display();
  }

  //FINAL STATISTICS

  if (bds::GetMeanVelocity(boid_vector) > std::numeric_limits<double>::max()) {
    std::cout << "velocità troppo alta" << '\n';
  } else {
    std::cout << "Final mean distance: " << bds::GetMeanDistance(boid_vector)
              << " +/- " << bds::GetStdDevDistance(boid_vector) << "\n";
    std::cout << "Final mean velocity: " << bds::GetMeanVelocity(boid_vector)
              << " +/- " << bds::GetStdDevVelocity(boid_vector) << "\n";
  }
}
