#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>

#include "statistics.hpp"

int main() {
  //INITIAL FACTORS
  lu_int n_boids;

  double sep_fact;
  double align_fact;
  double coes_fact;

  double sep_dist;
  double dist_vic;

  double rndm_mod;
  double wind_intensity;

  double time_interval;
  double time_check;
  double wind_time;

  char yn_answer;

  n_boids = 150;
  dist_vic = 7.;
  sep_dist = 0.1;
  sep_fact = 1;
  align_fact = 0.01; //range 0.01 - 0.2
  coes_fact = 0.1; //range 0.005 - 0.1
  time_interval = 60;
  time_check = 5.; 
  rndm_mod = 0;
  wind_intensity = 0;
  wind_time = 3;
  
  //USER INTERFACE

  /*std::cout << "BOIDS SIMULATION\n by Elisa Rodegher, Emma Rubbi, Gaetano valentino\n" << "\n This program performs a simulation of a flock of boids. Before starting, we need a series of input parameters. \n";
  std::cout << "Please type the following parameters: \n";
  std::cout << "a) Number of boids: \n";
  std::cin >> n_boids;
  std::cout << "b) Maximum distance for boids to be considered near \n";
  std::cin >> dist_vic;
  std::cout << "c) Maximum distance for boids to separate \n";
  std::cin >> sep_dist;
  std::cout << "Each boid moves following three rules: separation, alignment, coesion. Please type the coefficient that determine the impact of each rule (see relation for more informations)\n";
  std::cout << "d) Separation rule coefficient: \n";
  std::cin >> sep_fact;
  std::cout << "e) Alignment rule coefficient: \n";
  std::cin >> align_fact;
  std::cout << "f) Coesion rule coefficient: \n";
  std::cin >> coes_fact;
  std::cout << "The program uses a low-impact random booster. Do you want to manually initialize it?(y/n)\n";
  std::cin >> yn_answer;
  if (yn_answer == 'y' || yn_answer == 'Y') {
    std::cout << "Please insert the intensity of the booster (0 means no booster):\n";
    std::cin >> rndm_mod;
  } else {
    rndm_mod = 0.3;
  }
  std::cout << "Boids are subjected to wind force. Do you want to manually initialize the wind?\n";
  std::cin >> yn_answer;
  if (yn_answer == 'y' || yn_answer == 'Y') {
    std::cout << "Please type the wind's intensity. It will not change for the entire simulation\n";
    std::cin >> wind_intensity;
    std::cout << "The wind will randomly change direction every x seconds. Please insert the duration of each direction:\n";
    std::cin >> wind_time;
  } else {
    wind_intensity = 0.05;
    wind_time = 3.5;
  }
  std::cout << "Now type the simulation's duration (in seconds). You can end the simulation earlier by closing the graphic simulation window.\n";
  std::cin >> time_interval;
  std::cout << "Lastly, this program prints statistic values of the group every x seconds. Please insert the number of seconds between each print: \n"; 
  std::cin >> time_check;
  */
  //BOIDS INIZIALIZATION

  std::default_random_engine eng(static_cast<lu_int>(std::time(nullptr)));
  std::uniform_real_distribution<double> dist(-30., 30.);

  std::vector<bds::boid> boid_vector;
  couple p;
  couple s;

  for (lu_int i = 0; i != n_boids; ++i) {
    p = {2 * dist(eng), 3 * dist(eng)};
    s = {dist(eng), dist(eng)};

    boid_vector.push_back({p, s});
  }

  bds::wind b_wind{wind_intensity, 0.};

  double fieldwidth{90.};
  double fieldheight{60.};
  double Deltat{0.025};

  //GRAPHIC INIZIALIZATION

  sf::RenderWindow sky(sf::VideoMode(900, 600), "boidsgraphic",
                       sf::Style::Default);
  sky.setFramerateLimit(static_cast<unsigned int>(
      1 / Deltat));
  sf::Clock clock;
  sf::Clock stat_clock;
  sf::Clock wind_clock;
  // conversion width-height rate to window: 10

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

    if (wind_clock.getElapsedTime().asSeconds() >= wind_time) {
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
      gr_pos = 10 * gr_pos;
      GBoid.move(gr_pos[0], gr_pos[1]);

      GBoid.draw(sky);
    }

    sf::RectangleShape wind_panel(sf::Vector2f(70.f, 70.f));
    sf::Color lightGreen(167, 250, 154);
    wind_panel.setFillColor(lightGreen);
    sky.draw(wind_panel);

    bds::GraphicWind GWind{b_wind};
    GWind.setPosition(35., 35.);
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
