#include "boids.hpp"
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>


int main()
{
    lu_int n_boids;

    double sep_fact;
    double align_fact;
    double coes_fact;

    double sep_dist;
    double dist_vic;

    double time_interval;
    double time_check;

    n_boids = 100;
    dist_vic = 6.; 
    sep_dist = 0.5;
    sep_fact = 0.3; 
    align_fact = 0.09;
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
     std::cout << "This program prints statistic values of the group every x seconds. Please insert the number of seconds between each print: \n";
    std::cin >> time_check;
    */    
    // esisterà un modo più elegante per fare le domande all'utente? compilazione di una tabella nella parte grafica?

    std::default_random_engine eng(static_cast<lu_int>(std::time(nullptr)));
    std::uniform_real_distribution<double> dist(0., 2.);

    std::vector<bds::boid> boid_vector;
    couple p;
    couple s;

    for (lu_int i = 0; i != n_boids; ++i)
    {
        p = {dist(eng), dist(eng)};
        s = {dist(eng), dist(eng)};

        boid_vector.push_back({p, s});
    }
    
    // controllare che non ci siano boids uguali

    double fieldwidth{30.};
    double fieldheight{20.};
    double Deltat{0.025};
    
    sf::RenderWindow sky(sf::VideoMode(900, 600), "boidsgraphic", sf::Style::Default);
    sky.setFramerateLimit(40); //questo frame dovrebbe sincronizzarsi col tempo di calcolo
    sf::Clock clock;
    sf::Clock stat_clock;
    //conversion width-height rate to window: 60

        while(sky.isOpen()) {

        sf::Event event;
        while (sky.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                sky.close();
            }
        }

        if (clock.getElapsedTime().asSeconds() >= time_interval) {
            sky.close();
        }

        if (stat_clock.getElapsedTime().asSeconds() >= time_check) {
            std::cout << "Time passed: " << clock.getElapsedTime().asSeconds() << " s\n";
            std::cout << "Mean distance: " << bds::GetMeanDistance(boid_vector) << " +/- " << bds::GetStdDevDistance(boid_vector) << "\n";
            std::cout << "Mean velocity: " << bds::GetMeanVelocity(boid_vector) << " +/- " << bds::GetStdDevVelocity(boid_vector) << "\n";
            stat_clock.restart();
        }

        sky.clear(sf::Color(135, 206, 235));

        for (lu_int i = 0; i < boid_vector.size(); ++i)
        {   
            sf::ConvexShape boid;
            boid.setPointCount(3);
            boid.setPoint(0, sf::Vector2f(0.f, 0.f));
            boid.setPoint(1, sf::Vector2f(9.f, 3.f));
            boid.setPoint(2, sf::Vector2f(0.f, 6.f));
            boid.setFillColor(sf::Color::Black);
            
            boid.setOrigin(3.f, 3.f);
            boid.setPosition(450., 300.);

            v_mod(i, sep_fact, sep_dist, align_fact, dist_vic, coes_fact, boid_vector, fieldwidth, fieldheight);
            double angle_rad = boid_vector[i].get_angle();
            double angle_deg = angle_rad * (180 / 3.1415);
            boid.rotate(static_cast<float>(angle_deg));

            p_mod(i, boid_vector, Deltat);
            Pacman(boid_vector, i, fieldwidth, fieldheight);

            couple gr_pos = boid_vector[i].pos();
            gr_pos = 30 * gr_pos;
            boid.move(static_cast<float>(gr_pos[0]), static_cast<float>(gr_pos[1]));

            sky.draw(boid);
        }
        
        sky.display();

        }

    if (bds::GetMeanVelocity(boid_vector) > std::numeric_limits<double>::max()) {
        std::cout << "velocità troppo alta" << '\n';
    }
    else {

    std::cout << "Final mean distance: " << bds::GetMeanDistance(boid_vector) << " +/- " << bds::GetStdDevDistance(boid_vector) << "\n";
    std::cout << "Final mean velocity: " << bds::GetMeanVelocity(boid_vector) << " +/- " << bds::GetStdDevVelocity(boid_vector) << "\n";
    }
}


