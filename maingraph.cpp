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

    n_boids = 200;
    dist_vic = 6.; 
    sep_dist = 0.5;
    sep_fact = 0.7; 
    align_fact = 0.1;
    coes_fact = 0.09;
    time_interval = 60;


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

            // std::cout << "Before teleport: Position = (" << boid_vector[i].pos()[0] << ", " << boid_vector[i].pos()[1] << "), Velocity = (" << boid_vector[i].vel()[0] << ", " << boid_vector[i].vel()[1] << ")\n";

            v_mod(i, sep_fact, sep_dist, align_fact, dist_vic, coes_fact, boid_vector, fieldwidth, fieldheight);
            double angle_rad = boid_vector[i].get_angle();
            double angle_deg = angle_rad * (180 / 3.1415);
            boid.rotate(angle_deg);

            p_mod(i, boid_vector, Deltat);
            Pacman(boid_vector, i, fieldwidth, fieldheight);
            // std::cout << "After teleport: Position = (" << boid_vector[i].pos()[0] << ", " << boid_vector[i].pos()[1] << "), Velocity = (" << boid_vector[i].vel()[0] << ", " << boid_vector[i].vel()[1] << ")\n";
            couple gr_pos = boid_vector[i].pos();
            gr_pos = 30 * gr_pos;
            boid.move(gr_pos[0], gr_pos[1]);

            sky.draw(boid);
        }
        
        sky.display();

        }

    if (bds::GetMeanVelocity(boid_vector) > std::numeric_limits<double>::max()) {
        std::cout << "velocità troppo alta" << '\n';
    }
    else {

    std::cout << "Distanza media tra i boids" << bds::GetMeanDistance(boid_vector, sep_dist, fieldwidth, fieldheight) << "+/-" << bds::GetStdDevDistance(boid_vector, sep_dist, fieldwidth, fieldheight) << "\n";
    std::cout << "Velocità media dei boids" << bds::GetMeanVelocity(boid_vector) << "+/-" << bds::GetStdDevVelocity(boid_vector) << "\n";
    }
}


