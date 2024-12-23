#include "boids.hpp"
#include <random>
#include <ctime>

int main()
{
    lu_int n_boids;

    double sep_fact;
    double align_fact;
    double coes_fact;

    double sep_dist;
    double dist_vic;

double Intervalloditempo;

    std::cout << "Number of boids: \n";
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
    std::cin >> Intervalloditempo;
        
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
    double fieldwidth{15.};
    double fieldheight{10.};

    double Deltat{0.25};
    for (double t = 0; t < Intervalloditempo; t += Deltat)
    {
        for (lu_int i = 0; i < boid_vector.size(); ++i)
        {
            v_mod(i, sep_fact, sep_dist, align_fact, dist_vic, coes_fact, boid_vector, fieldwidth, fieldheight);
            p_mod(i, boid_vector, Deltat);
            
            Pacman(boid_vector, i, fieldwidth, fieldheight);
        }
        
    }

    std::cout << "Mean Distance: " << bds::GetMeanDistance(boid_vector, sep_dist, fieldwidth, fieldheight) << "+/-" << bds::GetStdDevDistance(boid_vector, sep_dist, fieldwidth, fieldheight) << "\n";
    std::cout << "Mean Velocity: " << bds::GetMeanVelocity(boid_vector) << "+/-" << bds::GetStdDevVelocity(boid_vector) << "\n";
}
