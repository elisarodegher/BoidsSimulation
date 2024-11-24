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

    std::cout << "Quanti boids vuoi per la simulazione?\n";
    std::cin >> n_boids;
    std::cout << "Formiscimi la distanza minima sotto la quale i boids si considereranno vicini";
    std::cin >> dist_vic;
    std::cout << "Forniscimi la distanza minima sotto la quale i boids inizieranno a separarsi";
    std::cin >> sep_dist;
     std::cout << "Separation factor: \n";
    std::cin >> sep_fact;
     std::cout << "Alignment factor: \n";
    std::cin >> align_fact;
     std::cout << "Coesion factor: \n";
    std::cin >> coes_fact;
     std::cout << "Durata della simulazione:\n";
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


    double Deltat{0.25};
    for (double t = 0; t < Intervalloditempo; t += Deltat)
    {
        for (lu_int i = 0; i < boid_vector.size(); ++i)
        {
            v_mod(i, sep_fact, sep_dist, align_fact, dist_vic, coes_fact, boid_vector);
            p_mod(i, boid_vector, Deltat);
        }
        double fieldwidth{15.};
        double fieldheight{10.};
        Pacman(boid_vector, fieldwidth, fieldheight);
    }

    std::cout << "Distanza media tra i boids" << bds::GetMeanDistance(boid_vector, sep_dist) << "+/-" << bds::GetStdDevDistance(boid_vector, sep_dist) << "\n";
    std::cout << "Velocità media dei boids" << bds::GetMeanVelocity(boid_vector) << "+/-" << bds::GetStdDevVelocity(boid_vector) << "\n";
}
