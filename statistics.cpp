#include "statistics.hpp"

double bds::GetMeanDistance(std::vector<boid> boid_vector)
{
    double dist_sum{0.};
    int n_events{0};

    dist_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& d_s, boid& i_boid) {
        double local_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& ds, boid& j_boid) {
            ds += sqrt(squaresum(i_boid.pos() - j_boid.pos()));
            ++n_events;
        return ds;
    });
    return d_s + local_sum;
    });

    dist_sum = dist_sum / 2; //doppio delle distanze sommate


    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            if (BoidsAreNear(i_boid, j_boid, sep_dist, field_width, field_height))
            { // ho messo un controllo vicini qua perchè non credo abbia senso considerare nel calcolo della media la distanza tra i boid megalontani (ma forse lo ha...)
                dist_sum += sqrt(squaresum(i_boid.pos() - j_boid.pos()));
                ++n_events; // perchè non so a priori quante sono le coppie di boids che contribuiscono alla determinazione della distanza media
            }
        }
    } */
    double meandist = dist_sum / n_events;
    return meandist;
} // tira fuori la distanza media tra i boids di un dato vettore, da chiamare nel ciclo

double bds::GetMeanVelocity(std::vector<boid> boid_vector)
{
    double vel_sum{0.};

    vel_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& vs, boid& i_boid) {
        vs += sqrt(squaresum(i_boid.vel()));
        return vs;
    });

    /*for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        vel_sum += sqrt(squaresum(i_boid.vel()));
    } */
    double meanvel = vel_sum / static_cast<double>(boid_vector.size());
    return meanvel; 
} // tira fuori la velocità media dei boids, da chiamare nel ciclo

double bds::GetStdDevDistance(std::vector<boid> boid_vector)
{
    double meandist = GetMeanDistance(boid_vector);
    double sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&](double &pass_sum, boid &i_boid) {
        double local_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double &dm, boid &j_boid){
            dm = (sqrt(squaresum(i_boid.pos() - j_boid.pos())) - meandist);
            dm *= dm;
            return dm;
        });
        pass_sum += local_sum;
        return pass_sum;
    });
    /*double square_dist_sum{0.};
    double square_dist_sum_2{0};

    square_dist_sum_2 = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0}, [&] (double& d_s, boid& i_boid) {
    double local_sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double& ds, boid& j_boid) {
            ds += squaresum(i_boid.pos() - j_boid.pos());
        return ds;
    });
    return d_s + local_sum;
    });

    square_dist_sum_2 = square_dist_sum_2 / 2;

    for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        for (lu_int j = 0; j < boid_vector.size(); ++j)
        {
            boid j_boid = boid_vector[j];
            square_dist_sum += squaresum(i_boid.pos() - j_boid.pos());
        }
    } 
    if(square_dist_sum_2 != square_dist_sum) {
        std::cout << "Problemo! \n" << square_dist_sum_2 << ", " << square_dist_sum << '\n'; 
    }
    double stddev = (square_dist_sum / static_cast<double>(boid_vector.size())) - (meandist * meandist); */
    double stddev = sqrt(sum / (static_cast<double>(boid_vector.size()) * (static_cast<double>(boid_vector.size()) -1 )
    * 2 ));
    return stddev;
} // calcolo della deviazione standard come vuole la sara magica (quadrato della media - media dei quadrati)

double bds::GetStdDevVelocity(std::vector<boid> boid_vector)
{
    double meanvel = GetMeanVelocity(boid_vector);
    //double square_vel_sum{0.};
    double sum;

    sum = std::accumulate(boid_vector.begin(), boid_vector.end(), double{0.}, [&] (double &sm, boid &i_boid) {
        sm = (sqrt(squaresum(i_boid.vel())) - meanvel);
        sm *= sm;
        return sm;
    });

    /* for (lu_int i = 0; i < boid_vector.size(); ++i)
    {
        boid i_boid = boid_vector[i];
        square_vel_sum += squaresum(i_boid.vel()); 
    } */

    double stddev = sqrt( sum / (static_cast<double>(boid_vector.size()) * (static_cast<double>(boid_vector.size()) -1 )
    * 2 ));
    return stddev;
} // come prima
