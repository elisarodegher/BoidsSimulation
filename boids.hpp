#include <array>
#include <iostream>
#include <vector>
#include <cassert>
#include <numeric>
#include <cmath> //cose che vanno a finire anche nel sorgente

using couple = std::array<double, 2>;
using lu_int = long unsigned int; // ho messo questi due alias perchè scrivere ogni volta i nomi originali diventa un suicidio

/* OPERAZIONI TRA ARRAY */
couple operator+(couple const &add1, couple const &add2);

couple operator-(couple const &add1, couple const &add2);

couple operator*(double scal, couple const &vet);

double squaresum(couple i);

namespace bds
{ // questo namespace contiene ogni funzione o classe o cosa relativa ai boid (bds sta per boids)
    class boid
    {
    private:
        couple pos_; // posizione del boid
        couple vel_; // velocità del boid

    public:
        boid(couple p, couple s); // costruttore di base, forse da togliere
        boid();                   // costruttore senza parametri, tendenzialmente va sostituito con il randomico

        void vel_mod(couple s);
        void pos_mod(double deltat);
        couple pos() const;
        couple vel() const; // funzioni che uso per cavare fuori velocità e posizione dal boid
    };

    double squaredistance(boid i);
    bool BoidsAreNear(boid i, boid j, double dist);

    std::vector<boid> create_boids_vector(lu_int n);
    std::vector<boid> inizialization(lu_int n_boids);

    couple v_separation(lu_int i, double sep_dist, double sep_fact, std::vector<boid> boid_vector);
    couple v_alignment(lu_int i, double alig_fact, std::vector<boid> boid_vector);
    couple v_coesion(lu_int i, double dist_vic, double coes_fact, std::vector<boid> boid_vector);
    void v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact, double dist_vic, double coes_fact, std::vector<boid> &boid_vector);
    void p_mod(lu_int i, std::vector<boid> boid_vector, double deltat);

    double GetMeanDistance(std::vector<boid> boid_vector, double sep_dist);
    double GetMeanVelocity(std::vector<boid> boid_vector);
    double GetStdDevDistance(std::vector<boid> boid_vector, double sep_dist);
    double GetStdDevVelocity(std::vector<boid> boid_vector);
}

void Pacman(std::vector<bds::boid> &boid_vector, double field_width, double field_height);

