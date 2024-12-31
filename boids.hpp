#include "operators.hpp"

namespace bds
{ // questo namespace contiene ogni funzione o classe o cosa relativa ai boid (bds sta per boids)
    class boid
    {
    private:
        couple pos_; // posizione del boid
        couple vel_; // velocità del boid

    public:
        boid(couple p, couple s); // costruttore di base

        void vel_mod(couple s);
        void pos_mod(double deltat);
        void pos_mod(couple p);
        couple pos() const;
        couple vel() const; 
        double get_angle() const;
        couple& get_pos();// funzioni che uso per cavare fuori velocità e posizione dal boid
    };
    bool operator==(boid i, boid j);
    bool operator!=(boid i, boid j);

    void periodize(couple& pos, double perx, double pery);
    bool BoidsAreNear(boid i, boid j, double dist, double field_width, double field_height);

    couple v_separation(lu_int i, double sep_dist, double sep_fact, std::vector<boid> boid_vector,double field_width, double field_height);
    couple v_alignment(lu_int i, double alig_fact, std::vector<boid> boid_vector);
    couple v_coesion(lu_int i, double dist_vic, double coes_fact, std::vector<boid> boid_vector,double field_width, double field_height);
    couple v_random();
    void v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact, double dist_vic, double coes_fact, std::vector<boid> &boid_vector, double field_width, double field_height);
    void p_mod(lu_int i, std::vector<boid>& boid_vector, double deltat);
    
    void Pacman(std::vector<bds::boid> &boid_vector, lu_int i, double field_width, double field_height);
}


//TO_DO list
//togliere warning di conversione