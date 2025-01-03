#include "operators.hpp"
#include <SFML/Graphics.hpp>

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
        couple get_pos_value() const;
        couple get_vel_value() const; 
        double get_angle() const;
        couple& get_pos_ref();// funzioni che uso per cavare fuori velocità e posizione dal boid
    };

    class wind {
        private:
        double intensity_;
        double angle_;
        public:
        wind(double intensity, double angle);

        void rotate(double rot_ang);
        couple get_coordinates();
        double get_angle_rad();
    };

    void periodize(couple& pos, double perx, double pery);
    bool BoidsAreNear(boid i, boid j, double dist, double field_width, double field_height);

    couple v_separation(lu_int i, double sep_dist, double sep_fact, const std::vector<boid>& boid_vector,double field_width, double field_height);
    couple v_alignment(lu_int i, double alig_fact, const std::vector<boid>& boid_vector);
    couple v_coesion(lu_int i, double dist_vic, double coes_fact, const std::vector<boid>& boid_vector,double field_width, double field_height);
    couple v_random(double rndm_mod);
    couple v_wind(wind i_wind);
    void v_mod(lu_int i, double sep_fact, double sep_dist, double alig_fact, double dist_vic, double coes_fact, std::vector<boid> &boid_vector, double field_width, double field_height, wind gen_wind);
    void p_mod(lu_int i, std::vector<boid>& boid_vector, double deltat);
    
    void Pacman(std::vector<bds::boid> &boid_vector, lu_int i, double field_width, double field_height);

    class GraphicBoids {
        private:
        sf::ConvexShape sup;
        sf::ConvexShape inf;

        public:
        GraphicBoids();
        void move(double x, double y);
        void rotate(double ang);
        void setPosition(double x, double y);
        void draw(sf::RenderWindow& window);

    };

    class GraphicWind {
        private:
        sf::RectangleShape wind_line;
        sf::ConvexShape wind_arrow;

        public:
        GraphicWind(wind w);
        void draw(sf::RenderWindow& window);
    };
};

//risolvere problemi nei test (lacrime)
//impostare una buona interfaccia utente
//curare l'uniformità dei test
//curare i nomi delle funzioni
//curare il vento (o toglierlo)