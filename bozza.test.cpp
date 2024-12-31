#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"

TEST_CASE("Testing the operations of couples")
{
    couple a{4., 7.};
    couple b{1., 18.};
    couple c{-3., -8.};
    couple minus_c{3., 8.};
    couple sum_ab{5., 25.};
    couple subtr_ab{3., -11.};  // a meno b
    couple two_dot_a{12., 21.}; // a moltiplicato per due
    couple sum_ac{1., -1.};
    couple sum_bc{-2., 10.};
    couple zero{0., 0.};
    CHECK(a + b == sum_ab);
    CHECK(a - b == subtr_ab);
    CHECK(3 * a == two_dot_a);
    CHECK(a + c == sum_ac);
    CHECK(b + c == sum_bc);
    CHECK(0 * a == zero);
    CHECK(c + minus_c == zero);
    CHECK(5 * zero == zero);
}

TEST_CASE("Testing the squaresum")
{
    couple a{4., 3.};
    couple b{-4., -3.};
    couple c{-1., 1.};

    couple zero{0., 0.};
    CHECK(squaresum(a) == 25.);
    CHECK(squaresum(b) == 25.);
    CHECK(squaresum(zero) == 0.);
    CHECK(squaresum(c) == 2.);
}

TEST_CASE("Testing the class member functions")
{
    bds::boid tboid{}; // boid con cui testiamo il costruttore nullo
    couple tvel{5., -3.};
    tboid.pos_mod(1.);   // la posizione viene modificata con questo deltat (1sec), dovrebbe rimanere uguale
    tboid.vel_mod(tvel); // velocità del test boid settata a 5,-3.
    couple ais{5., -3};
    couple bis{0., 0.};

    CHECK(tboid.vel() == ais);
    CHECK(tboid.pos() == bis);

    couple xpos{1., 1.};
    couple xvel{1., 2.};
    bds::boid xboid{xpos, xvel}; // boid con cui testiamo il costruttore normale
    couple added_vel{1., 1.};
    xboid.vel_mod(added_vel);
    couple new_vel{2., 3.};
    couple new_pos{3., 4.};
    xboid.pos_mod(1.);
    CHECK(xboid.vel() == new_vel);
    CHECK(xboid.pos() == new_pos);


    couple ypos{1., 1.};
    couple yvel{1., 2.};
    bds::boid yboid{ypos, yvel}; // boid con cui testiamo il costruttore normale
    couple added_vel{1., 1.};
    xboid.vel_mod(added_vel);
    couple new_vel{2., 3.};
    couple new_pos{3., 4.};
    xboid.pos_mod(1.);
    CHECK(xboid.vel() == new_vel);
    CHECK(xboid.pos() == new_pos);


}

TEST_CASE("Testing 'Boidsarenear' function") {
    bds::boid i{};
    bds::boid j{};
    double dist{5.};
    CHECK(BoidsAreNear(i, j, dist) == 0);

    couple ivel{1.5, 2.};
    i.vel_mod(ivel);
    i.pos_mod(1.);
    CHECK(BoidsAreNear(i, j, dist) == 1);

    i.pos_mod(1.);
    CHECK(BoidsAreNear(i, j, dist) == 0);

    i.pos_mod(1.);
    CHECK(BoidsAreNear(i, j, dist) == 0);


    ivel = {-3., -4.};
    i.vel_mod(ivel);
    i.pos_mod(3.);
    couple c{0., 0.};

    CHECK(i.pos() == c);
    CHECK(j.pos() == c);
    CHECK(BoidsAreNear(i, j, dist) == 1);
}



TEST_CASE("Testing velocity modifier") {
    std::vector<bds::boid> test_vector;
    couple pi{0., 0};
    couple vi{0., 0.}; //se sono fermi non ce ne frega un cazzo
    bds::boid a{pi, vi};
    test_vector.push_back(a);


    pi = {1., 2.};
    vi = {1., 1.};
    bds::boid b{pi, vi};
    test_vector.push_back(b);

    pi = {-2., 1.};
    vi = {1., 0.};
    bds::boid c{pi, vi};
    test_vector.push_back(c);

    pi = {1., -2.};
    vi = {1., 2.};
    bds::boid d{pi, vi};
    test_vector.push_back(d);

    bds::v_mod(0, 0.5, 3., 1, 5., 4, test_vector);
    
    double av1 = test_vector[0].vel()[0];
    double av2 = test_vector[0].vel()[1];

    CHECK(av1 == 0.75);
    CHECK(av2 == 0.75);
    
}

TEST_CASE("Testing the fly rules"){

    SUBCASE("Separation Velocity Function"){

        std::vector<bds::boid> test_vector;
        couple pi{-2., 4};
        couple vi{0., 0.}; //se sono fermi non ce ne frega un cazzo
        bds::boid a{pi, vi};
        test_vector.push_back(a); //boid 0 

        pi = {4., 3.};
        bds::boid b{pi, vi};
        test_vector.push_back(b); //boid 1

        pi = {1., 1.};
        bds::boid c{pi, vi};
        test_vector.push_back(c); //boid 2

        pi = {-1., 0.};
        bds::boid d{pi, vi};
        test_vector.push_back(d); //boid 3

        pi = {2., 0.};
        bds::boid e{pi, vi};
        test_vector.push_back(e); //boid 4

        double dist1{3.};
        lu_int i{2};
        couple v_sep;
        v_sep = bds::v_separation(i, dist1, 2, test_vector);

        REQUIRE(v_sep.size() == 2);
        CHECK(v_sep[0] == 2);
        CHECK(v_sep[1] == 4);
        

        double dist2 {1.5};
        v_sep = bds::v_separation(i, dist2, 2, test_vector);

        CHECK(v_sep[0] == -2);
        CHECK(v_sep[1] == 2);

        double dist3 {1};
        v_sep = bds::v_separation(i, dist3, 2, test_vector);

        CHECK(v_sep[0] == 0);
        CHECK(v_sep[1] == 0);

        v_sep = bds::v_separation(i, dist1, 0, test_vector);

        CHECK(v_sep[0] == 0);
        CHECK(v_sep[1] == 0);
    }

    SUBCASE("Alignment Velocity Function") {

        lu_int i{0}; //testiamo la funzione applicata al boid 0

        couple pi{1., 3.};
        couple vi{3.,2.};
        std::vector<bds::boid> test_vector; // vettore boids
        bds::boid a{pi, vi}; //boid 0
        test_vector.push_back(a);

        pi = {3.,4.};
        vi = {1.,1.};
        bds::boid b{pi, vi}; //boid 1
        test_vector.push_back(b); 

        pi = {2.,4.};
        vi = {3.,1.};
        bds::boid c{pi, vi}; //boid 2
        test_vector.push_back(c);

        pi = {3.,1.};
        vi = {5.,4.};
        bds::boid d{pi, vi}; //boid 3
        test_vector.push_back(d);

        pi = {1.,1.};
        vi = {2.,1.};
        bds::boid e{pi, vi}; //boid 4
        test_vector.push_back(e);



        couple v_alig = bds::v_alignment(i,0.5,test_vector);
        REQUIRE(v_alig.size()==2);

        CHECK(v_alig[0] == doctest::Approx(2.25));
        CHECK(v_alig[1] == doctest::Approx(-0.5)); 


}

    SUBCASE("Coesion Function"){

        lu_int i{0}; //testiamo la funzione applicata al boid 0

        couple pi{1., 3.};
        couple vi{3.,2.};
        std::vector<bds::boid> test_vector; // vettore boids
        bds::boid a{pi, vi}; //boid 0
        test_vector.push_back(a);

        pi = {3.,4.};
        vi = {1.,1.};
        bds::boid b{pi, vi}; //boid 1
        test_vector.push_back(b); 

        pi = {2.,4.};
        vi = {3.,1.};
        bds::boid c{pi, vi}; //boid 2
        test_vector.push_back(c);

        pi = {3.,1.};
        vi = {5.,4.};
        bds::boid d{pi, vi}; //boid 3
        test_vector.push_back(d);

        pi = {1.,1.};
        vi = {2.,1.};
        bds::boid e{pi, vi}; //boid 4
        test_vector.push_back(e);

        double dist_vic = 1.

        couple v_coes = bds::v_coesion(i, dist_vic, 0.5, test_vector);
        REQUIRE(v_coes.size()==2);

        CHECK(v_coes[0] == doctest::Approx(2.25));
        CHECK(v_coes[1] == doctest::Approx(-0.5)); 

    }

}