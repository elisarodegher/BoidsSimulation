#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boids.hpp"

TEST_CASE("Testing the sum of couples") {
    //da fare dopo tanto dovrebbe andare, insieme anche alle altre operazioni tra array

}

TEST_CASE("Testing the class member functions") {
    bds::boid tboid{};
    couple tvel{5., -3.};
    tboid.pos_mod(1.);
    tboid.vel_mod(tvel);
    couple ais{5., -3.};
    couple bis{0., 0.};

    CHECK(tboid.vel() == ais);
    CHECK(tboid.pos() == bis);

    tboid.pos_mod(1.);
    bis = {5., -3.};

    CHECK(tboid.pos() == bis);
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

TEST_CASE("Testing separation velocity function") {
    std::vector<bds::boid> test_vector;
    couple pi{-2., 4};
    couple vi{0., 0.}; //se sono fermi non ce ne frega un cazzo
    bds::boid a{pi, vi};
    test_vector.push_back(a);

    pi = {4., 3.};
    bds::boid b{pi, vi};
    test_vector.push_back(b);

    pi = {1., 1.};
    bds::boid c{pi, vi};
    test_vector.push_back(c);

    pi = {-1., 0.};
    bds::boid d{pi, vi};
    test_vector.push_back(d);

    pi = {2., 0.};
    bds::boid e{pi, vi};
    test_vector.push_back(e);

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

