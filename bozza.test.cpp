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


