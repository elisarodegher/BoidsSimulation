#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "statistics.hpp"

TEST_CASE("Testing the operations of couples") {
  couple a{4., 7.};
  couple b{1., 18.};
  couple c{-3., -8.};
  couple minus_c{3., 8.};
  couple sum_ab{5., 25.};
  couple subtr_ab{3., -11.};   // a - b
  couple two_dot_a{12., 21.};  // 2 * a
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

TEST_CASE("Testing the squaresum") {
  couple a{4., 3.};
  couple b{-4., -3.};
  couple c{-1., 1.};

  couple zero{0., 0.};
  CHECK(squaresum(a) == 25.);
  CHECK(squaresum(b) == 25.);
  CHECK(squaresum(zero) == 0.);
  CHECK(squaresum(c) == 2.);
}

TEST_CASE("Testing the class member functions") {
  bds::boid tboid{
      couple{0., 0.},
      couple{0., 0.}}; 
  couple tvel{5., -3.};
  tboid.pos_mod(1.);  
  tboid.vel_mod(tvel); 

  CHECK(tboid.get_vel_value() == couple{5., -3});
  CHECK(tboid.get_pos_value() == couple{0., 0.});

  bds::boid xboid{{1., 1.}, {1., 2.}};
  xboid.vel_mod({1., 1.});
  xboid.pos_mod(1.);
  CHECK(xboid.get_vel_value() == couple{2., 3.});
  CHECK(xboid.get_pos_value() == couple{3., 4.});

  bds::boid yboid{{1., 1.}, {1., 2.}};  // boid con cui testiamo il costruttore normale
  yboid.vel_mod({1., 1.});
  yboid.pos_mod(1.);
  CHECK(yboid.get_vel_value() == couple{2., 3.});
  CHECK(yboid.get_pos_value() == couple{3., 4.});
}

TEST_CASE("Testing 'Boidsarenear' function") {
  bds::boid i{couple{0., 0.}, couple{0., 0.}};
  bds::boid j{couple{0., 0.}, couple{0., 0.}};
  double dist{5.};
  double field_width{15.};
  double field_height{10};
  CHECK(BoidsAreNear(i, j, dist, field_width, field_height) == 1);

  i.vel_mod({1.5, 2.});
  i.pos_mod(1.);
  CHECK(BoidsAreNear(i, j, dist, field_width, field_height) == 1);

  i.pos_mod(1.);
  CHECK(BoidsAreNear(i, j, dist, field_width, field_height) == 0);

  i.pos_mod(1.);
  CHECK(BoidsAreNear(i, j, dist, field_width, field_height) == 0);

  i.vel_mod({-3., -4.});
  i.pos_mod(3.);
  couple c{0., 0.};

  CHECK(i.get_pos_value() == c);
  CHECK(j.get_pos_value() == c);
  CHECK(BoidsAreNear(i, j, dist, field_width, field_height) == 1);
}



TEST_CASE("Separation Velocity Function") {
  double field_width{15.};
  double field_height{10.};

  std::vector<bds::boid> test_vector;
  couple pi{-2., 4};
  couple vi{0., 0.};  // se sono fermi non ce ne frega un cazzo
  bds::boid a{pi, vi};
  test_vector.push_back(a);  // boid 0

  pi = {4., 3.};
  bds::boid b{pi, vi};
  test_vector.push_back(b);  // boid 1

  pi = {1., 1.};
  bds::boid c{pi, vi};
  test_vector.push_back(c);  // boid 2

  pi = {-1., 0.};
  bds::boid d{pi, vi};
  test_vector.push_back(d);  // boid 3

  pi = {2., 0.};
  bds::boid e{pi, vi};
  test_vector.push_back(e);  // boid 4
  SUBCASE("Testing a - 2 boids in range") {
    double dist1{3.};
    couple v_sep =
        bds::v_separation(2, dist1, 2, test_vector, field_width, field_height);

    REQUIRE(v_sep.size() == 2);
    CHECK(v_sep[0] == 2);
    CHECK(v_sep[1] == 4);
  }

  SUBCASE("Testing b - 1 boid in range") {
    double dist2{1.5};
    couple v_sep =
        bds::v_separation(2, dist2, 2, test_vector, field_width, field_height);

    CHECK(v_sep[0] == -2);
    CHECK(v_sep[1] == 2);
  }

  SUBCASE("Testing c - no boids in range") {
    double dist3{1};
    couple v_sep =
        bds::v_separation(2, dist3, 2, test_vector, field_width, field_height);

    CHECK(v_sep[0] == 0);
    CHECK(v_sep[1] == 0);
  }

  SUBCASE("Testing d - no separation factor") {
    double dist1 = 3;
    couple v_sep =
        bds::v_separation(2, dist1, 0, test_vector, field_width, field_height);

    CHECK(v_sep[0] == 0);
    CHECK(v_sep[1] == 0);
  }

  SUBCASE("Testing e - 2 boids in the same position") {
    bds::boid f{{1., 1.}, {1., 1.}};
    test_vector.push_back(f);

    couple v_sep =
        bds::v_separation(2, 0.5, 1, test_vector, field_width, field_height);

    CHECK(v_sep[0] == 0);
    CHECK(v_sep[1] == 0);
  }  // forse da togliere

  SUBCASE("Testing f - behaviour near borders") {
    bds::boid g{{7, 1}, {0., 0.}};
    bds::boid h{{8, 2.}, {0., 0.}};
    bds::boid i{{-7, 1.}, {0., 0.}};

    std::vector<bds::boid> bord_vector;
    bord_vector.push_back(g);
    bord_vector.push_back(h);
    bord_vector.push_back(i);

    couple v_sep =
        bds::v_separation(0, 2, 2, bord_vector, field_width, field_height);

    CHECK(v_sep[0] == -4);
    CHECK(v_sep[1] == -2);
  }
}

TEST_CASE("Alignment Velocity Function") {
  couple pi{1., 3.};
  couple vi{3., 2.};
  std::vector<bds::boid> test_vector;  // vettore boids
  bds::boid a{pi, vi};                 // boid 0
  test_vector.push_back(a);

  pi = {3., 4.};
  vi = {1., 1.};
  bds::boid b{pi, vi};  // boid 1
  test_vector.push_back(b);

  pi = {2., 4.};
  vi = {3., 1.};
  bds::boid c{pi, vi};  // boid 2
  test_vector.push_back(c);

  pi = {3., 1.};
  vi = {5., 4.};
  bds::boid d{pi, vi};  // boid 3
  test_vector.push_back(d);

  pi = {1., 1.};
  vi = {2., 1.};
  bds::boid e{pi, vi};  // boid 4
  test_vector.push_back(e);

  SUBCASE("Testing a - different boids to align") {
    couple v_alig = bds::v_alignment(0, 0.5, test_vector);
    REQUIRE(v_alig.size() == 2);

    CHECK(v_alig[0] == doctest::Approx(-0.125));
    CHECK(v_alig[1] == doctest::Approx(-0.125));
  }

  SUBCASE("Testing b - no boids to align") {
    bds::boid single{{1., 2.}, {3., 4.}};
    std::vector<bds::boid> sing_vector{single};

    couple v_alig = bds::v_alignment(0, 56, sing_vector);

    CHECK(v_alig[0] == 0);
    CHECK(v_alig[1] == 0);
  }

  SUBCASE("Testing c - boids already aligned") {
    bds::boid a_0{{1., 2.}, {3., -6.}};
    bds::boid a_1{{3., 4.}, {3., -6.}};
    bds::boid a_2{{5., -2.}, {3., -6.}};
    bds::boid a_3{{-2, -3.}, {3., -6.}};

    std::vector<bds::boid> alig_vector;
    alig_vector.push_back(a_0);
    alig_vector.push_back(a_1);
    alig_vector.push_back(a_2);
    alig_vector.push_back(a_3);

    couple v_alig = bds::v_alignment(0, 1, alig_vector);

    CHECK(v_alig[0] == 0);
    CHECK(v_alig[1] == 0);
  }

  SUBCASE("Testing d - no alignment factor") {
    couple v_alig = bds::v_alignment(0., 0., test_vector);

    CHECK(v_alig[0] == 0);
    CHECK(v_alig[1] == 0);
  }

  SUBCASE("Testing e - two boids are the same") {
    bds::boid f{{1., 3.}, {3., 2.}};
    test_vector.push_back(f);

    couple v_alig = bds::v_alignment(0, 0.5, test_vector);

    CHECK(v_alig[0] == doctest::Approx(-0.1));
    CHECK(v_alig[1] == doctest::Approx(-0.1));
  }
}

TEST_CASE("Coesion Function") {
  double field_width{15.};
  double field_height{10.};
  couple pi{1., 3.};
  couple vi{3., 2.};
  std::vector<bds::boid> test_vector;  // vettore boids
  bds::boid a{pi, vi};                 // boid 0
  test_vector.push_back(a);

  pi = {3., 4.};
  vi = {1., 1.};
  bds::boid b{pi, vi};  // boid 1
  test_vector.push_back(b);

  pi = {2., 4.};
  vi = {3., 1.};
  bds::boid c{pi, vi};  // boid 2
  test_vector.push_back(c);

  pi = {3., 1.};
  vi = {5., 4.};
  bds::boid d{pi, vi};  // boid 3
  test_vector.push_back(d);

  pi = {1., 1.};
  vi = {2., 1.};
  bds::boid e{pi, vi};  // boid 4
  test_vector.push_back(e);

  double dist_vic;

  SUBCASE("Testing a - some boids in range") {
    dist_vic = 5;
    couple v_coes = bds::v_coesion(0, dist_vic, 0.5, test_vector, field_width,
                                   field_height);
    REQUIRE(v_coes.size() == 2);

    CHECK(v_coes[0] == doctest::Approx(0.625));
    CHECK(v_coes[1] == doctest::Approx(-0.25));
  }

  SUBCASE("Testing b - no boids in range") {
    dist_vic = 1;

    couple v_coes = bds::v_coesion(0, dist_vic, 0.5, test_vector, field_width,
                                   field_height);
    REQUIRE(v_coes.size() == 2);

    CHECK(v_coes[0] == 0.);
    CHECK(v_coes[1] == 0.);
  }

  SUBCASE("Testing c - all boids in the same position") {
    bds::boid c_0{{3., 2.}, {2., 4.}};
    bds::boid c_1{{3., 2.}, {3., -1.}};
    bds::boid c_2{{3., 2.}, {4., 6.}};
    bds::boid c_3{{3., 2.}, {6., 7.}};

    std::vector<bds::boid> centre_vector;
    centre_vector.push_back(c_0);
    centre_vector.push_back(c_1);
    centre_vector.push_back(c_2);
    centre_vector.push_back(c_3);

    dist_vic = 2;

    couple v_coes = bds::v_coesion(0, dist_vic, 0.5, centre_vector, field_width,
                                   field_height);
    REQUIRE(v_coes.size() == 2);

    CHECK(v_coes[0] == 0.);
    CHECK(v_coes[1] == 0.);
  }

  SUBCASE("Testing d - no coesion factor") {
    dist_vic = 5.;

    couple v_coes =
        bds::v_coesion(0, dist_vic, 0., test_vector, field_width, field_height);
    REQUIRE(v_coes.size() == 2);

    CHECK(v_coes[0] == 0.);
    CHECK(v_coes[1] == 0.);
  }

  SUBCASE("Testing e - behaviour near border") {
    bds::boid b_0{{7., -4.}, {3., 4.}};
    bds::boid b_1{{-6., -4.}, {3., 5.}};
    bds::boid b_2{{7., 4.}, {7., -1.}};

    std::vector<bds::boid> bord_vector;
    bord_vector.push_back(b_0);
    bord_vector.push_back(b_1);
    bord_vector.push_back(b_2);

    dist_vic = 3.;

    couple v_coes =
        bds::v_coesion(0, dist_vic, 2, bord_vector, field_width, field_height);

    CHECK(v_coes[0] == 2);
    CHECK(v_coes[1] == -2);
  }
}

TEST_CASE("Testing velocity modifier") {
    std::vector<bds::boid> test_vector;
    couple pi{0., 0};
    couple vi{0., 0.};
    bds::boid a{pi, vi};
    double field_width{15.};
    double field_height{15.};
    double rndm_mod{0.01};
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

    bds::wind null_wind{0., 0.};

    bds::v_mod(0, 0.5, 3., 1, 5., 4, test_vector, field_width, field_height, null_wind, rndm_mod);

    double av1 = test_vector[0].get_vel_value()[0];
    double av2 = test_vector[0].get_vel_value()[1];

    CHECK(av1 == doctest::Approx(1.).epsilon(0.02));
    CHECK(av2 == doctest::Approx(1.8333).epsilon(0.02));

    SUBCASE("Test with wind") {
      bds::wind some_wind{3., 0.};
      bds::v_mod(0, 0.5, 3., 1, 5., 4, test_vector, field_width, field_height, some_wind, rndm_mod);

      CHECK(test_vector[0].get_vel_value()[0] == doctest::Approx(1.3).epsilon(0.02));
      CHECK(test_vector[0].get_vel_value()[1] == doctest::Approx(1.8333).epsilon(0.02));

    }

}