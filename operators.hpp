#ifndef OPERATORS_HPP
#define OPERATORS_HPP


#include <array>
#include <iostream>
#include <vector>
#include <cassert>
#include <numeric>
#include <cmath> 
#include <random>
#include <ctime>
#include <algorithm>

using couple = std::array<double, 2>;
using lu_int = long unsigned int; 

couple operator+(couple const &add1, couple const &add2);

couple operator-(couple const &add1, couple const &add2);

couple operator*(double scal, couple const &vet);

double squaresum(couple i);

void operator+=(couple &add1, couple const &add2);

void operator-=(couple &add1, couple const &add2);

void operator*=(couple &couple, double mult);

double to_degrees(double ang_rad);

double to_radians(double ang_deg);

#endif