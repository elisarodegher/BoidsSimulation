#include <array>
#include <iostream>
#include <vector>
#include <cassert>
#include <numeric>
#include <cmath> 
#include <random>
#include <ctime>
#include <algorithm>//cose che vanno a finire anche nel sorgente

using couple = std::array<double, 2>;
using lu_int = long unsigned int; // ho messo questi due alias perchè scrivere ogni volta i nomi originali diventa un suicidio

/* OPERAZIONI TRA ARRAY */
couple operator+(couple const &add1, couple const &add2);

couple operator-(couple const &add1, couple const &add2);

couple operator*(double scal, couple const &vet);

double squaresum(couple i);

void operator+=(couple &add1, couple const &add2);

void operator-=(couple &add1, couple const &add2);

void operator*=(couple &couple, double mult);
