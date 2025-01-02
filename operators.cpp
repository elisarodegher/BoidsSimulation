#include "operators.hpp"

couple operator+(couple const &add1, couple const &add2) {
  double a = add1[0] + add2[0];
  double b = add1[1] + add2[1];
  return couple{a, b};
}  // somma di due array

couple operator-(couple const &add1, couple const &add2) {
  double a = add1[0] - add2[0];
  double b = add1[1] - add2[1];
  return couple{a, b};
}  // differenza tra due array

couple operator*(double scal, couple const &vet) {
  return couple{scal * vet[0], scal * vet[1]};
}  // prodotto tra coppia e scalare

double squaresum(couple i) {
  auto a = i.begin();
  auto b = i.end();
  double sum = std::inner_product(a, b, a, 0);
  return sum;
}  // quadrato del modulo (per una coppia "i" di dati in generale)

void operator+=(couple &add1, couple const &add2) { add1 = add1 + add2; }

void operator-=(couple &add1, couple const &add2) { add1 = add1 - add2; }

void operator*=(couple &add0, double mult) { add0 = (mult * add0); }