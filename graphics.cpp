#include "graphics.hpp"

bds::GraphicBoids::GraphicBoids() {
  sup.setPointCount(3);
  sup.setPoint(0, sf::Vector2f(0.f, 0.f));
  sup.setPoint(1, sf::Vector2f(9.f, 3.f));
  sup.setPoint(2, sf::Vector2f(2.f, 3.f));

  inf.setPointCount(3);
  inf.setPoint(0, sf::Vector2f(0.f, 6.f));
  inf.setPoint(1, sf::Vector2f(2.f, 3.f));
  inf.setPoint(2, sf::Vector2f(9.f, 3.f));

  sup.setFillColor(sf::Color::Black);
  inf.setFillColor(sf::Color::Blue);

  sup.setOrigin(sf::Vector2f(3., 3.));
  inf.setOrigin(sf::Vector2f(3., 3.));
}

void bds::GraphicBoids::move(double x, double y) {
  inf.move(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
  sup.move(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
}

void bds::GraphicBoids::rotate(double ang) {
  inf.rotate(static_cast<float>(ang));
  sup.rotate(static_cast<float>(ang));
}

void bds::GraphicBoids::setPosition(double x, double y) {
  inf.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
  sup.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
}

void bds::GraphicBoids::draw(sf::RenderWindow& window) {
  window.draw(sup);
  window.draw(inf);
}

bds::GraphicWind::GraphicWind(wind w) : wind_line{sf::RectangleShape(sf::Vector2f(60., 3.))} {
    wind_arrow.setPointCount(5);
    wind_arrow.setPoint(0, sf::Vector2f(60.f, -2.f));
    wind_arrow.setPoint(1, sf::Vector2f(66.f, 1.5f));
    wind_arrow.setPoint(2, sf::Vector2f(60.f, 5.f));
    wind_arrow.setPoint(3, sf::Vector2f(57.f, 3.f));
    wind_arrow.setPoint(4, sf::Vector2f(57.f, 0.f));

    wind_line.setOrigin(sf::Vector2f(33.f, 1.5f));
    wind_arrow.setOrigin(sf::Vector2f(33.f, 1.5f));
 
 sf::Color lightRed(217, 88, 88);
    wind_line.setFillColor(sf::Color::Red);
    wind_arrow.setFillColor(lightRed);

    wind_line.setRotation(static_cast<float>(to_degrees(w.get_angle_rad())));
    wind_arrow.setRotation(static_cast<float>(to_degrees(w.get_angle_rad())));
}

void bds::GraphicWind::draw(sf::RenderWindow& window) {
    window.draw(wind_line);
    window.draw(wind_arrow);
}

void bds::GraphicWind::setPosition(double x, double y) {
    wind_line.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
    wind_arrow.setPosition(sf::Vector2f(static_cast<float>(x), static_cast<float>(y)));
}