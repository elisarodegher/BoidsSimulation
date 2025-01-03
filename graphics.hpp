#include "boids.hpp" 

namespace bds {
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
        void setPosition(double x, double y);
        void draw(sf::RenderWindow& window);
        
    };
}