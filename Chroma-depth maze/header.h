
#include <SFML/Graphics.hpp>
#include "math.h"
#include "../eigen-eigen-323c052e1731/Eigen/Dense"


class Ball
{
    public :
        float x, y, z;
        // Vector3f v; //Not used yet
        Ball()
        {
            x = 0.;
            y = 0.;
            z = 0.;
        };

        Ball(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        ~Ball(){}
};

class Arguments
{
public:
    sf::RenderWindow* window;
    sf::Image* image;
    Ball* ball;

    Arguments(sf::RenderWindow* w, sf::Image* i, Ball* b)
    {
        window = w;
        image = i;
        ball = b;
    }

};
