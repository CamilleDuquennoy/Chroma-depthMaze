
#include <SFML/Graphics.hpp>
#include "math.h"
#include "../eigen-eigen-323c052e1731/Eigen/Dense"


class Ball
{
    public :
        float x, y, z;
        float radius = 10.f;
        Eigen::Vector3f v; // Speed of the ball
        Ball()
        {
            x = 0.;
            y = 0.;
            z = 0.;
        };

        Ball(float x, float y)
        {
            this->x = x;
            this->y = y;
            z = 0.;
            v = Eigen::Vector3f(0., 0., 0.);
        }
        Ball(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            v = Eigen::Vector3f(0., 0., 0.);
        }

        Ball (float x, float y, Eigen::Vector3f v)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->v = v;
        }

        ~Ball(){}
};
