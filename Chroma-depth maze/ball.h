
#include "math.h"
#include "../eigen-eigen-323c052e1731/Eigen/Dense"


class Ball
{
    public :
        float x, y, z;
        float radius = 10.f;
        float realRadius = 17.5;
        Eigen::Vector3f v; // Speed of the ball
        Eigen::Vector3f a; // Acceleration
        Ball()
        {
            x = 0.;
            y = 0.;
            z = 0.;
            v = Eigen::Vector3f(0., 0., 0.);
            a = Eigen::Vector3f(0., 0., 0.);
        };

        Ball(float x, float y)
        {
            this->x = x;
            this->y = y;
            z = 0.;
            v = Eigen::Vector3f(0., 0., 0.);
            a = Eigen::Vector3f(0., 0., 0.);
        }
        Ball(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            v = Eigen::Vector3f(0., 0., 0.);
            a = Eigen::Vector3f(0., 0., 0.);
        }

        Ball (float x, float y, float z, Eigen::Vector3f v)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->v = v;
            a = Eigen::Vector3f(0., 0., 0.);
        }

        Ball (float x, float y, Eigen::Vector3f v)
        {
            this->x = x;
            this->y = y;
            z = 0.;
            this->v = v;
            a = Eigen::Vector3f(0., 0., 0.);
        }
        ~Ball(){}

        void to4K(bool is4K)
        {
            if (is4K)
            {
                x *= 2.;
                y *= 2.;
                realRadius *= 2.;
                v *= 2.;
                a *= 2.;
            }
        }
};
