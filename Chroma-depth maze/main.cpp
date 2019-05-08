
#define _USE_MATH_DEFINES
#define MAP_PATH "map.png"
#define ZMAP_PATH "z_map.png"

#include <string>
#include <thread>
#include <iostream>
#include <math.h>
#include "header.h"

using namespace Eigen;
using namespace std;
using namespace sf;


void loadMap (const string mapPath, Image &map)
{
    if (!map.loadFromFile(mapPath))
    {
        cout << "Couldn't open the map file" << endl;
    }
    else
    cout << "Map file opened" << endl;

}

void buildSphereMap(const string zMapPath, MatrixXf &sphereMap, Matrix<Eigen::Vector3f, Dynamic, Dynamic> &normalMap)
{
    Image pixelMap;
    if (!pixelMap.loadFromFile(zMapPath))
    {
        cout << "Couldn't open the zmap file" << endl;
    }
    else
    {
       float zMax = 100.;
        cout << "Started loading zMap" << endl;

        Vector2u size = pixelMap.getSize();
        sphereMap.resize(size.x, size.y);

        normalMap.resize(sphereMap.rows(), sphereMap.cols());

        cout << "Resized zmap to " << size.x << "; " << size.y << endl;
        for (unsigned int i = 0; i < size.x; i++)
        {
            for (unsigned int j = 0; j < size.y; j++)
            {
                float z = -INFINITY;
                Color color = pixelMap.getPixel(i, j);
                float R = (float) color.r / 255.;
                float G = (float) color.g / 255.;
                float B = (float) color.b / 255.;

                //sets the depth depending on the color
                /* /!\Doesn't check a few things, like blue when there's red, etc. /!\ */
                if (R > 0.99)
                    z = (R - G / 2. + B / 2.) * zMax;

                else
                {
                    if (R > 0.01)
                        z =(1.- G + R) * zMax / 2.;

                    else
                    {
                        if (G > 0.9)
                            z =(G - 1. - B) * zMax / 2.;

                        else
                        {
                            if (G > 0.025)
                                z = (G / 2 - B) * zMax;

                            else
                                z = (B - 2.) * zMax;
                        }
                    }
                }
                sphereMap(i, j)  = z;
            }
        }
    }

    /*Let's create the normal map
    To have a good normal, we have a 10pixel margin where the normal is not right*/
    for (int i = 0; i < normalMap.rows(); i++)
    {
        for (int j = 0; j < normalMap.cols(); j++)
        {
            normalMap(i, j) = Eigen::Vector3f(0., 0., 1.);
        }
    }

    for (int i = 10; i < normalMap.rows() - 10; i++)
    {
        for (int j = 10; j < normalMap.cols() - 10; j++)
        {
            Eigen::Vector3f n;

            //TODO: don't count if difference is too high
            float iPlus = sphereMap(i+10, j);
            float iMinus = sphereMap(i-10, j);
            float jPlus = sphereMap(i, j+10);
            float jMinus = sphereMap(i, j-10);
            float zNormal = sphereMap(i, j);

            if (abs(iPlus - zNormal) > 30.)
                iPlus = zNormal;
            if (abs(iMinus - zNormal) > 30.)
                iMinus = zNormal;
            if (abs(jPlus - zNormal) > 30.)
                jPlus = zNormal;
            if (abs(jMinus - zNormal) > 30.)
                jMinus = zNormal;

            n = Eigen::Vector3f(20., 0., iPlus - iMinus)
                    .cross(Eigen::Vector3f(0., 20., jPlus - jMinus));
            n.normalize();
            normalMap(i, j) = n;
        }
    }

    /*Not sure smoothing is a good idea, need to find a way for red walls*/
//    cout << "Raw zMap done" << endl;
//    /*Let's smooth the values*/
//    for (int i = 0; i < m.rows(); i++)
//    {
//        for (int j = 0; j < m.cols(); j++)
//        {
//            /*This method is really slow...*/
//            int count = 0;
//            sphereMap(i, j) = 0.;
//            for (int k = max(i - 10, 0); k < min(i + 10, m.rows() - 1); k+= 10)
//            {
//                for (int l = max(j - 10, 0); l < min(j + 10, m.cols() - 1); l+= 10)
//                {
//                    sphereMap(i, j) += m(k, l);
//                    count++;
//                }
//            }
//            sphereMap(i, j) /= (float) count;
//        }
//    }

    cout << "Finished loading zMap" << endl;
};

void makeFall(Ball &ball, const MatrixXf map)
{
    /*For the moment it doesn't include any speed, gravitation force or anything,
    it just goes down at eache frame*/
    //TODO: implement a real gravity

    //For now we assume pixel.x & pixel.y are equal to position in table map
    int i = (int) ball.x;
    int j = (int) ball.y;
    float z = map(i, j);
    int x = i;
    int y = j;

    // Get neighbour with lowest z
//    cout << map(i-1, j-1) << ";" << map(i-1, j) << ";" << map(i-1, j+1) << endl;
//    cout << map(i, j-1) << ";" << map(i, j) << ";" << map(i, j+1) << endl;
//    cout << map(i+1, j-1) << ";" << map(i+1, j) << ";" << map(i+1, j+1) << endl;
    if (map(i-10, j-10) < z && i>9 && j>9)
    {
        z = map(i-1, j-1);
        x = i-1;
        y = j-1;
    }

    if (map(i-10, j) <= z && i>9)
    {
        z = map(i-1, j);
        x = i-1;
        y = j;
    }

    if (map(i-10, j+10) < z && i > 9 && j < map.cols() - 10)
    {
        z = map(i-1, j+1);
        x = i-1;
        y = j+1;
    }

    if (map(i, j-10) <= z && j > 9)
    {
        z = map(i, j-1);
        x = i;
        y = j-1;
    }

    if (map(i, j+10) <= z && j < map.cols() - 10)
    {
        z = map(i, j+1);
        x = i;
        y = j+1;
    }

    if (map(i+10, j-10) < z && i < map.rows() - 10 && j > 9)
    {
        z = map(i+1, j-1);
        x = i+1;
        y = j-1;
    }

    if (map(i+10, j) <= z && i < map.rows() - 10)
    {
        z = map(i+1, j);
        x = i+1;
        y = j;
    }

    if (map(i+10, j+10) < z && i < map.rows() - 10 && j < map.cols() - 10)
    {
        z = map(i+1, j+1);
        x = i+1;
        y = j+1;
    }

    ball.x = (float) x;
    ball.y = (float) y;
    ball.z = z;
};

void makeFallWithNormals(Ball &ball, const MatrixXf zMap, const Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap, const Time timeElapsed)
{
    //TODO: frame rate is too slow (3fps)
    Eigen::Vector3f g(0., 0., -1.);   // Gravitation force

    Eigen::Vector3f beforePos(ball.x , ball.y, ball.z);
    Eigen::Vector3f v = ball.v;

    Eigen::Vector3f n(0., 0., 0.);
    if (ball.z - zMap((int) ball.x, (int) ball.y) < 5.)
        n = normalMap((int) ball.x, (int) ball.y);
    v += 10. * timeElapsed.asSeconds() * (g + n);

    float distance = (timeElapsed.asSeconds() * v).norm();
    Eigen::Vector3f direction = v.normalized();

    Eigen::Vector3f pos = beforePos;
        cout << pos(0) << "; " << pos(1) << endl;

    float radius = ball.radius;

    //Collisions
    for (float t = 0.; t < distance; t += 0.1)  //Need to adapt the incrementing of t
    {
        pos = beforePos + t * direction;
        pos(0) = max(min(zMap.rows()-1.f, pos(0)), 0.f);
        pos(1) = max(min(zMap.cols()-1.f, pos(1)), 0.f);

        if (zMap((int) pos(0), (int) pos(1)) - pos(2) > 0.5)
        {
            radius = 10. + zMap((int) pos(0), (int) pos(1)) / 20.;
            pos += radius * direction;
            pos(0) = max(min(zMap.rows()-1.f, pos(0)), 0.f);
            pos(1) = max(min(zMap.cols()-1.f, pos(1)), 0.f);

            if (zMap((int) pos(0), (int) pos(1)) - pos(2) > 2*radius)  // There's a wall
            {
                // If we want the ball to bounce we need to have some data about the "orientation" of the wall, for now we'll just stop the speed
                v = Eigen::Vector3f(0., 0., 0.);
                pos -= t * direction;
            }
            else
            {
                v(2) = 0.;
            }
            pos -= radius * direction;
            break;
        }
    }


    ball.v = v;
    ball.x = pos(0);
    ball.y = pos(1);
    ball.z = zMap((int) ball.x, (int) ball.y);
    ball.radius = 10. + ball.z / 20.;
    cout << "After: " << v(0) << "; " << v(1) << "; " << v(2) << endl;
}

void moveBall(Ball* ball)
{
    //TODO: get joystick or keyboard input to move the ball
};

void moveWorld()
{
    //TODO: get other joystick input to turn the world
};

void wrapMapToSphere(Image image)
{
    //TODO: the function, but later
};

void saveZMap(MatrixXf zMap, const string fileName)
{
    Image img;
    img.create(zMap.rows(), zMap.cols());

    for(int i = 0; i < zMap.rows(); i++)
    {
        for (int j = 0; j < zMap.cols(); j++)
        {
            float z = (200. + zMap(i, j)) / 300. * 255.;
            img.setPixel(i, j, Color(z, z, z));
        }
    }

    if (!img.saveToFile(fileName))
    {
        cout << "Couldn't save the zMap file" << endl;
    }
}

int main( int argc, char * argv[] )
{
    RenderWindow window(VideoMode(1366, 768), "Chroma-depth maze"); //Need to config size for Geo-cosmos
    Image* chromaMap = new Image();
    loadMap(MAP_PATH, *chromaMap);

    MatrixXf zMap;
    Matrix<Eigen::Vector3f, Dynamic, Dynamic> nMap;
    //TODO: create a good map with photoshop (kinda finished)

    Sprite sprite;
    Texture texture;
    texture.loadFromImage(*chromaMap);
    sprite.setTexture(texture);

    /* let's create the graphic image of the ball*/
    CircleShape pawn(10.f);
    pawn.setFillColor(sf::Color(0, 0, 0));

    buildSphereMap(ZMAP_PATH, zMap, nMap);
    saveZMap(zMap, "zMap_grey.png");

    Ball ball(683., 350.);
//    Ball ball(950, 480, zMap(950, 480), Eigen::Vector3f(40., -15., 0.));

    Clock clock;

    while(window.isOpen())
    {
        Event event;
        while(window.pollEvent(event))
        {
            if(event.type == Event::Closed)
            {
                window.close();
                return 0;
            }

            //TODO: add joystick events to moveBall or moveWorld
        }

        /*Apply the gravitation physics*/
//        makeFall(ball, zMap);
        Time elapsed = clock.restart();
        makeFallWithNormals(ball, zMap, nMap, elapsed);


        //TODO: add an arrival check, later
        //TODO: adjust frame rate
//        texture.update(*chromaMap);

        pawn.setRadius(ball.radius);
        pawn.setPosition(ball.x - ball.radius, ball.y - ball.radius);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();

    }
    return 0;
}
