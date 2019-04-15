
#define _USE_MATH_DEFINES
#define MAP_PATH "Map.png"

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
    if (!map.loadFromFile(MAP_PATH))
    {
        cout << "Couldn't open the map file" << endl;
    }
    else
    cout << "Map file opened" << endl;

}

void buildSphereMap(Image pixelMap, MatrixXf &sphereMap, Matrix<Eigen::Vector3f, Dynamic, Dynamic> &normalMap)
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
            if (R > 0.9)
                z = (R - G / 2.) * zMax;

            else
            {
                if (R > 0.1)
                    z = R * zMax / 2.;

                else
                {
                    if (G > 0.9)
                        z = - B * zMax / 2.;

                    else
                    {
                        if (G > 0.1)
                            z = (G / 2 - B) * zMax;

                        else
                            z = (B - 1) * 2. * zMax;
                    }
                }
            }
            sphereMap(i, j)  = z;
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

            n = Eigen::Vector3f(20., 0., sphereMap(i+10, j) - sphereMap(i-10, j))
                    .cross(Eigen::Vector3f(0., 20., sphereMap(i, j+10) - sphereMap(i, j-10)));
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
    Eigen::Vector3f g(0., 0., -1.);   // Gravitation force

    Eigen::Vector3f beforePos(ball.x, ball.y, ball.z);
    Eigen::Vector3f v = ball.v;

    Eigen::Vector3f n(0., 0., 0.);
//    if (zMap((int) ball.x, (int) ball.y) == ball.z)
        n = normalMap((int) ball.x, (int) ball.y);
    v += 2. * timeElapsed.asSeconds() * (g + n);
    Eigen::Vector3f afterPos = beforePos + timeElapsed.asSeconds() * v;    //TODO: collisions

    Eigen::Vector3f direction = afterPos - beforePos;
    float distance = direction.norm();
    direction.normalize();

    Eigen::Vector3f pos = beforePos;
    for (float t = 0.; t < distance; t += 0.1)  //Need to adapt the incrementing of t
    {
        //TODO: check if hits something
        pos = beforePos + t * direction;
        if (zMap((int) pos(0) + 5, (int) pos(1) + 5) > pos(2))
        {
            pos(2) = zMap((int) pos(0), (int) pos(1));

            if (zMap((int) pos(0) + 5, (int) pos(1) + 5) - pos(2) > 20.)  // Need to check depending on the ball's diameter
            {
                v *= -1.;
            }
            break;
        }
    }


    ball.v = v;
    ball.x = max(min((float) zMap.rows()-1.f, pos(0)), 0.f);
    ball.y = max(min((float) zMap.cols()-1.f, pos(1)), 0.f);
    ball.z = pos(2);
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

    /* let's create the graphic image of the ball*/
    CircleShape pawn(10.f);
    pawn.setFillColor(sf::Color(0, 0, 0, 0));  /*pawn is transparent*/
    pawn.setOutlineThickness(10.f);
    pawn.setOutlineColor(sf::Color::White);
    pawn.setOrigin(10.f, 10.f);

    buildSphereMap(*chromaMap, zMap, nMap);

//    Ball ball(683., 350.);
    Ball ball(900, 500, 0, Eigen::Vector3f(10., 0., 0.));

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
        texture.update(*chromaMap);
        sprite.setTexture(texture);

        pawn.setPosition(ball.x, ball.y);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();

    }
    return 0;
}
