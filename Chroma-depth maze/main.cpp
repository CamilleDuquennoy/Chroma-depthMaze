
#define _USE_MATH_DEFINES
#define MAP_PATH "Map.png"

#include <string>
#include <thread>
#include <iostream>
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

void buildSphereMap(Image pixelMap, MatrixXf &sphereMap)
{
    //TODO: have some average for z
    float zMax = 10.;
    cout << "Started loading zMap" << endl;

    Vector2u size = pixelMap.getSize();
    sphereMap.resize(size.x, size.y);

    MatrixXf m(sphereMap.rows(), sphereMap.cols());

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
            m(i, j)  = z;
        }
    }

    cout << "Raw zMap done" << endl;
    /*Let's smooth the values*/
    for (int i = 0; i < m.rows(); i++)
    {
        for (int j = 0; j < m.cols(); j++)
        {
            int count = 0;
            sphereMap(i, j) = 0.;
            for (int k = max(i - 5, 0); k < min(i + 5, m.rows() - 1); k++)
            {
                for (int l = max(j - 5, 0); l < min(j + 5, m.cols() - 1); l++)
                {
                    sphereMap(i, j) += m(k, l);
                    count++;
                }
            }
            sphereMap(i, j) /= (float) count;

//            if(i == 0)
//            {
//                if (j == 0)
//                {
//                    sphereMap(i, j) = (
//                       m(i,j) +    m(i,j+1) +
//                       m(i+1,j) +  m(i+1,j+1))
//                       / 4.;
//                }
//                else if (j == m.cols() - 1)
//                {
//                    sphereMap(i, j) = (
//                       m(i,j) +    m(i,j-1) +
//                       m(i+1,j) +  m(i+1,j-1))
//                       / 4.;
//                }
//                else
//                {
//                    sphereMap(i, j) = (
//                       m(i,j-1) +   m(i,j) +    m(i,j+1) +
//                       m(i+1,j-1) + m(i+1,j) +  m(i+1,j+1))
//                       / 6.;
//                }
//            }
//            else if (i == m.rows() - 1)
//            {
//
//                if (j == 0)
//                {
//                    sphereMap(i, j) = (
//                       m(i,j) +    m(i,j+1) +
//                       m(i-1,j) +  m(i-1,j+1))
//                       / 4.;
//                }
//                else if (j == m.cols() - 1)
//                {
//                    sphereMap(i, j) = (
//                       m(i,j) +    m(i,j-1) +
//                       m(i-1,j) +  m(i-1,j-1))
//                       / 4.;
//                }
//                else
//                {
//                    sphereMap(i, j) = (
//                       m(i,j-1) +   m(i,j) +    m(i,j+1) +
//                       m(i-1,j-1) + m(i-1,j) +  m(i-1,j+1))
//                       / 6.;
//                }
//            }
//            else
//            {
//                if (j == 0)
//                {
//                    sphereMap(i, j) = (
//                       m(i-1,j) +  m(i-1,j+1) +
//                       m(i,j) +    m(i,j+1) +
//                       m(i+1,j) +  m(i+1,j+1))
//                       / 6.;
//                }
//                else if (j == m.cols() - 1)
//                {
//                    sphereMap(i, j) = (
//                       m(i-1,j-1)+  m(i-1,j) +
//                       m(i,j) +     m(i,j-1) +
//                       m(i+1,j) +   m(i+1,j-1))
//                       / 6.;
//                }
//                else
//                {
//                    sphereMap(i, j) = (
//                       m(i-1,j-1) + m(i-1,j) +  m(i-1,j+1) +
//                       m(i,j-1) +   m(i,j) +    m(i,j+1) +
//                       m(i+1,j-1) + m(i+1,j) +  m(i+1,j+1))
//                       / 9.;
//                }
//            }
        }
    }

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
    if (map(i-5, j-5) < z && i>4 && j>4)
    {
        z = map(i-5, j-5);
        x = i-5;
        y = j-5;
    }

    if (map(i-5, j) <= z && i>4)
    {
        z = map(i-5, j);
        x = i-5;
        y = j;
    }

    if (map(i-5, j+5) <= z && i > 4 && j < map.cols() - 5)
    {
        z = map(i-5, j+5);
        x = i-5;
        y = j+5;
    }

    if (map(i, j-5) <= z && j > 4)
    {
        z = map(i, j-5);
        x = i;
        y = j-5;
    }

    if (map(i, j+5) <= z && j < map.cols() - 5)
    {
        z = map(i, j+5);
        x = i;
        y = j+5;
    }

    if (map(i+5, j-5) <= z && i < map.rows() - 5 && j > 4)
    {
        z = map(i+5, j-5);
        x = i+5;
        y = j-5;
    }

    if (map(i+5, j) <= z && i < map.rows() - 5)
    {
        z = map(i+5, j);
        x = i+5;
        y = j;
    }

    if (map(i+5, j+5) <= z && i < map.rows() - 5 && j < map.cols() - 5)
    {
        z = map(i+5, j+5);
        x = i+5;
        y = j+5;
    }

    ball.x = (float) x;
    ball.y = (float) y;
    ball.z = z;
};

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
    //TODO: create a good map with photoshop (kinda finished)

    Sprite sprite;
    Texture texture;
    texture.loadFromImage(*chromaMap);

    // let's create the graphic image of the ball
    CircleShape pawn(10.f);
    pawn.setFillColor(sf::Color(0, 0, 0, 0));  // pawn is transparent
    pawn.setOutlineThickness(10.f);
    pawn.setOutlineColor(sf::Color::White);

    buildSphereMap(*chromaMap, zMap);

    Ball ball(663., 250.);

    while(window.isOpen())
    {
        Event event;
        while(window.pollEvent(event))
        {
            if(event.type == Event::Closed)
                window.close();

            //TODO: add joystick events to moveBall or moveWorld
        }

        //Apply the gravitation physics
        makeFall(ball, zMap);


        //TODO: add an arrival check, later
        //TODO: adjust frame rate
        texture.update(*chromaMap);
        sprite.setTexture(texture);

        pawn.setPosition(ball.x, ball.y);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();
        sleep(seconds(0.5f));

    }
    return 0;
}
