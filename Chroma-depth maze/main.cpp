
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
    float zMax = 10.;
    cout << "Started loading zMap" << endl;

    Vector2u size = pixelMap.getSize();
    sphereMap.resize(size.x, size.y);   //Might be inverted

    cout << "Resized zmap to " << size.x << "; " << size.y << endl;
    for (unsigned int i = 0; i < size.x; i++)
    {
        for (unsigned int j = 0; j < size.y; j++)
        {
            float z = -INFINITY;
//            cout << "Getting the color of pixel " << i << "; " << j << endl;
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
                    };
                };
            };
            sphereMap(i, j)  = z;
        };
    };


    cout << "Finished loading zMap" << endl;
};

void makeFall(Ball* ball, MatrixXf map)
{
    /*For the moment it doesn't include any speed, gravitation force or anything,
    it just goes down at eache frame*/
    //TODO: implement a real gravity

    //For now we assume pixel.x & pixel.y are equal to position in table map
    int i = (int) ball->x;
    int j = (int) ball->y;
    float z = map(i, j);
    int x = i;
    int y = j;

    // Get neighbour with lowest z
    if (map(i-1, j-1) < z && i>0 && j >0)
    {
        z = map(i-1, j-1);
        x = i-1;
        y = j-1;
    }

    if (map(i-1, j) < z && i >0)
    {
        z = map(i-1, j);
        x = i-1;
        y = j;
    }

    if (map(i-1, j+1) < z && i > 0 && j < map.cols() - 1)
    {
        z = map(i-1, j+1);
        x = i-1;
        y = j+1;
    }

    if (map(i, j-1) < z && j > 0)
    {
        z = map(i, j-1);
        x = i;
        y = j-1;
    }

    if (map(i, j+1) < z && j < map.cols() - 1)
    {
        z = map(i, j+1);
        x = i;
        y = j+1;
    }

    if (map(i+1, j-1) < z && i < map.rows() - 1 && j > 0)
    {
        z = map(i+1, j-1);
        x = i+1;
        y = j-1;
    }

    if (map(i+1, j) < z && i < map.rows() - 1)
    {
        z = map(i+1, j);
        x = i+1;
        y = j;
    }

    if (map(i+1, j+1) < z && i < map.rows() - 1 && j < map.cols() - 1)
    {
        z = map(i+1, j+1);
        x = i+1;
        y = j+1;
    }

    ball->x = (float) x;
    ball->y = (float) y;
    ball->z = z;
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

void display(Arguments arg)
{
    RenderWindow* window = arg.window;
    Image* image = arg.image;
    Ball* ball = arg.ball;


    //TODO: display map and ball on screen
    wrapMapToSphere(*image);  //Only for Geo-cosmos

    cout << "Ok2" << endl;
    while(window->isOpen())
    {
    }
};

int main( int argc, char * argv[] )
{
    RenderWindow window(VideoMode(1366, 768), "Chroma-depth maze"); //Need to config size for Geo-cosmos
    Image* chromaMap = new Image();
    loadMap(MAP_PATH, *chromaMap);

    MatrixXf zMap;
    //TODO: create a good map with photoshop

    Sprite sprite;
    Texture texture;
    texture.loadFromImage(*chromaMap);

    // let's create the graphic image of the ball
    CircleShape pawn(10.f);
    pawn.setFillColor(sf::Color(0, 0, 0, 0));  // pawn is transparent
    pawn.setOutlineThickness(10.f);
    pawn.setOutlineColor(sf::Color::White);

    cout << "OK" << endl;
    buildSphereMap(*chromaMap, zMap);

    Ball* ball = new Ball(663., 200);

    //TODO: draw on a separate thread, and add arguments to display
//    Thread displayWindow(display, Arguments(&window, chromaMap, ball));
//
//    displayWindow.launch();
//    cout << "Ok1" << endl;


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

        pawn.setPosition(ball->x, ball->y);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();

    }
    return 0;
}
