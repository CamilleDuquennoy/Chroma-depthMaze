
#define _USE_MATH_DEFINES

#ifndef M_PI
 #define M_PI 3.14159
#endif

#include <iostream>
#include <list>
#include <math.h>
#include <string>
#include <thread>
#include "header.h"

using namespace Eigen;
using namespace std;
using namespace sf;

bool is4K = true;
bool gridOption = true;
bool textureOption = true;
bool shadeOption = true;

string mapPath;
string zMapPath;

void loadMap(Image &map)
{
    mapPath = "maps/map";

    if (is4K) mapPath += "_4K";

    if (gridOption || textureOption || shadeOption) mapPath += "_";
    if (gridOption) mapPath += "G";
    if (textureOption) mapPath += "T";
    if (shadeOption) mapPath += "S";

    mapPath += ".png";


    if (!map.loadFromFile(mapPath))
    {
        cout << "Couldn't open the map file" << endl;
    }
    else
    cout << "File " << mapPath << " opened" << endl;

}

void buildSphereMap(MatrixXf &sphereMap, Matrix<Eigen::Vector3f, Dynamic, Dynamic> &normalMap, list<Eigen::Vector4i> &holesList)
{
    Image zImage;

    zMapPath = "z_maps/z_map_hole";
    if (is4K) zMapPath += "_4K";
    zMapPath += ".png";

    if (!zImage.loadFromFile(zMapPath))
    {
        cout << "Couldn't open the zmap file" << endl;
    }
    else
    {
        float zMax = 100.;
        if (is4K) zMax *= 4.;
        cout << "Started loading zMap" << endl;

        Vector2u size = zImage.getSize();
        sphereMap.resize(size.x, size.y);
        normalMap.resize(size.x, size.y);

        cout << "Resized zmap to " << size.x << "; " << size.y << endl;
        for (unsigned int i = 0; i < size.x; i++)
        {
            for (unsigned int j = 0; j < size.y; j++)
            {
                float z = -INFINITY;
                Color color = zImage.getPixel(i, j);
                float R = (float) color.r / 255.;
                float G = (float) color.g / 255.;
                float B = (float) color.b / 255.;

                /* Sets z depending on the color*/
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

                /* Let's create the holes list */
                if (color.a < 250)
                {
                    /* It's a hole */
                    int a = color.a;
                    z = -2.*zMax;

                    Eigen::Vector4i newHole(i, j, -1, -1);
                    for (Eigen::Vector4i hole : holesList)
                    {
                        int k = hole(0);
                        int l = hole(1);

                        if (abs(zImage.getPixel(k, l).a - a) < 50 && (pow(k-i, 2) + pow(l-j, 2) > 100))
                        {   /* The two first int are the start of the hole, the next two the arrival point */
                            newHole(2) = k;
                            newHole(3) = l;
                        }
                    }
                    holesList.push_back(newHole);
                    if (newHole(2) !=-1)
                        holesList.push_back(Eigen::Vector4i(newHole(2) , newHole(3), i, j));
                }

                sphereMap(i, j) = z;
            }
        }
        holesList.remove_if([](Eigen::Vector4i h){ return h(2) == -1;});
    }

    for (int i = 0; i < normalMap.rows(); i++)
    {
        for (int j = 0; j < normalMap.cols(); j++)
        {
            normalMap(i, j) = Eigen::Vector3f(0., 0., 1.);
        }
    }

    int margin = 25;
    if (is4K) margin *= 2;
    for (int i = margin; i < normalMap.rows() - margin; i++)
    {
        for (int j = margin; j < normalMap.cols() - margin; j++)
        {
            Eigen::Vector3f n;

            float iPlus = sphereMap(i+margin, j);
            float iMinus = sphereMap(i-margin, j);
            float jPlus = sphereMap(i, j+margin);
            float jMinus = sphereMap(i, j-margin);
            float zNormal = sphereMap(i, j);

            float wallLimit = 50.;
            if (is4K) wallLimit *= 4;
            if (abs(iPlus - zNormal) > wallLimit)
                iPlus = zNormal;
            if (abs(iMinus - zNormal) > wallLimit)
                iMinus = zNormal;
            if (abs(jPlus - zNormal) > wallLimit)
                jPlus = zNormal;
            if (abs(jMinus - zNormal) > wallLimit)
                jMinus = zNormal;

            n = Eigen::Vector3f(2*margin, 0., iPlus - iMinus)
                    .cross(Eigen::Vector3f(0., 2*margin, jPlus - jMinus));
            n.normalize();
            normalMap(i, j) = n;
        }
    }
    cout << "Finished loading zMap" << endl << endl;
};

void makeBallFall(Ball &ball, MatrixXf zMap, const Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap, list<Eigen::Vector4i> holesList, const Time elapsedTime)
{
    Eigen::Vector3f gravitation(0., 0., -1.);
    if (is4K) gravitation *= 4.;

    Eigen::Vector3f beforePos(ball.x , ball.y, ball.z);
    Eigen::Vector3f v = ball.v;

    Eigen::Vector3f normal = normalMap((int) ball.x, (int) ball.y);
    v += 10. * elapsedTime.asSeconds() * (gravitation + normal);

    float distance = (elapsedTime.asSeconds() * v).norm();
    Eigen::Vector3f direction = v.normalized();

    Eigen::Vector3f pos = beforePos;

    float radius = ball.radius;

    /* Collisions */
    for (float t = 0.; t < distance; t += 0.1)
    {
        pos = beforePos + t * direction;
        if (pos(0) > zMap.rows() - 1) pos(0) -= zMap.rows();
        if (pos(0) < 0.) pos(0) += zMap.rows();
        pos(1) = max(min(zMap.cols()-1.f, pos(1)), 0.f);

        if (zMap((int) pos(0), (int) pos(1)) - pos(2) > 0.5)
        {
            radius = 10. + zMap((int) pos(0), (int) pos(1)) / 20.;
            pos += radius * direction;
            if (pos(0) > zMap.rows() - 1) pos(0) -= zMap.rows();
            if (pos(0) < 0.) pos(0) += zMap.rows();
            pos(1) = max(min(zMap.cols()-1.f, pos(1)), 0.f);

            if (zMap((int) pos(0), (int) pos(1)) - pos(2) > ball.realRadius)    /* There's a wall */
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

    /* Is the ball deep? */
    if (zMap((int) pos(0), (int) pos(1)) < -195.)
    {
        for(Eigen::Vector4i hole : holesList)
        {
            if (pow(pos(0) - hole(0), 2) + pow(pos(1) - hole(1), 2) <= 100.)   /* The ball is close to the hole */
            {
                v = Eigen::Vector3f(-v(0), v(1), -v(2));
                pos = Eigen::Vector3f(hole(2), hole(3), zMap(hole(2), hole(3))) + 20.*v.normalized();
            }
        }
    }

    ball.v = v;
    ball.x = pos(0);
    ball.y = pos(1);
    ball.z = zMap((int) ball.x, (int) ball.y);
    ball.radius = 10. + ball.z / 20.;
}

void rotateWorld(Image &chromaMap, Image referenceMap, Matrix3f rotation)
{
    sf::Vector2i size = (sf::Vector2i) chromaMap.getSize();

    for (int i = 0; i < size.x; i++)
    {
        for (int j = 0; j < size.y; j++)
        {
            float theta = (float) i / size.x * 2. * M_PI - M_PI;
            float phi = (float) j / size.y * M_PI;

            Eigen::Vector3f newCartesianPos = rotation * Eigen::Vector3f(sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi));

            float newPhi = acos(newCartesianPos(2));
            float newTheta = atan2(newCartesianPos(1), newCartesianPos(0)) + M_PI;


            int newI = min((double) size.x - 1, (float) newTheta * size.x / (2*M_PI));
            int newJ = min((double) size.y - 1, (float) newPhi * size.y / (M_PI));

            chromaMap.setPixel(i, j, referenceMap.getPixel(newI, newJ));
        }
    }
}

void setPawnPosition(CircleShape &pawn, Ball ball, Image chromaMap, Matrix3f rotation)
{
    sf::Vector2i size = (sf::Vector2i) chromaMap.getSize();

    float theta = (ball.x - ball.radius) / size.x * 2. * M_PI - M_PI;
    float phi = (ball.y - ball.radius) / size.y * M_PI;

    Eigen::Vector3f pawnPos = rotation.transpose() * Eigen::Vector3f(sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi));

    float newPhi = acos(pawnPos(2));
    float newTheta = atan2(pawnPos(1), pawnPos(0)) + M_PI;

    float newX = min((double) size.x - 1, (float) newTheta * size.x / (2*M_PI));
    float newY = min((double) size.y - 1, (float) newPhi * size.y / (M_PI));

    pawn.setPosition(newX, newY);
}

void centerMap(CircleShape &pawn, Image &chromaMap, const Image referenceMap, Matrix3f &rotation)
{
    sf::Vector2i size = (sf::Vector2i) chromaMap.getSize();

    /* We use the opposite rotation */
    float dX = (float) pawn.getPosition().x + pawn.getRadius()- size.x / 2.;
    float dY = (float) pawn.getPosition().y + pawn.getRadius() - size.y / 2.;
    float dTheta = dX / (float) size.x * 2. * M_PI;
    float dPhi = dY / (float) size.y * M_PI;

    Matrix3f thetaRot;
    thetaRot <<     cos(dTheta),    -sin(dTheta), 0.,
                    sin(dTheta),     cos(dTheta), 0.,
                    0.,              0.,          1.;

    Matrix3f phiRot;
    phiRot <<       cos(dPhi),      0.,     sin(dPhi),
                    0.,             1.,     0.,
                    -sin(dPhi),     0.,     cos(dPhi);

    rotation = thetaRot * phiRot * rotation;

    rotateWorld(chromaMap, referenceMap, rotation);

    pawn.setPosition(size.x - 2, size.y - 2);

    cout << "Finished centering the map" << endl;
}

void saveZMap(MatrixXf zMap, const string fileName)
{
    Image img;
    img.create(zMap.rows(), zMap.cols());

    for(int i = 0; i < zMap.rows(); i++)
    {
        for (int j = 0; j < zMap.cols(); j++)
        {
            float z;
            if (is4K) z = (200. + zMap(i, j) / 4.) / 300. * 255.;
            else z = (200. + zMap(i, j)) / 300. * 255.;
            img.setPixel(i, j, Color(z, z, z));
        }
    }

    if (!img.saveToFile(fileName)) cout << "Couldn't save the zMap file" << endl;
    else cout << "Saved the zMap file" << endl;
}

void manageEvents(Event event, Window &window, Ball &ball, CircleShape &pawn, Clock &clock, Image &chromaMap, Image &referenceMap, Matrix3f &rotation)
{
    switch (event.type)
        {
        case Event::Closed:
            window.close();
            break;

        case Event::LostFocus:
            {
                Event nextEvent;
                bool noFocus = true;

                while(window.waitEvent(nextEvent) && noFocus)
                {
                    if (event.type != Event::GainedFocus)
                        noFocus = false;
                        clock.restart();
                }
            }
            break;

        case Event::JoystickConnected:
            cout << "Joystick " << event.joystickConnect.joystickId << " connected" << endl;
            break;

        case Event::JoystickDisconnected:
            cout << "Joystick " << event.joystickConnect.joystickId << " disconnected" << endl;
            break;

        case Event::JoystickButtonPressed:
            cout << "Joystick button " << event.joystickButton.button << " pressed" << endl;

            switch (event.joystickButton.button)
                {
                case 0: /*Square*/
                    //Grid option
                    gridOption = !gridOption;
                    loadMap(referenceMap);
                    rotateWorld(chromaMap, referenceMap, rotation);
                    break;

                case 1: /*Cross*/
                    cout << "Centering the map" << endl;
                    centerMap(pawn, chromaMap, referenceMap, rotation);
                    break;

                case 2: /*Circle*/
                    textureOption = !textureOption;
                    loadMap(referenceMap);
                    rotateWorld(chromaMap, referenceMap, rotation);
                    break;

                case 3: /*Triangle*/
                    shadeOption = !shadeOption;
                    loadMap(referenceMap);
                    rotateWorld(chromaMap, referenceMap, rotation);
                    break;
                }
            break;

        case Event::JoystickMoved:
            cout << "Joystick " << event.joystickMove.joystickId << " moved in " << event.joystickMove.axis << " direction" << endl
            << "At " << event.joystickMove.position << " position" << endl;
            break;

        case Event::KeyPressed:
            cout << "Key " << event.key.code << " pressed" << endl;

            if (event.key.code == Keyboard::Enter) centerMap(pawn, chromaMap, referenceMap, rotation);
            if (event.key.code == Keyboard::Escape) window.create(VideoMode(window.getSize().x, window.getSize().y), "Chroma-depth maze", Style::Default);
            if (event.key.code == Keyboard::F) window.create(VideoMode(window.getSize().x, window.getSize().y), "Chroma-depth maze", Style::Fullscreen);
            break;

        case Event::MouseButtonPressed:
            cout << "Mouse button " << event.mouseButton.button << " pressed" << endl;
            break;

        default:
            break;
        }
}

int main( int argc, char * argv[] )
{
    Image* chromaMap = new Image();
    loadMap(*chromaMap);
    Image* referenceMap = new Image();
    loadMap(*referenceMap);
    RenderWindow window(VideoMode(chromaMap->getSize().x, chromaMap->getSize().y), "Chroma-depth maze", Style::Fullscreen); //Need to config size for Geo-cosmos
    window.setJoystickThreshold(90);    //Need to adapt to the game controller

    MatrixXf zMap;
    Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap;
    list<Eigen::Vector4i> holesList;
    Matrix3f rotation = Matrix3f::Identity();
    //TODO: create a good map with photoshop (finished for the dev part only)

    Sprite sprite;
    Texture texture;
    texture.loadFromImage(*chromaMap);
    sprite.setTexture(texture);

    /* let's create the graphic image of the ball*/
    CircleShape pawn(30.f);
    pawn.setFillColor(sf::Color::Black);
    pawn.setOutlineColor(sf::Color(255, 255, 255, 150));
    pawn.setOutlineThickness(2.f);

    if (is4K)
    {
        pawn.setRadius(2*pawn.getRadius());
        pawn.setOutlineThickness(2*pawn.getOutlineThickness());
    }

    buildSphereMap(zMap, normalMap, holesList);

    cout << "Holes' list :" << endl;
    for (Vector4i hole : holesList)
        cout << hole(0) << "; " << hole(1) << "; " << hole(2) << "; " << hole(3) << endl << endl;

    saveZMap(zMap, "zMap_grey.png");


//    Ball ball(2*960, 2*580, Eigen::Vector3f(0., 2*10., 0.));
    Ball ball(1400, 693, Eigen::Vector3f(20., 0., 0.));
    ball.z = zMap(ball.x, ball.y);
    ball.radius = 10. + ball.z / 20.;
    ball.to4K(is4K);

    Clock clock;
    Time elapsedTime;

    while(window.isOpen())
    {
        Event event;
        while(window.pollEvent(event))
        {
            manageEvents(event, window, ball, pawn, clock, *chromaMap, *referenceMap, rotation);
            if (!window.isOpen()) return 0;
        }

        /*Apply the gravitation physics*/
        elapsedTime = clock.restart();
        makeBallFall(ball, zMap, normalMap, holesList, elapsedTime);

        //TODO: add an arrival check, later
        texture.update(*chromaMap);

        pawn.setRadius(ball.radius);
        setPawnPosition(pawn, ball, *chromaMap, rotation);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();

    }
    return 0;
}
