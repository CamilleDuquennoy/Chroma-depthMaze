
#define _USE_MATH_DEFINES

#ifndef M_PI
 #define M_PI 3.14159
#endif

#include <iostream>
#include <list>
#include <math.h>
#include <string>
#include <thread>
#include "level.h"

using namespace Eigen;
using namespace std;
using namespace sf;

int levelMax = 3;
int levelNumber = 0;
int mode = 2;   /*0 is normal HD screen, 1 is 4K Geo-Cosmos, 2 is worldeye screen, 3 is normal screen in sphere mode*/
float offSet = 0.;

Font font;

void setPawnPosition(CircleShape &pawn, Ball ball, Image referenceMap, Matrix3f rotation)
{
    sf::Vector2i size = (sf::Vector2i) referenceMap.getSize();

    float newX, newY;
    rotateCoord(rotation.inverse(), size, ball.x, ball.y, newX, newY);

    newX = max(0.f, newX - ball.radius);
    newY = max(0.f, newY - ball.radius);

    float x = newX;
    float y = newY;

    if (mode > 1)
    {
        if (mode == 2)
        {
            x = x / size.x * 1280;
            y = y / size.y * 720;
            size = sf::Vector2i(1280, 720);
        }
        sphericalToCartesian(newX, newY, x, y, size);
    }
    pawn.setPosition(x, y + offSet);
}

void addAccelerationToBall(Ball &ball, CircleShape pawn, Level level, float xBall, float yBall)
{
    float scale = 5.;
    if (mode == 1) scale *= 2.;
    if (mode == 2) scale /= 2.;

    float newX, newY, newX2, newY2;
    rotateCoord(level.rotation, (sf::Vector2i) level.chromaMap.getSize(), pawn.getPosition().x + xBall, pawn.getPosition().y + yBall - offSet, newX2, newY2);
    rotateCoord(level.rotation, (sf::Vector2i) level.chromaMap.getSize(), pawn.getPosition().x, pawn.getPosition().y - offSet, newX, newY);

    ball.a = scale * Eigen::Vector3f(newX2 - newX, newY2 - newY, 0.f).normalized() * Eigen::Vector2f(xBall, yBall).norm();
}

void checkControllerState(Window &window, Ball &ball, CircleShape pawn, Level &level)
{
    Joystick::update();

    if (Joystick::isConnected(0))
    {
        float xBall = Joystick::getAxisPosition(0, Joystick::X) / 100.;
        float yBall = Joystick::getAxisPosition(0, Joystick::Y) / 100.;
        float xWorld = Joystick::getAxisPosition(0, Joystick::Z) / 100.;
        float yWorld = Joystick::getAxisPosition(0, Joystick::R) / 100.;

//        float xBall = 0.;
//        float yBall = 0.;
//        if (Keyboard::isKeyPressed(Keyboard::Left)) xBall = -1.;
//        if (Keyboard::isKeyPressed(Keyboard::Right)) xBall = 1.;
//        if (Keyboard::isKeyPressed(Keyboard::Up)) yBall = -1.;
//        if (Keyboard::isKeyPressed(Keyboard::Down)) yBall = 1.;
        addAccelerationToBall(ball, pawn, level, xBall, yBall);

        if (xWorld*xWorld + yWorld*yWorld > 0.01)
        {
            float thetaBall = (pawn.getPosition().x + pawn.getRadius()) / level.referenceMap.getSize().x * 2. * M_PI - M_PI;
            float phiBall = (pawn.getPosition().y + pawn.getRadius() - offSet) / level.referenceMap.getSize().y * M_PI - M_PI / 2.;
//
//            Eigen::Vector3f pawnPos = rotation.transpose() * Eigen::Vector3f(sin(phiBall)*cos(thetaBall), sin(phiBall)*sin(thetaBall), cos(phiBall));

            Matrix3f ballRotOffset = angleToRotation(thetaBall, 0.);

            level.rotation = angleToRotation(-xWorld, 0.) * angleToRotation(0., -yWorld/2.) * ballRotOffset.inverse() * level.rotation;

//            cout << level.rotation << endl;
            level.rotateWorld();
            addAccelerationToBall(ball, pawn, level, -xWorld * 20., -yWorld * 20.);
        }
    }
}

void manageEvents(Event event, Window &window, Ball &ball, CircleShape &pawn, Clock &clock, Level &level)
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
                    level.gridOption = !level.gridOption;
                    level.update();
                    break;

                case 1: /*Cross*/
                    cout << "Centering the map" << endl;
                    level.centerMap(ball);
                    break;

                case 2: /*Circle*/
                    level.textureOption = !level.textureOption;
                    level.update();
                    break;

                case 3: /*Triangle*/
                    level.shadeOption = !level.shadeOption;
                    level.update();
                    break;

                case 4: /*L1*/
                    cout << "ball : " << ball.x << "; " << ball.y << " at " << ball.z << endl << "pawn : " << pawn.getPosition().x << "; " << pawn.getPosition().y << endl;
                    break;

                case 5: /*R1*/
                    level.rotation = Matrix3f::Identity();
                    level.update();
                    break;

                case 12: /*Playstation*/
                    if (mode == 0)
                    {
                        mode = 3;
                        level.rotateWorld();
                    }
                    else if (mode == 3)
                    {
                        mode = 0;
                        level.switchToSphericalDisplay();
                    }
                }
            break;

        case Event::JoystickMoved:
            cout << "Joystick " << event.joystickMove.joystickId << " moved in " << event.joystickMove.axis << " direction" << endl
            << "At " << event.joystickMove.position << " position" << endl;
            break;

        case Event::KeyPressed:
            cout << "Key " << event.key.code << " pressed" << endl;

            if (event.key.code == Keyboard::Enter)
                level.centerMap(ball);

            if (event.key.code == Keyboard::Escape)
                window.create(VideoMode(window.getSize().x, window.getSize().y), "Chroma-depth maze", Style::Default);

            if (event.key.code == Keyboard::F)
                window.create(VideoMode(window.getSize().x, window.getSize().y), "Chroma-depth maze", Style::Fullscreen);
            break;

        case Event::MouseButtonPressed:
            cout << "Mouse button " << event.mouseButton.button << " pressed" << endl;
            break;

        default:
            break;
        }
}

bool isArrived(Ball ball, Eigen::Vector2f goal)
{
    float distanceToGoal = pow(ball.x - goal(0), 2) + pow(ball.y - goal(1), 2);
    float distMin = 100.;
    if (mode == 1) distMin *= 4;
    if (mode == 2) distMin /= 4;
    return (distanceToGoal < distMin);
}

void levelComplete(RenderWindow &window, Level* &level, int &levelNumber, Ball &ball)
{
    cout << "Well done, you've finished level " << levelNumber << "!" << endl;

    int textSize = 30;
    if (mode == 1) textSize *= 2;
    if (levelNumber < levelMax)
    {
        Text text("   Well done, you've finished the level! \nClick on any button to go to the next one", font, textSize);
        text.setFillColor(Color::White);
        //TODO: center text
        sf::FloatRect textRect = text.getLocalBounds();
        text.setOrigin(textRect.left + textRect.width/2.0f,
                       textRect.top  + textRect.height/2.0f);
        text.setPosition(window.getSize().x / 2., window.getSize().y / 2.);

        while(window.isOpen())
        {
            Event event;
            while (window.pollEvent(event))
            {
                if (event.type == Event::JoystickButtonPressed || event.type == Event::Closed) window.close();
            }
            window.clear();
            window.draw(text);
            window.display();
        }

        levelNumber++;
        level = new Level(levelNumber, mode);   //TODO: have it loaded while the message is displayed
        ball = *new Ball();
        ball.x = level->chromaMap.getSize().x / 3;  //TODO: adapt to the map
        ball.y = level->chromaMap.getSize().y / 2;

        window.create(VideoMode(level->chromaMap.getSize().x, level->chromaMap.getSize().y), "Chroma-depth maze", Style::Default);
    }
    else
    {
        Text text("   Well done, you've finished all the levels! \nClick on any button to go to exit the game", font, textSize);
        text.setFillColor(Color::White);
        //TODO: center text
        sf::FloatRect textRect = text.getLocalBounds();
        text.setOrigin(textRect.left + textRect.width/2.0f,
                       textRect.top  + textRect.height/2.0f);
        text.setPosition(window.getSize().x / 2., window.getSize().y / 2.);

        while(window.isOpen())
        {
            Event event;
            while (window.pollEvent(event))
            {
                if (event.type == Event::JoystickButtonPressed || event.type == Event::Closed) window.close();
            }
            window.clear();
            window.draw(text);
            window.display();
        }
    }
}

int main( int argc, char * argv[] )
{
    Level* level = new Level(levelNumber, mode);
    cout << "Goal: " << level->goal(0) << "; " << level->goal(1) << endl;
    sf::Vector2u size = level->chromaMap.getSize();
    sf::Vector2u refSize = level->referenceMap.getSize();
    RenderWindow window(VideoMode(size.x, size.y), "Chroma-depth maze", Style::Default);
    window.setJoystickThreshold(20);    //Need to adapt to the game controller

    font.loadFromFile("ARIALN.TTF");

    //TODO: create a good map with photoshop (finished for the dev part only)

    Sprite sprite;
    Texture texture;
    texture.loadFromImage(level->chromaMap);
    texture.setSmooth(true);
    sprite.setTexture(texture);

    /* let's create the graphic image of the ball*/
    CircleShape pawn(30.f);
    pawn.setFillColor(sf::Color::Black);
    pawn.setOutlineColor(sf::Color(255, 255, 255, 150));
    pawn.setOutlineThickness(2.f);

    if (mode == 1)
    {
        offSet = 120.;
        pawn.setRadius(2*pawn.getRadius());
        pawn.setOutlineThickness(2*pawn.getOutlineThickness());
    }
    if (mode == 2)
    {
        offSet = 0.;
        pawn.setRadius(2/pawn.getRadius());
        pawn.setOutlineThickness(2/pawn.getOutlineThickness());
    }
    sprite.setPosition(0., offSet);

    cout << "Holes' list :" << endl;
    for (Vector4i hole : level->holesList)
        cout << hole(0) << "; " << hole(1) << "; " << hole(2) << "; " << hole(3) << endl << endl;

    level->saveZMap("z_maps/zMap_grey.png");


    Ball ball(refSize.x / 2., refSize.y / 2., Eigen::Vector3f(0., 0., 0.));
//    Ball ball(1300, 693, Eigen::Vector3f(20., 0., 0.));
    ball.z = level->zMap(ball.x, ball.y);
    cout << ball.z << "!" << endl;
    ball.radius = 10. + ball.z / 20.;
    ball.to4K(mode);

    Clock clock;
    Time elapsedTime;

    while(window.isOpen())
    {
        Event event;
        while(window.pollEvent(event))
        {
            manageEvents(event, window, ball, pawn, clock, *level);
            if (!window.isOpen()) return 0;
        }

        /*Apply the gravitation physics*/
        elapsedTime = clock.restart();
        checkControllerState(window, ball, pawn, *level);
        level->makeBallFall(ball, elapsedTime);

        texture.update(level->chromaMap);

        pawn.setRadius(ball.radius);
        setPawnPosition(pawn, ball, level->chromaMap, level->rotation);

        if (isArrived(ball, level->goal)) levelComplete(window, level, levelNumber, ball);

        window.clear();
        window.draw(sprite);
        window.draw(pawn);
        window.display();

    }
    return 0;
}
