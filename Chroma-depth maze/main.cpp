
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

int levelNumber = 0;
bool is4K = false;

Font font;

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

void checkControllerState(Window &window, Ball &ball, CircleShape &pawn, Level &level)
{
    Joystick::update();

    if (Joystick::hasAxis(0, Joystick::X))
    {
        float xBall = Joystick::getAxisPosition(0, Joystick::X);
        float yBall = Joystick::getAxisPosition(0, Joystick::Y);
        float xWorld = Joystick::getAxisPosition(0, Joystick::Z);
        float yWorld = Joystick::getAxisPosition(0, Joystick::R);

        //TODO: Need to take into account the rotation of the world
        float scale = 50.;
        if (is4K) scale /= 2.;

        ball.a = Eigen::Vector3f(xBall / scale, yBall / scale, 0.);

        if (xWorld*xWorld + yWorld*yWorld > 100. && false)
        {
            float dTheta = xWorld / 100. * 2. * M_PI;

            float dPhi = -yWorld / 100. * M_PI;

            Matrix3f thetaRot;
            thetaRot <<     cos(dTheta),    -sin(dTheta), 0.,
                            sin(dTheta),     cos(dTheta), 0.,
                            0.,              0.,          1.;

            Matrix3f phiRot;
            phiRot <<       cos(dPhi),      0.,     sin(dPhi),
                            0.,             1.,     0.,
                            -sin(dPhi),     0.,     cos(dPhi);


//            float thetaBall = ball.x / referenceMap.getSize().x * M_PI - M_PI;
//            float phiBall = ball.y / referenceMap.getSize().y * M_PI;
//
//            Eigen::Vector3f pawnPos = rotation.transpose() * Eigen::Vector3f(sin(phiBall)*cos(thetaBall), sin(phiBall)*sin(thetaBall), cos(phiBall));

            float ballOffSet = -pawn.getPosition().x / level.referenceMap.getSize().x * M_PI - M_PI;
//            acos(pawnPos(2));
            Matrix3f phiRotBallAdapt;
            phiRotBallAdapt <<  cos(ballOffSet),    -sin(ballOffSet), 0.,
                                sin(ballOffSet),     cos(ballOffSet), 0.,
                                0.,                  0.,              1.;
            level.rotation = phiRotBallAdapt * thetaRot * phiRot * phiRotBallAdapt.transpose() * level.rotation;

            cout << level.rotation << endl;
            level.rotateWorld();
            ball.a += Eigen::Vector3f(-xWorld / scale, -yWorld / scale, 0.);
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
                    //Grid option
                    level.gridOption = !level.gridOption;
                    level.update();
                    break;

                case 1: /*Cross*/
                    cout << "Centering the map" << endl;
                    level.centerMap(pawn);
                    break;

                case 2: /*Circle*/
                    level.textureOption = !level.textureOption;
                    level.update();
                    break;

                case 3: /*Triangle*/
                    level.shadeOption = !level.shadeOption;
                    level.update();
                    break;

                case 4:
                    cout << "ball : " << ball.x << "; " << ball.y << " at " << ball.z << endl << "pawn : " << pawn.getPosition().x << "; " << pawn.getPosition().y << endl;
                }
            break;

        case Event::JoystickMoved:
            cout << "Joystick " << event.joystickMove.joystickId << " moved in " << event.joystickMove.axis << " direction" << endl
            << "At " << event.joystickMove.position << " position" << endl;
            break;

        case Event::KeyPressed:
            cout << "Key " << event.key.code << " pressed" << endl;

            if (event.key.code == Keyboard::Enter)
                level.centerMap(pawn);

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
    if (is4K) distMin *= 4;
    return (distanceToGoal < distMin);
}

void levelComplete(RenderWindow &window, Level* &level, int &levelNumber, Ball &ball)
{
    //For now it's just some basic message, need to create a pop-up or something
    cout << "Well done, you finished the " << levelNumber << " level!" << endl;

    int textSize = 30;
    if (is4K) textSize *= 2;
    Text text("   Well done, you've finished the level! \nClick on any button to go to the next one", font, textSize);
    text.setColor(Color::White);
    //center text
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width/2.0f,
                   textRect.top  + textRect.height/2.0f);
    text.setPosition(window.getSize().x / 2., window.getSize().y / 2.);

    while(window.isOpen())
    {
        Event event;
        while (window.pollEvent(event))
        {
            if (event.type == Event::JoystickButtonPressed) window.close();
        }
        window.clear();
        window.draw(text);
        window.display();
    }

    levelNumber++;
    level = new Level(levelNumber, is4K);
    ball.x = level->chromaMap.getSize().x / 2;  //Will need to add the margin
    ball.y = level->chromaMap.getSize().y / 2;

    window.create(VideoMode(level->chromaMap.getSize().x, level->chromaMap.getSize().y), "Chroma-depth maze", Style::Fullscreen);
}

int main( int argc, char * argv[] )
{
    Level* level = new Level(levelNumber, is4K);
    RenderWindow window(VideoMode(level->chromaMap.getSize().x, level->chromaMap.getSize().y), "Chroma-depth maze", Style::Fullscreen);
    window.setJoystickThreshold(50);    //Need to adapt to the game controller

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

    if (is4K)
    {
        pawn.setRadius(2*pawn.getRadius());
        pawn.setOutlineThickness(2*pawn.getOutlineThickness());
    }

    cout << "Holes' list :" << endl;
    for (Vector4i hole : level->holesList)
        cout << hole(0) << "; " << hole(1) << "; " << hole(2) << "; " << hole(3) << endl << endl;

    level->saveZMap("zMap_grey.png");


//    Ball ball(660, 480, Eigen::Vector3f(0., 0., 0.));
    Ball ball(1300, 693, Eigen::Vector3f(20., 0., 0.));
    ball.z = level->zMap(ball.x, ball.y);
    ball.radius = 10. + ball.z / 20.;
    ball.to4K(is4K);

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

        //TODO: add an arrival check, later
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
