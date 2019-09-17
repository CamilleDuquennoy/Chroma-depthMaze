
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
using namespace sf;     /* SFML */

int levelMax = 3;       /* Total number of levels */
int levelNumber = 0;    /* Current level number */
int mode = 3;           /* 0 is normal HD screen, 1 is 4K Geo-Cosmos, 2 is worldeye screen, 3 is normal screen in sphere mode */
float offSet = 0.;      /* Vertical offset for the Geo-Cosmos */

Font font;              /* Font of the displayed messages */

/* Sets the pawn's position on the displayed image, by rotating the ball's position on the referenceMap with the rotation matrix of the level */
void setPawnPosition(CircleShape &pawn, Ball &ball, Image referenceMap, Matrix3f rotation)
{
    sf::Vector2i size = (sf::Vector2i) referenceMap.getSize();

    /* Rotationg the ball's position */
    float newX, newY;
    rotateCoord(rotation.inverse(), size, ball.x, ball.y, newX, newY);

    newX = max(0.f, newX - ball.radius);
    newY = max(0.f, newY - ball.radius);

    float x = newX;
    float y = newY;

    if (mode > 1)   // Sphere mode
    {
        if (newX <= size.x/4 || newX >= 3*size.x/4) // The ball is on the hidden part of the map
            ball.visible = false;
        else
            ball.visible = true;

        /* Converting the position of the ball in spherical mode */
        sphericalToCartesian(newX, newY, x, y, size);
        if (mode == 2)
        {
            x = x / size.x * 640;
            y = y / size.y * 480;
        }
    }

    else ball.visible = true;

    pawn.setPosition(x, y + offSet);
}

/* Adds an acceleration to the ball, in the direction of the vector(xBall, yBall), pawn being the displayed image of the ball, on the current level */
void addAccelerationToBall(Ball &ball, CircleShape pawn, Level level, float xBall, float yBall)
{
    float scale = 5.;
    if (mode == 1) scale *= 2.; // 4K mode
    if (mode == 2) scale /= 2.; // Worldeye mode

    float newX, newY, newX2, newY2;
    /* Computing the equivalent direction of the vector(xBall, yBall), once rotated back to match the referenceMap  */
    rotateCoord(level.rotation, (sf::Vector2i) level.chromaMap.getSize(), pawn.getPosition().x + xBall, pawn.getPosition().y + yBall - offSet, newX2, newY2);
    rotateCoord(level.rotation, (sf::Vector2i) level.chromaMap.getSize(), pawn.getPosition().x, pawn.getPosition().y - offSet, newX, newY);

    ball.a = scale * Eigen::Vector3f(newX2 - newX, newY2 - newY, 0.f).normalized() * Eigen::Vector2f(xBall, yBall).norm();
}

/* Checks the controller's joysticks states, and acts accordingly to add acceleration to the ball or rotate the world*/
void checkControllerState(Window &window, Ball &ball, CircleShape pawn, Level &level)
{
    Joystick::update();

    if (Joystick::isConnected(0))
    {
        /* Left Joystick on a Playstation controller */
        float xBall = Joystick::getAxisPosition(0, Joystick::X) / 100.;
        float yBall = Joystick::getAxisPosition(0, Joystick::Y) / 100.;
        /* Right Joystick on a Playstation controller */
        float xWorld = Joystick::getAxisPosition(0, Joystick::Z) / 100.;
        float yWorld = Joystick::getAxisPosition(0, Joystick::R) / 100.;

        /* Keyboard version to move the ball */
//        float xBall = 0.;
//        float yBall = 0.;
//        if (Keyboard::isKeyPressed(Keyboard::Left)) xBall = -1.;
//        if (Keyboard::isKeyPressed(Keyboard::Right)) xBall = 1.;
//        if (Keyboard::isKeyPressed(Keyboard::Up)) yBall = -1.;
//        if (Keyboard::isKeyPressed(Keyboard::Down)) yBall = 1.;

        /* Moving the ball in the left joystick's direction */
        addAccelerationToBall(ball, pawn, level, xBall, yBall);

        /* Moving the world in the right joystick's direction if the joystick has been pushed (not working yet) */
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

/* Manages the different events that happens in the window */
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

        /* The mapping is made with a Playstation controller */
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
                    /* Resetting the rotation */
                    level.rotation = Matrix3f::Identity();
                    level.update();
                    break;

                case 12: /*Playstation button*/
                    /* Switching from spherical mode to equirectangular, and vice-versa */
                    cout << "mode : " << mode << endl;
                    //Problem: the game detects the push twice
                    if (mode == 0)
                    {
                        mode = 3;
                        level.mode = 3;
                    }
                    else if (mode == 3)
                    {
                        mode = 0;
                        level.mode = 0;
                    }
                    level.rotateWorld();
                    setPawnPosition(pawn, ball, level.referenceMap, level.rotation);
                    break;
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

/* Checks if the ball is close enough to the goal of the level */
bool isArrived(Ball ball, Eigen::Vector2f goal)
{
    float distanceToGoal = pow(ball.x - goal(0), 2) + pow(ball.y - goal(1), 2);
    float distMin = 100.;
    if (mode == 1) distMin *= 4;    // 4K mode
    if (mode == 2) distMin /= 4;    // Worldeye mode
    return (distanceToGoal < distMin);
}

/* Displays a message saying the player finished the level, then switches to the next one */
void levelComplete(RenderWindow &window, Level* &level, int &levelNumber, Ball &ball)
{
    cout << "Well done, you've finished level " << levelNumber << "!" << endl;

    int textSize = 30;
    if (mode == 1) textSize *= 2;   // 4K mode

    /* If there is a next level */
    if (levelNumber < levelMax)
    {
        Text text("   Well done, you've finished the level! \nClick on any button to go to the next one", font, textSize);
        text.setFillColor(Color::White);
        sf::FloatRect textRect = text.getLocalBounds();
        text.setOrigin(textRect.left + textRect.width/2.0f,
                       textRect.top  + textRect.height/2.0f);
        text.setPosition(window.getSize().x / 2., window.getSize().y / 2.);

        /* The window displaying the congratulation text stays open until a button is pressed */
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
        /* Opening the next level */
        level = new Level(levelNumber, mode);   //TODO: have it loaded while the message is displayed
        ball = *new Ball();
        ball.x = level->chromaMap.getSize().x / 3;  //TODO: adapt the starting position of the ball to the map
        ball.y = level->chromaMap.getSize().y / 2;

        window.create(VideoMode(level->chromaMap.getSize().x, level->chromaMap.getSize().y), "Chroma-depth maze", Style::Default);
    }

    /* If this was the last level */
    else
    {
        Text text("   Well done, you've finished all the levels! \nClick on any button to go to exit the game", font, textSize);
        text.setFillColor(Color::White);
        sf::FloatRect textRect = text.getLocalBounds();
        text.setOrigin(textRect.left + textRect.width/2.0f,
                       textRect.top  + textRect.height/2.0f);
        text.setPosition(window.getSize().x / 2., window.getSize().y / 2.);

        /* The window displaying the congratulation text stays open until a button is pressed */
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
    /* Loading of the "levelNumber"th level with the defined mode */
    Level* level = new Level(levelNumber, mode);
    cout << "Goal: " << level->goal(0) << "; " << level->goal(1) << endl;

    /* Creation of the main window */
    sf::Vector2u size = level->chromaMap.getSize();
    sf::Vector2u refSize = level->referenceMap.getSize();
    RenderWindow window(VideoMode(size.x, size.y), "Chroma-depth maze", Style::Default);

    /* Setting the detection threshold of the game controller */
    window.setJoystickThreshold(20);    //Need to adapt to the game controller's type

    font.loadFromFile("ARIALN.TTF");

    /* Initialization of the map image */
    Sprite sprite;
    Texture texture;
    texture.loadFromImage(level->chromaMap);
    texture.setSmooth(true);
    sprite.setTexture(texture);

    /* Creation of the graphic image of the ball, named pawn */
    CircleShape pawn(30.f);
    pawn.setFillColor(sf::Color::Black);
    pawn.setOutlineColor(sf::Color(255, 255, 255, 150));
    pawn.setOutlineThickness(2.f);

    /* Setting of the pawn depending on the mode */
    if (mode == 1)  // 4K mode
    {
        offSet = 120.;
        pawn.setRadius(2*pawn.getRadius());
        pawn.setOutlineThickness(2*pawn.getOutlineThickness());
    }
    if (mode == 2)  // Worldeye mode
    {
        pawn.setRadius(2/pawn.getRadius());
        pawn.setOutlineThickness(2/pawn.getOutlineThickness());
    }

    /* Translating the image of the potential offset */
    sprite.setPosition(0., offSet);

    cout << "Holes' list :" << endl;
    for (Vector4i hole : level->holesList)
        cout << hole(0) << "; " << hole(1) << "; " << hole(2) << "; " << hole(3) << endl << endl;

    /* Saving the zMap in the "zMap_grey.png" image */
    level->saveZMap("z_maps/zMap_grey.png");


    /* Initialization of the ball */
    Ball ball(refSize.x / 2., refSize.y / 2., Eigen::Vector3f(0., 0., 0.));
//    Ball ball(1300, 693, Eigen::Vector3f(20., 0., 0.));
    ball.z = level->zMap(ball.x, ball.y);
    ball.radius = 10. + ball.z / 20.;
    ball.adaptMode(mode);

    Clock clock;
    Time elapsedTime;

    while(window.isOpen())
    {
        Event event;
        /* Managing the window's events */
        while(window.pollEvent(event))
        {
            manageEvents(event, window, ball, pawn, clock, *level);
            if (!window.isOpen()) return 0;
        }

        /* Computation of the ball's next position */
        elapsedTime = clock.restart();
        /* Getting the joysticks input to compute the ball's acceleration and the potential rotation of the world */
        checkControllerState(window, ball, pawn, *level);
        /* Applying the gravitation force */
        level->makeBallFall(ball, elapsedTime);

        /* Updating the map's displayed image */
        texture.update(level->chromaMap);

        /* Seting the ball's image's position and radius */
        pawn.setRadius(ball.radius);
        setPawnPosition(pawn, ball, level->referenceMap, level->rotation);

        /* Checking if the ball is close to the goal, and switching to next level if so */
        if (isArrived(ball, level->goal)) levelComplete(window, level, levelNumber, ball);

        /* Displaying everything on the window */
        window.clear();
        window.draw(sprite);
        if (ball.visible)
            window.draw(pawn);

        window.display();

    }
    return 0;
}
