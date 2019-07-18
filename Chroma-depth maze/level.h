#ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
#endif

#ifndef M_PI
 #define M_PI 3.14159
#endif

#include <list>
#include <math.h>
#include <string>
#include <SFML/Graphics.hpp>
#include "../eigen-eigen-323c052e1731/Eigen/Dense"
#include "ball.h"


using namespace Eigen;
using namespace std;
using namespace sf;

class Level
{
public:

    bool is4K;
    bool gridOption = true;
    bool textureOption = true;
    bool shadeOption = true;
    int levelNumber;

    Image referenceMap;
    Image chromaMap;
    Matrix3f rotation;

    MatrixXf zMap;
    float zMax;
    Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap;
    list<Eigen::Vector4i> holesList;
    Eigen::Vector2f goal;

    Level(int levelNum, bool is4K)
    {
        this->levelNumber = levelNum;
        this->is4K = is4K;
        loadMap(chromaMap);
        loadMap(referenceMap);
        rotation = Matrix3f::Identity();

        zMax = 100.;
        if (is4K) zMax *= 4.;

        buildSphereMap();
    }

    ~Level(){}

    void rotateWorld()
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

    void makeBallFall(Ball &ball, const Time elapsedTime)
    {
        Eigen::Vector3f gravitation(0., 0., -1.);
        if (is4K) gravitation *= 2.;

        Eigen::Vector3f beforePos(ball.x , ball.y, ball.z);
        Eigen::Vector3f v = ball.v;

        Eigen::Vector3f normal = normalMap((int) ball.x, (int) ball.y);
        v += 10. * elapsedTime.asSeconds() * (gravitation + normal + ball.a);

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
        if (zMap((int) pos(0), (int) pos(1)) < - 2 * zMax * 0.95)
        {
            for(Eigen::Vector4i hole : holesList)
            {
                float distMin = 100.;
                if (is4K) distMin *= 4.;
                if (pow(pos(0) - hole(0), 2) + pow(pos(1) - hole(1), 2) <= distMin)   /* The ball is close to the hole */
                {
                    v = Eigen::Vector3f(-v(0), v(1), -v(2));
                    pos = Eigen::Vector3f(hole(2), hole(3), zMap(hole(2), hole(3))) + (zMax * 0.1 + 10.) * v.normalized();
                }
            }
        }

        ball.v = v;
        ball.x = pos(0);
        ball.y = pos(1);
        ball.z = zMap((int) ball.x, (int) ball.y);
        if (is4K) ball.radius = 20. + ball.z / 20;
        else ball.radius = 10. + ball.z / 20.;
    }

    void centerMap(CircleShape &pawn)
    {
        sf::Vector2i size = (sf::Vector2i) chromaMap.getSize();

        /* We use the opposite rotation */
        float dX = (float) pawn.getPosition().x + pawn.getRadius() - size.x / 2.;
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

        rotateWorld();

        pawn.setPosition(size.x - 2, size.y - 2);

        cout << "Finished centering the map" << endl;
    }

    void saveZMap(const string fileName)
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

    void update()
    {
        loadMap(referenceMap);
        rotateWorld();
    }

    void nextLevel(int levelNumber)
    {
        this->levelNumber = levelNumber;
        loadMap(chromaMap);
        loadMap(referenceMap);
        rotation = Matrix3f::Identity();

        buildSphereMap();
    }

private:
    string mapPath;
    string zMapPath;


    void loadMap(Image &map)
    {
        mapPath = "maps/";
        if (levelNumber < 0) mapPath += "Test";
        else
        {
            mapPath += "lv";
            mapPath += to_string(levelNumber);
        }
        mapPath += "/map";

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

    void buildSphereMap()
    {
        Image zImage;

        zMapPath = "z_maps/";

        if (levelNumber < 0) zMapPath += "Test";
        else
        {
            zMapPath += "lv";
            zMapPath += to_string(levelNumber);
        }

        zMapPath += "/z_map_hole";
        if (is4K) zMapPath += "_4K";
        zMapPath += ".png";

        if (!zImage.loadFromFile(zMapPath))
        {
            cout << "Couldn't open the zmap file" << endl;
        }
        else
        {
            cout << "Started loading zMap" << endl;

            Vector2u size = zImage.getSize();
            zMap.resize(size.x, size.y);
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

                    if (color == Color::White)
                        goal = Eigen::Vector2f(i, j);

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

                    zMap(i, j) = z;
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

                float iPlus = zMap(i+margin, j);
                float iMinus = zMap(i-margin, j);
                float jPlus = zMap(i, j+margin);
                float jMinus = zMap(i, j-margin);
                float zNormal = zMap(i, j);

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
    }

};
