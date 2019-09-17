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

/* Takes angles theta and phi and returns a rotation matrix of those two angles */
Matrix3f angleToRotation(float theta, float phi)
{
    Matrix3f thetaRot;
    thetaRot <<     cos(theta),    -sin(theta), 0.,
                    sin(theta),     cos(theta), 0.,
                    0.,              0.,          1.;

    Matrix3f phiRot;
    phiRot <<       cos(phi),      0.,     sin(phi),
                    0.,             1.,     0.,
                    -sin(phi),     0.,     cos(phi);

    return thetaRot * phiRot;
}

/* Takes a rotation matrix, the i and j coordinates in a pixel, the size of the image,
Then computes the new coordinates of i and j once rotated */
void rotateCoord(Matrix3f rotation, sf::Vector2i size, float i, float j, float &newI, float &newJ)
{
    float theta = (float) i / size.x * (2.*M_PI) - M_PI;
    float phi = (float) j / size.y * M_PI;

    Eigen::Vector3f newCartesianPos = rotation * Eigen::Vector3f(sin(phi) * cos(theta),
                                                                 sin(phi) * sin(theta),
                                                                 cos(phi));

    float newPhi = acos(newCartesianPos(2));
    float newTheta = atan2(newCartesianPos(1), newCartesianPos(0)) + M_PI;


    newI = min((double) size.x - 1, (float) newTheta * size.x / (2*M_PI));
    newJ = min((double) size.y - 1, (float) newPhi * size.y / (M_PI));
}

/* Computes the cartesian x and y coordinates of the i and j spherical coordinates of an image, with size being the size of the image */
void sphericalToCartesian(int i, int j, float &x, float &y, sf::Vector2i size)
{
    float rho = min(size.x, size.y) / 2.;
    float theta = (float) i / size.x * 2. * M_PI - M_PI;
    float phi = (float) j / size.y * M_PI;

    x = sin(phi) * sin(theta) * rho + size.x / 2.;
    y = -cos(phi) * rho + size.y / 2.;
}

/* Computes the spherical theta and phi coordinates of the i and j cartesian coordinates of an image, with size being the size of the image */
void cartesianToSpherical(int i, int j, float &theta, float &phi, sf::Vector2i size)
{
    float rho = min(size.x, size.y) / 2.;
    float x = (float) (i - size.x / 2.) / rho;
    float y = (float) (-j + size.y / 2.) / rho;

    phi = acos(y);
    if (sin(phi) != 0)
        theta = asin(x / sin(phi));
    else
        theta = 0.;

    theta = (theta + M_PI) * size.x / (2*M_PI);
    phi *= size.y / M_PI;
}

class Level
{
public:

    int mode = 0; /* 0 is normal HD screen, 1 is 4K Geo-Cosmos, 2 is Worldeye screen, 3 is normal screen in sphere mode */
    bool gridOption = true;
    bool textureOption = true;
    bool shadeOption = true;
    int levelNumber;

    Image referenceMap; /* The original image of the level from which the computations are made, that is not rotated nor resized */
    Image chromaMap;    /* The displayed image of the level, that can be rotated or resized */
    Matrix3f rotation;

    MatrixXf zMap;      /* zMap computed from the referenceMap image */
    float zMax;         /* Highest Z */
    Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap;    /* normalMap computed from the referenceMap image */
    Eigen::Vector2f goal;

    /* List of the holes, the two first int are the coordinates of the entrance point, the two last the exit point.
    Each hole is twice in the list, so that the two ends are both entries and exits */
    list<Eigen::Vector4i> holesList;

    /* Constructor */
    Level(int levelNumber, int mode)
    {
        this->levelNumber = levelNumber;
        this->mode = mode;
        /* Loading of the reference and displayed images of the map */
        loadMap(chromaMap);
        loadMap(referenceMap);
        rotation = Matrix3f::Identity();

        zMax = 100.;
        if (mode == 1) zMax *= 4.;  // 4K mode

        /* Building the zMap, the normalMap and the holes' list */
        buildSphereMap();

        if (mode > 1) switchToSphericalDisplay();   // Sphere mode
    }

    ~Level(){}

    /* Wraps the map on a sphere for the sphere mode */
    void switchToSphericalDisplay()
    {
        sf::Vector2i size = (sf::Vector2i) chromaMap.getSize();
        sf::Vector2i newSize = size;
        if (mode == 2) newSize = sf::Vector2i(640, 480);

        /* Cleaning the image */
        chromaMap.create(newSize.x, newSize.y, Color::Black);

        /* Going through the pixels of the sphere to compute which one of the equirectangular one it corresponds to */
        for (int i = (newSize.x - newSize.y)/2; i < newSize.x + newSize.y/2; i++)
        {
            for (int j = 0; j < newSize.y; j++)
            {
                float rho = min(newSize.x, newSize.y) / 2.;
                float distanceToCenter = pow((i - newSize.x / 2) / rho, 2) + pow((j - newSize.y / 2.) / rho, 2);
                if (distanceToCenter <= 1.) // If the pixel is inside the circle
                {
                    /* Computing the phi and theta coordinates in the equirectangular image */
                    float theta, phi;
                    cartesianToSpherical(i, j, theta, phi, newSize);

                    if (mode == 2)  // Worldeye mode
                        chromaMap.setPixel(i, j, referenceMap.getPixel(theta * size.x / newSize.x, phi * size.y / newSize.y));
                    else
                        chromaMap.setPixel(i, j, referenceMap.getPixel(theta, phi));
                }
            }
        }
    }

    /* Computes the chromaMap by rotating the referenceMap with the rotation matrix
    If it is in sphere mode, the function then wraps it on a sphere */
    void rotateWorld()
    {
        sf::Vector2i size = (sf::Vector2i) referenceMap.getSize();

        if (mode <= 1)  // Equirectangular mode
        {
            /* Going through the pixels of the chromaMap to compute which one of the referenceMap one it corresponds to */
            for (int i = 0; i < size.x; i++)
            {
                for (int j = 0; j < size.y; j++)
                {
                    float newI, newJ;
                    rotateCoord(rotation, size, i, j , newI, newJ);
                    chromaMap.setPixel(i, j, referenceMap.getPixel((int) newI, (int) newJ));
                }
            }
        }


        else   //Sphere mode
        {
            sf::Vector2i newSize = size;
            if (mode == 2) newSize = sf::Vector2i(640, 480);    // Worldeye mode

            /* Cleaning the image */
            chromaMap.create(newSize.x, newSize.y, Color::Black);

            /* Going through the pixels of the sphere of the chromaMap
            To compute which one of the equirectangular referenceMap one it corresponds to */
            for (int i = (newSize.x - newSize.y)/2; i < newSize.x + newSize.y/2; i++)
            {
                for (int j = 0; j < newSize.y; j++)
                {
                    float rho = min(newSize.x, newSize.y) / 2.;
                    float distanceToCenter = pow((i - newSize.x / 2) / rho, 2) + pow((j - newSize.y / 2.) / rho, 2);
                    if (distanceToCenter <= 1.) // If the pixel is inside the circle
                    {
                        /* Computing the phi and theta coordinates in the equirectangular rotated image */
                        float theta, phi;
                        cartesianToSpherical(i, j, theta, phi, newSize);

                        /* Computing the newI and newJ coordinates in the referenceMap */
                        float newI, newJ;
                        if (mode == 2)  // Worldeye mode
                            rotateCoord(rotation, size, theta * size.x / newSize.x, phi * size.y / newSize.y, newI, newJ);
                        else
                            rotateCoord(rotation, size, theta, phi, newI, newJ);

                        chromaMap.setPixel(i, j, referenceMap.getPixel(newI, newJ));
                    }
                }
            }
        }
    }

    /* Computes where the ball goes after a time "elapsedTime" has passed */
    void makeBallFall(Ball &ball, const Time elapsedTime)
    {
        /* Initialization of the gravitation force */
        Eigen::Vector3f gravitation(0., 0., -1.);
        if (mode == 1) gravitation *= 2.;

        /* Copying the ball's position and speed */
        Eigen::Vector3f beforePos(ball.x , ball.y, ball.z);
        Eigen::Vector3f v = ball.v;

        /* Initialization of the normal of the floor where the ball is */
        Eigen::Vector3f normal = normalMap((int) ball.x, (int) ball.y);
        if (mode == 1) normal *= 2.;    // 4K mode
        /* Integration of the forces and the ball's acceleration */
        v += 10. * elapsedTime.asSeconds() * (gravitation + normal + ball.a);

        float distance = (elapsedTime.asSeconds() * v).norm();
        Eigen::Vector3f direction = v.normalized();

        Eigen::Vector3f pos = beforePos;
        float radius = ball.radius;

        /* Checking wall collisions */
        /* The loop goes through each pixel up to the supposed final position, and stops if it encounters any wall */
        for (float t = 0.; t < distance; t += 0.1)
        {
            pos = beforePos + t * direction;

            /* Dealing the borders problems */
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
            /* Checking if the ball is close to a hole */
            for(Eigen::Vector4i hole : holesList)
            {
                float distMin = 100.;
                if (mode == 1) distMin *= 4.;   // 4K mode
                if (pow(pos(0) - hole(0), 2) + pow(pos(1) - hole(1), 2) <= distMin)   /* The ball is close to the hole */
                {
                    /* Getting the ball to the other side of the hole, moving it a bit aside of the hole, and modifying the speed accordingly */
                    v = Eigen::Vector3f(-v(0), v(1), -v(2));
                    pos = Eigen::Vector3f(hole(2), hole(3), zMap(hole(2), hole(3))) + (zMax * 0.1 + 10.) * v.normalized();
                }
            }
        }

        ball.v = v;
        ball.x = pos(0);
        ball.y = pos(1);
        ball.z = zMap((int) ball.x, (int) ball.y);

        if (mode == 1) ball.radius = 20. + ball.z/2. / 20;  // 4K mode
        else ball.radius = 10. + ball.z / 20.;
    }

    /* Rotates the map to center it on the ball's position */
    void centerMap(Ball ball)
    {
        sf::Vector2i size = (sf::Vector2i) referenceMap.getSize();

        /* We use the opposite rotation */
        float dX = (float) ball.x - size.x / 2.;
        float dY = (float) ball.y - size.y / 2.;
        float dTheta = dX / (float) size.x * 2. * M_PI;
        float dPhi = dY / (float) size.y * M_PI;

        /* Computing the rotation matrix of angles dPhi and dTheta */
        rotation = angleToRotation(dTheta, dPhi);

        rotateWorld();

        cout << "Finished centering the map" << endl;
    }

    /* Creates a grey image of the zMap and saves it as the fileName */
    void saveZMap(const string fileName)
    {
        /* Initialization */
        Image img;
        img.create(zMap.rows(), zMap.cols());

        for(int i = 0; i < zMap.rows(); i++)
        {
            for (int j = 0; j < zMap.cols(); j++)
            {
                /* Computing the grey value of the depth of the pixel */
                float z;
                if (mode == 1) z = (200. + zMap(i, j) / 4.) / 300. * 255.;  // 4K mode
                else z = (200. + zMap(i, j)) / 300. * 255.;
                img.setPixel(i, j, Color(z, z, z));
            }
        }

        if (!img.saveToFile(fileName)) cout << "Couldn't save the zMap file" << endl;
        else cout << "Saved the zMap file" << endl;
    }

    /* Updates the chromaMap and the referenceMap */
    void update()
    {
        loadMap(referenceMap);
        rotateWorld();
    }

private:
    string mapPath;     /* path name to the referenceMap image */
    string zMapPath;    /* path name to the image used in buildSphere() to compute the zMap, normalMap, holesList and goal point */

    /* Loads the referenceMap depending on the options, the level and the 4K mode */
    void loadMap(Image &map)
    {
        /* Setting the pathName depending on the options */
        mapPath = "maps/";

        /* Checking the level's number */
        if (levelNumber == 0) mapPath += "Test";
        else
        {
            mapPath += "lv";
            mapPath += to_string(levelNumber);
        }

        mapPath += "/map";

        /* Checking the mode is 4K */
        if (mode == 1) mapPath += "_4K";

        /* Checking the other options */
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

    /* Builds the level's zMap, normalMap and holesList from the zMap image */
    void buildSphereMap()
    {
        Image zImage;

        /* Setting the pathName depending on the options */
        zMapPath = "z_maps/";

        /* Checking the level's number */
        if (levelNumber == 0) zMapPath += "Test";
        else
        {
            zMapPath += "lv";
            zMapPath += to_string(levelNumber);
        }

        zMapPath += "/z_map_hole";
        /* Checking the mode is 4K */
        if (mode == 1) zMapPath += "_4K";
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

                    /* Checking if it is the goal */
                    if (color == Color::White)
                        goal = Eigen::Vector2f(i, j);

                    /* Computes z depending on the color */
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
                                if (G > 0.1)
                                    z = (G / 2 - B) * zMax;

                                else
                                    z = (B - 2.) * zMax;
                            }
                        }
                    }

                    /* Creation of the holes list */
                    if (color.a < 250)
                    {
                        /* Pixel is partly transparent: it's a hole */
                        int a = color.a;
                        z = -2.*zMax;

                        Eigen::Vector4i newHole(i, j, -1, -1);
                        /* Checking if another pixel has the same transparency to find the other side of the hole */
                        for (Eigen::Vector4i hole : holesList)
                        {
                            int k = hole(0);
                            int l = hole(1);

                            /* Checking if the transparency if close while position is different */
                            if (abs(zImage.getPixel(k, l).a - a) < 50 && (pow(k-i, 2) + pow(l-j, 2) > 100))
                            {
                                /* The two first int are the start of the hole, the next two the arrival point */
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

        /* Initialization of the normals */
        for (int i = 0; i < normalMap.rows(); i++)
        {
            for (int j = 0; j < normalMap.cols(); j++)
            {
                normalMap(i, j) = Eigen::Vector3f(0., 0., 1.);
            }
        }

        int margin = 25;
        if (mode == 1) margin *= 2; // 4K mode
        /* Computes the cross product of the the vectors representing the z slope in x and y axis */
        for (int i = margin; i < normalMap.rows() - margin; i++)
        {
            for (int j = margin; j < normalMap.cols() - margin; j++)
            {
                Eigen::Vector3f n;

                float zIPlus = zMap(i+margin, j);
                float zIMinus = zMap(i-margin, j);
                float zJPlus = zMap(i, j+margin);
                float zJMinus = zMap(i, j-margin);
                float zNormal = zMap(i, j);

                /* If the z difference is too big: it's a wall, the function won't take it into account */
                float wallLimit = 50.;
                if (mode == 1) wallLimit *= 4;  // 4K mode
                if (abs(zIPlus - zNormal) > wallLimit)
                    zIPlus = zNormal;
                if (abs(zIMinus - zNormal) > wallLimit)
                    zIMinus = zNormal;
                if (abs(zJPlus - zNormal) > wallLimit)
                    zJPlus = zNormal;
                if (abs(zJMinus - zNormal) > wallLimit)
                    zJMinus = zNormal;

                Eigen::Vector3f xVector = Eigen::Vector3f(2*margin, 0., zIPlus - zIMinus);
                Eigen::Vector3f yVector = Eigen::Vector3f(0., 2*margin, zJPlus - zJMinus);
                n = xVector.cross(yVector);
                n.normalize();
                normalMap(i, j) = n;
            }
        }
        cout << "Finished loading zMap" << endl << endl;
    }

};
