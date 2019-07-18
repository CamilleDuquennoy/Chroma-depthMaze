

class Level
{
public:
    string mapPath;
    string zMapPath;
    Image *referenceMap;
    Image *chromaMap;
    MatrixXf zMap;
    Matrix<Eigen::Vector3f, Dynamic, Dynamic> normalMap;
    list<Eigen::Vector4i> holesList;
    Eigen::Vector2f goal;

    Matrix3f rotation;

    Level(int levelNumber)
    {
        chromaMap = new Image();
        loadMap(*chromaMap);
        referenceMap = new Image();
        loadMap(*referenceMap);

        rotation = Matrix3f::Identity();

        buildSphereMap(zMap, normalMap, holesList);
    };

    ~Level()
    {

    };

private:
    void loadMap(Image &map)
    {
        mapPath = "maps/";
        if (level < 0) mapPath += "Test";
        else
        {
            mapPath += "lv";
            mapPath += to_string(level);
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
    };

    void buildSphereMap(MatrixXf &sphereMap, Matrix<Eigen::Vector3f, Dynamic, Dynamic> &normalMap, list<Eigen::Vector4i> &holesList)
    {
        Image zImage;

        zMapPath = "z_maps/";

        if (level < 0) zMapPath += "Test";
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
            zMax = 100.;
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

};
