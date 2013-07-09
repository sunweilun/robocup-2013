#ifndef ROBOT_H
#define ROBOT_H
#include "WorldMap.h"


class Robot
{
    friend class WorldMap;
private:
    IplImage* image;
    int imgCounter;
    float x,y,ori;
    WorldMap worldMap;
    void getImage();
public:
    Robot();
    void setCoord(float x,float y,float ori);
    void turnLeft(float angle);
    void turnRight(float angle);
    void moveForward(float dist,float max_speed);
    void drawMap();
    ~Robot();
};

#endif // ROBOT_H
