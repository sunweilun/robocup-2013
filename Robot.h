#ifndef ROBOT_H
#define ROBOT_H
#include "WorldMap.h"
#include "BallTracker.h"
#include <stdio.h>
#include <sys/time.h>


class Robot
{
    friend class WorldMap;
    friend class BallTracker;
private:
    BallTracker ballTracker;
    cv::Point2f ball_velocity;
    cv::Point2f world2image(const cv::Point2f& coord);
    cv::Point2f image2world(const cv::Point2f& coord);
    bool radar; // 1 when radar is on, 0 otherwise
    bool ballLocated,ownGoalLocated,oppGoalLocated;
    ImageProcessor ip;
    IplImage* image_l; // stores newly acquired image
    IplImage* image_r;
    int imgCounter; // counts number of images
    float x,y,ori; // ori--orientateion
    cv::Point2f ball_coord,ownGoal_coord,oppGoal_coord;
    cv::Point2f ownGoal_frontDir;
    float ownGoal_width;
    WorldMap worldMap;
    void getImage(); // get a new image and store it in *image_l and *image_r
    bool locateBall(); // locate ball_coord according to *image_r
    bool locateOwnGate();
    void updateRadar();
    bool getBallInfo(cv::Point2f &ballVelocity,cv::Point2f &ballPosition);
    std::vector<cv::Point2f> shootRoute;
public:
    Robot();
    void keepGoal();
    void rotateTo(const cv::Point2f &new_dir);
    void radarOn();
    void radarOff();
    void setCoord(float x,float y,float ori);
    void turnLeft(float angle); // angle in degrees
    void turnRight(float angle); // angle in degrees
    void moveForward(float dist,float max_speed);
    void moveTo(const cv::Point2f& wCoord,float max_speed);
    void moveRotate(bool isLeft, int radius, float arc);//time is in second
    void spin();//std::vector<cv::Point2f> balls);
    void shoot();
    void drawMap(); // excutes predefined movement to draw the map
    void findBall(); // rotate until the target ball is located
    std::vector<cv::Point2f> findMulBall();
    ~Robot();
};

#endif // ROBOT_H
