#ifndef ROBOT_H
#define ROBOT_H
#include "WorldMap.h"
#include "BallTracker.h"
#include "getPhoto2.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
class Robot
{
    friend class WorldMap;
    friend class BallTracker;
    friend void* keeperMotionThread(void* params);
    friend void*stopKeeperThread(void * args);
private:
    timespec timeBase;
    bool abort;
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
    void updateBallStatus();
    bool adjustWorldCoordinate(IplImage* image, double coordAdjustRate=0);
    double mainAngle;
    double mainGroupId;
    struct myLine
    {
        cv::Point2f p[2];//start point and end point
        double theta;//angle
        double l;//length
        int clsId;//a number labeling a temporary orientation group
        myLine()
            {p[0].x=0;p[0].y=0;p[1].x=0;p[1].y=0;theta=0;l=0;clsId=-1;};
        myLine(const myLine& myL)
            {p[0]=myL.p[0];p[1]=myL.p[1];theta=myL.theta;l=myL.l;clsId=myL.clsId;};
        myLine(const cv::Point2f& p1, const cv::Point2f& p2)
            {p[0].x=p1.x;p[0].y=p1.y;p[1].x=p2.x;p[1].y=p2.y;
            theta=atan2(float(p2.y-p1.y),float(p2.x-p1.x));
            l=sqrt(float((p2.y-p1.y)*(p2.y-p1.y))
                    +float((p2.x-p1.x)*(p2.x-p1.x)));
            clsId=-1;};
    };
    int maxLines;//limit the maximum number of lines kept
    vector<myLine> lines;//record most lines
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
    void moveRotate(bool isLeft, float radius, float arc);//time is in second
    void spin();//std::vector<cv::Point2f> balls);
    void shoot();
    void drawMap(); // excutes predefined movement to draw the map
    void findBall(); // rotate until the target ball is located
    std::vector<cv::Point2f> findMulBall();
    ~Robot();
    int init_socket(const char* ipStr, const int host);
    void listenAndAct();
    int socket_robo;
};

#endif // ROBOT_H
