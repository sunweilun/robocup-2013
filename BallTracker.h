#include "Utils.h"

#include <deque>
#include "Robot.h"
using namespace std;

#ifndef BALLTRACKER_H
#define BALLTRACKER_H

class Robot;

static int width = 320;
static int height = 240;
static CvScalar BALL_COLOR_HSV = cvScalar(270,0.52,0.5);
const double rg[3]={20,0.2,0.3};

int checkColorThreshold(float* c);

struct BallTracker
{
	deque<cv::Point3f> pos_scr;//x,y,r on screen
	deque<cv::Point3f> pos;//x,y,t in worldCoordinate
	deque<IplImage*> images;
	Robot* robot;

    ~BallTracker();
    void scr2wld(int frameId);
	int pushFrame(const IplImage* image, double t);
	int processFrame(int frameId);
	int popFrame(int numOfFrame);
    cv::Point3f ballDetect(const IplImage* image1);
    void setParent(Robot* robot){ this->robot = robot; };
};

struct Area
{
	int id;
	double x;
	double y;
	int xmin;
	int ymin;
	int xmax;
	int ymax;
	int area;
	Area(int id,int x,int y,int xmin,int xmax,int ymin,int ymax,int area)
	{this->id=id;this->x=x;this->y=y;this->area=area;this->xmin=xmin;this->xmax=xmax;this->ymin=ymin;this->ymax=ymax;};
};

#endif
