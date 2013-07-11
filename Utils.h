#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>

#ifndef UTILS_H
#define UTILS_H

using namespace std;

bool getTurningPoint(const cv::Point2f& robot,const cv::Point2f& center,const cv::Point& target,cv::Point2f &turningPoint,float radius);

void calib();

IplImage *loadDatImage(char *fn);

IplImage* get_hsv(const IplImage* img);

std::vector<cv::Point3f> ball_detection(const IplImage* img, const cv::Point2f& hb, const cv::Point2f& sb, const cv::Point2f& vb);//img BGR

#endif // UTILS_H
