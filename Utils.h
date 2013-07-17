#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>

#ifndef UTILS_H
#define UTILS_H

using namespace std;

bool getTurningPoint(const cv::Point2f& robot,const cv::Point2f& center,const cv::Point& target,cv::Point2f &turningPoint,float radius);

float cal_distance(const cv::Point2f& p1, const cv::Point2f& p2);

IplImage *loadDatImage(char *fn);

IplImage* get_hsv(const IplImage* img);

std::vector<cv::Point3f> ball_detection(const IplImage* img, const cv::Point2f& hb, const cv::Point2f& sb, const cv::Point2f& vb);//img BGR

float length(const cv::Point2f &p);

float getTime(const cv::Point2f &p1,const cv::Point2f &d1,const cv::Point2f &p2,const cv::Point2f &d2);

#endif // UTILS_H
