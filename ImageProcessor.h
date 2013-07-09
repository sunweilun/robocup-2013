#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <stdio.h>
#include "Utils.h"

class ImageProcessor
{
private:
    cv::Point2f h_bound,s_bound,v_bound;
    bool inBound(float h,float s,float v);
public:
    void setBound(const cv::Point2f &h_bound,const cv::Point2f &s_bound,const cv::Point2f &v_bound);
    IplImage* eliminateBackground(const IplImage* hsv_img);
    IplImage* extractColorBlocks(const IplImage* hsv_img);
};

#endif // IMAGEPROCESSOR_H
