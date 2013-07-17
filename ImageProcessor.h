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
    std::vector<cv::Point3f> extractCircles(const IplImage* img);
    void extractMulCircles(const IplImage* img, std::vector<cv::Point3f>& res);
    //sk add
    IplImage* deleteNoise(const IplImage* image);
    bool getOnlyBlue(const IplImage * src, IplImage * blue);
    int* scanUp(IplImage* blue);
    IplImage* getBound(int *bound);
    bool leftBlue(IplImage* blue, int i, int j);
    bool rightBlue(IplImage* blue, int i, int j);
    bool stepUpJudge(IplImage* blue, int i, int j);
};

#endif // IMAGEPROCESSOR_H
