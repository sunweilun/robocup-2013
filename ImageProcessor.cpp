#include "ImageProcessor.h"

void ImageProcessor::setBound(const cv::Point2f &h_bound,const cv::Point2f &s_bound,const cv::Point2f &v_bound)
{
    this->h_bound = h_bound;
    this->s_bound = s_bound;
    this->v_bound = v_bound;
}

bool ImageProcessor::inBound(float h,float s,float v)
{
    bool h_inbound = h>=h_bound.x && h<=h_bound.y;
    bool s_inbound = s>=s_bound.x && s<=s_bound.y;
    bool v_inbound = v>=v_bound.x && v<=v_bound.y;
    return h_inbound && s_inbound && v_inbound;
}

IplImage* ImageProcessor::eliminateBackground(const IplImage* hsv_img)
{
    IplImage* eli_image = cvCreateImage(cvGetSize(hsv_img),IPL_DEPTH_32F,3);
    memcpy(eli_image->imageData,hsv_img->imageData,4*3*hsv_img->width*hsv_img->height);
    float *hsv_data = (float*) hsv_img->imageData;
    float *eli_data = (float*) eli_image->imageData;
    for(int x=0;x<eli_image->width;x++)
    {
        bool inb = false;
        for(int y=0;!inb && y<eli_image->height;y++)
        {
            int idx = 3*(y*eli_image->width+x);
            float h = hsv_data[idx];
            float s = hsv_data[idx+1];
            float v = hsv_data[idx+2];
            inb = inBound(h,s,v);
            if(!inb)
            {
                eli_data[idx+2] = 0;
            }
        }
    }
    return eli_image;
}

IplImage* ImageProcessor::extractColorBlocks(const IplImage* hsv_img)
{
    IplImage* cb = cvCreateImage(cvGetSize(hsv_img),IPL_DEPTH_8U,1);
    memset(cb->imageData,0,cb->width*cb->height);
    float* hsv_data = (float*) hsv_img->imageData;
    for(int i=0;i<hsv_img->width*hsv_img->height;i++)
    {
        float h = hsv_data[i*3];
        float s = hsv_data[i*3+1];
        float v = hsv_data[i*3+2];
        if(inBound(h,s,v))
        {
            cb->imageData[i] = 255;
        }
    }
    return cb;
}
