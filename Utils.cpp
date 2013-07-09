#include "Utils.h"

IplImage *loadDatImage(char *fn)
{
    IplImage *image;
    FILE* file = fopen(fn,"r");
    int width,height;
    fscanf(file,"%d %d\n",&width,&height);
    image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    int r,g,b;
    for(int i=0;i<width*height;i++)
    {
        fscanf(file,"%x %x %x\n",&r,&g,&b);
        image->imageData[3*i] = b;
        image->imageData[3*i+1] = g;
        image->imageData[3*i+2] = r;
    }
    fclose(file);
    return image;
}

IplImage* get_hsv(const IplImage* img)
{
    IplImage *hsv_img = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F,3);
    cvConvertScale(img,hsv_img,1/255.0);
    cvCvtColor(hsv_img,hsv_img,CV_BGR2HSV);
    return hsv_img;
}
