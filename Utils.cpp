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

std::vector<cv::Point3f> ball_detection(const IplImage* img, const cv::Point2f& hb, const cv::Point2f& sb, const cv::Point2f& vb)//img BGR
{
	std::vector<cv::Point3f> ret;

	IplImage *image = cvCreateImage(cvSize(img->width*2,img->height*2),IPL_DEPTH_8U,3);
	cvSet(image,cvScalar(255,255,255));
	for(int i=img->height/2;i<img->height/2+img->height;++i)
	{
		memcpy(image->imageData+(i*image->width+img->width/2)*3,img->imageData+(i-img->height/2)*img->width*3,img->width*3);
	}
	IplImage *image_Gray = cvCreateImage(cvSize(image->width,image->height),IPL_DEPTH_8U,1);
	cvCvtColor( image, image_Gray, CV_BGR2GRAY );
	CvMemStorage* storage = cvCreateMemStorage(0);
	cvSmooth( image_Gray, image_Gray, CV_GAUSSIAN, 7, 7 ); // smooth it, otherwise a lot of false circles may be detected
	CvSeq* circles = cvHoughCircles( image_Gray, storage, CV_HOUGH_GRADIENT, 2, image_Gray->height/4, 200, 10,15);
	int i;
	for( i = 0; i < circles->total; i++ )
	{
		float* p = (float*)cvGetSeqElem( circles, i );
		int color[3]={0,0,0};
		int num=0;
		for(int y=cvRound(p[1])-cvRound(p[2]);y<cvRound(p[1])+cvRound(p[2]);++y)
		{
			if(y<img->height/2 || y>img->height/2+img->height)
				continue;
			int x0=sqrt(cvRound(p[2])*cvRound(p[2])-(double)((cvRound(p[1])-y)*(cvRound(p[1])-y)));
			for(int x=cvRound(p[0])-x0;x<cvRound(p[0])+x0;++x)
			{
				if(x<img->width/2 || x>img->width/2+img->width)
					continue;
				for(int j=0;j<3;++j)
					color[j]+=(unsigned char)(image->imageData[(y*image->width+x)*3+j]);
				++num;
			}
		}
		if(num==0)
			continue;
		for(int j=0;j<3;++j)
			color[j]/=num;
		IplImage *float_point = cvCreateImage(cvSize(1,1),IPL_DEPTH_32F,3);
		IplImage *hsv_point = cvCreateImage(cvSize(1,1),IPL_DEPTH_32F,3);
		((float*)float_point->imageData)[0] = ((float)color[0])/(float)255.0;
		((float*)float_point->imageData)[1] = ((float)color[1])/(float)255.0;
		((float*)float_point->imageData)[2] = ((float)color[2])/(float)255.0;
		cvCvtColor(float_point,hsv_point,CV_BGR2HSV);
		float *data = (float*) hsv_point->imageData;
		float h = data[0];
		float s = data[1];
		float v = data[2];

		cvReleaseImage(&float_point);
		cvReleaseImage(&hsv_point);

		if(h<hb.x || h>hb.y)
			continue;
		if(s<sb.x || s>sb.y)
			continue;
		if(v<vb.x || v>vb.y)
			continue;
		cv::Point3f retp;
		retp.x=cvRound(p[0])-img->width/2;
		retp.y=cvRound(p[1])-img->height/2;
		retp.z=cvRound(p[2]);//radius
		ret.push_back(retp);
	}
	//cvReleaseData(&circles);
	cvReleaseImage(&image);
	cvReleaseImage(&image_Gray);
	cvReleaseMemStorage(&storage);
	return ret;
}
