#include "Utils.h"

float arctan(float x,float y)//range=(-pi/2,3pi/2)
{
	float angle=asin(y/sqrt(x*x+y*y));
	if(x<0)
		angle=M_PI-angle;
	return angle;
}

float cal_distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    cv::Point2f delta = p1 - p2;
    return sqrt(pow(delta.x,2)+pow(delta.y,2));
}

bool getTurningPoint(const cv::Point2f& robot,const cv::Point2f& center,const cv::Point& target,cv::Point2f &turningPoint,float radius)
{
    float xB=center.x-robot.x;
    float yB=center.y-robot.y;
    float xS=target.x-robot.x;
    float yS=target.y-robot.y;
    float r=radius;
    float xT=0;
    float yT=0;
		float thetaB;
		float thetaBs;
		float thetaT;
		float thetaTs;
		float divider;
    float t;

	float product=xB*yS-yB*xS;
	if(product<=0)
	{
		thetaB=arctan(xB,yB);
		thetaBs=arctan(xB-xS,yS-yB);
		thetaT=thetaB-asin(r/sqrt(xB*xB+yB*yB));
		thetaTs=thetaBs-asin(r/sqrt((xB-xS)*(xB-xS)+(yB-yS)*(yB-yS)));
		divider=sin(thetaT+thetaTs);
		//if(divider!=0)
		{
			t=(xS*sin(thetaTs)+yS*cos(thetaTs))/divider;
		}
		xT=t*cos(thetaT);
		yT=t*sin(thetaT);
	}
	else
	{
		xB=-xB;
		xS=-xS;
		thetaB=arctan(xB,yB);
		thetaBs=arctan(xB-xS,yS-yB);
		thetaT=thetaB-asin(r/sqrt(xB*xB+yB*yB));
		thetaTs=thetaBs-asin(r/sqrt((xB-xS)*(xB-xS)+(yB-yS)*(yB-yS)));
		divider=sin(thetaT+thetaTs);
		//if(divider!=0)
		{
			t=(xS*sin(thetaTs)+yS*cos(thetaTs))/divider;
		}
		xT=t*cos(thetaT);
		yT=t*sin(thetaT);
		xT=-xT;
	}

	cv::Point2f ret(xT+robot.x,yT+robot.y);
	turningPoint=ret;
	return true;
}

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
			int x0=sqrt(cvRound(p[2])*cvRound(p[2])-(float)((cvRound(p[1])-y)*(cvRound(p[1])-y)));
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

float getTime(const cv::Point2f &p1,const cv::Point2f &d1,const cv::Point2f &p2,const cv::Point2f &d2)
{
    float time = p2.x*d2.y-p2.y*d2.x;
    time -= p1.x*d2.y-p1.y*d2.x;
    time /= d1.x*d2.y-d2.x*d1.y;
    return time;
}

float length(const cv::Point2f &p)
{
    return sqrt(pow(p.x,2)+pow(p.y,2));
}

int getAcc(int k,float dist)
{
    int ret=0;
    if(dist>k*abs(k)/2.0*DELTA_V*DELTA_T/float(1e6))
    {
        ret=1;
    }
    else if(dist<k*abs(k)/2.0*DELTA_V*DELTA_T/float(1e6))
    {
        ret=-1;
    }
    else
    {
        if(k>0)
        {
            ret=-1;
        }
        else if(k<0)
        {
            ret=1;
        }
    }
    if(abs(k+ret)<=MAX_SPEED_LEVEL)
        return ret;
    else
        return 0;
}
