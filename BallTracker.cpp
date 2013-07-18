#include "BallTracker.h"

int checkColorThreshold(float* c)
{
	if(abs(c[0]-BALL_COLOR_HSV.val[0])<rg[0] && abs(c[1]-BALL_COLOR_HSV.val[1])<rg[1] && abs(c[2]-BALL_COLOR_HSV.val[2])<rg[2])
		return 1;
	else
		return 0;
}

BallTracker::~BallTracker()
{
    popFrame(images.size());
}

int BallTracker::pushFrame(const IplImage* image, double t)
{
	IplImage* img=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,3);
	cvCopyImage(image,img);
	images.push_back(img);
	pos.push_back(cv::Point3f(-1,-1,t));
	pos_scr.push_back(cv::Point3f(-1,-1,-1));
	return 0;
}

int BallTracker::popFrame(int numOfFrame)
{
	int n=0;
	for(int i=0;i<numOfFrame;++i)
	{
		cvReleaseImage(&(images.front()));
		images.pop_front();
		pos.pop_front();
		pos_scr.pop_front();
		++n;
	}
	return n;
}

void BallTracker::scr2wld(int frameId)
{
    cv::Point2f scrPos(pos_scr[frameId].x,pos_scr[frameId].y-pos_scr[frameId].z);
	cv::Point2f roboPos=robot->worldMap.coord_screen2robot(scrPos);
	cv::Point2f worldPos=robot->worldMap.coord_robot2world(roboPos);
	pos[frameId].x=worldPos.x;
	pos[frameId].y=worldPos.y;
    return;
}
int BallTracker::processFrame(int frameId)
{
	if(images.size()==0)
		return -1;
	if(images.size()==1)
	{
		pos_scr[frameId]=ballDetect(images[frameId]);
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
		return 1;
	}
	if(frameId==0)
	{
		pos_scr[frameId]=ballDetect(images[frameId]);
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
		if(pos_scr[frameId+1]==cv::Point3f(-1,-1,-1))
			pos_scr[frameId+1]=ballDetect(images[frameId+1]);
		if(pos_scr[frameId+1]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId+1);
		return 2;
	}
	//if(images.size()-1==frameId)
	{
		pos_scr[frameId]=ballDetect(images[frameId]);
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
		if(pos_scr[frameId-1]==cv::Point3f(-1,-1,-1))
			pos_scr[frameId-1]=ballDetect(images[frameId-1]);
		if(pos_scr[frameId-1]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId-1);
		return 2;
	}
	return 0;
}

cv::Point3f BallTracker::ballDetect(const IplImage* image1)
{
    if(image1 == NULL)
        exit(1);
	IplImage* tmp1=cvCreateImage(cvGetSize(image1),IPL_DEPTH_32F,3);
	IplImage* bina=cvCreateImage(cvGetSize(image1),IPL_DEPTH_8U,1);
	cvConvertScale(image1,tmp1,1.0/255.0,0);
	cvCvtColor(tmp1,tmp1,CV_BGR2HSV);

	float* dat1=(float*)tmp1->imageData;

	cvSet(bina,cvScalar(0));
	int sum=0;
	for(int i=0;i<height;++i)
	{
		for(int j=0;j<width;++j)
		{
			if(checkColorThreshold(dat1+(i*width+j)*3)==0)
				continue;
			bina->imageData[i*width+j]=255;
			++sum;
		}
	}
	char* b=bina->imageData;
	int id[240*320];
	memset(id,0,240*320*sizeof(int));
	int newId=1;
	deque<int> waitQueue;
	vector<Area> areas;
	for(int i=0;i<height;++i)
	{
		for(int j=0;j<width;++j)
		{
			if(b[i*width+j]==0)
				continue;
			if(id[i*width+j]>0)
				continue;
			waitQueue.push_back(i*width+j);
			int curPos=waitQueue.front();
			id[i*width+j]=newId;
			int curArea=1;
			int xx=0;
			int yy=0;
			int xmin=curPos%320;
			int ymin=curPos/320;
			int xmax=curPos%320;
			int ymax=curPos/320;
			while(waitQueue.size()>0)
			{
				curPos=waitQueue.front();
				xx+=curPos%320;
				yy+=curPos/320;
				if(curPos%320<xmin)
					xmin=curPos%320;
				if(curPos%320>xmax)
					xmax=curPos%320;
				if(curPos/320<ymin)
					ymin=curPos/320;
				if(curPos/320>ymax)
					ymax=curPos/320;
				if(curPos%320-1>0)
				{
					if((unsigned char)b[curPos-1]==255 && id[curPos-1]==0)
					{
						id[curPos-1]=newId;
						++curArea;
						waitQueue.push_back(curPos-1);
					}
				}
				if(curPos-width>0)
				{
					if((unsigned char)b[curPos-width]==255 && id[curPos-width]==0)
					{
						id[curPos-width]=newId;
						++curArea;
						waitQueue.push_back(curPos-width);
					}
				}
				if(curPos%320+1<width)
				{
					if((unsigned char)b[curPos+1]==255 && id[curPos+1]==0)
					{
						id[curPos+1]=newId;
						++curArea;
						waitQueue.push_back(curPos+1);
					}
				}
				if(curPos+width<240*320)
				{
					if((unsigned char)b[curPos+width]==255 && id[curPos+width]==0)
					{
						id[curPos+width]=newId;
						++curArea;
						waitQueue.push_back(curPos+width);
					}
				}
				waitQueue.pop_front();
			}
			xx/=curArea;
			yy/=curArea;
			areas.push_back(Area(newId,xx,yy,xmin,xmax,ymin,ymax,curArea));
			//printf("id=%d centerPoint=(%d,%d) area=%d\n",newId,xx,yy,curArea);
			++newId;
		}
	}
	cvReleaseImage(&bina);
	cvReleaseImage(&tmp1);
	if(areas.size()>0)
	{
		int mx=0;
		for(int i=0;i<areas.size();++i)
		{
		    cv::Point2f scrPos(areas[i].x,areas[i].y-(areas[mx].ymax-areas[mx].ymin)/2);
		    cv::Point2f roboPos=robot->worldMap.coord_screen2robot(scrPos);
		    cv::Point2f worldPos=robot->worldMap.coord_robot2world(roboPos);
		    cv::Point2f imgPos=robot->world2image(worldPos);
		    CvRect bBox=robot->worldMap.getMap_bbox();

		    if(!(imgPos.x>bBox.x && imgPos.x<bBox.x+bBox.width && imgPos.y>bBox.y && imgPos.y<bBox.y+bBox.height))
                continue;
			if(areas[mx].area<areas[i].area)
				mx=i;
		}
		return cv::Point3f(areas[mx].x,areas[mx].y,(areas[mx].ymax-areas[mx].ymin)/2);
	}
	return cv::Point3f(-1,-1,-1);
}
