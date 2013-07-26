#include "BallTracker.h"

int checkColorThreshold(float* c)
{
	if(c[0]<BALL_MOVING_H_UB && c[0]>BALL_MOVING_H_LB && c[1]<BALL_MOVING_S_UB && c[1]>BALL_MOVING_S_LB  && c[2]<BALL_MOVING_V_UB && c[2]>BALL_MOVING_V_LB )
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
	//printf("images.size=%d after push.\n",images.size());
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

int BallTracker::popBackFrame(int numOfFrame)
{
	int n=0;
	for(int i=0;i<numOfFrame;++i)
	{
		cvReleaseImage(&(images.back()));
		images.pop_back();
		pos.pop_back();
		pos_scr.pop_back();
		++n;
	}
	return n;
}

void BallTracker::scr2wld(int frameId)
{
    cv::Point2f scrPos(pos_scr[frameId].x,pos_scr[frameId].y);
	cv::Point2f roboPos=robot->worldMap.coord_screen2robot(scrPos,false);
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
        //printf("if_1\n");
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
            pos_scr[frameId]=ballDetect(images[frameId]);
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
		return 1;
	}
	if(frameId==0)
	{
	//printf("if_2\n");
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
		pos_scr[frameId]=ballDetect(images[frameId]);
			//printf("if_2_1\n");
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
			//printf("if_2_2\n");
		if(pos_scr[frameId+1]==cv::Point3f(-1,-1,-1))
			pos_scr[frameId+1]=ballDetect(images[frameId+1]);
			//printf("if_2_3\n");
		if(pos_scr[frameId+1]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId+1);
			//printf("if_2_4\n");
		return 2;
	}
	if(images.size()-1==frameId)
	{
	//printf("if_3\n");
	//printf("pos_scr.size=%d images.size=%d frameId=%d\n",pos_scr.size(),images.size(),frameId);
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
		pos_scr[frameId]=ballDetect(images[frameId]);
			//printf("if_3_1\n");
		if(pos_scr[frameId]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId);
			//printf("if_3_2\n");
		if(pos_scr[frameId-1]==cv::Point3f(-1,-1,-1))
			pos_scr[frameId-1]=ballDetect(images[frameId-1]);
			//printf("if_3_3\n");
		if(pos_scr[frameId-1]==cv::Point3f(-1,-1,-1))
			return -1;
        scr2wld(frameId-1);
			//printf("if_3_4\n");
		return 2;
	}
	return 0;
}

cv::Point3f BallTracker::ballDetect(const IplImage* image1)
{
	//struct timespec ts,te;
    //clock_gettime(CLOCK_REALTIME,&ts);
    if(image1 == NULL)
        exit(1);
	IplImage* tmp1=cvCreateImage(cvGetSize(image1),IPL_DEPTH_32F,3);
	IplImage* bina=cvCreateImage(cvGetSize(image1),IPL_DEPTH_8U,1);
	cvConvertScale(image1,tmp1,1.0/255.0,0);
	cvCvtColor(tmp1,tmp1,CV_BGR2HSV);

    //clock_gettime(CLOCK_REALTIME,&te);
    //printf("!!---1 time = %f\n",double(te.tv_nsec-ts.tv_nsec)/(1e9));
	//clock_gettime(CLOCK_REALTIME,&ts);
    float* dat1=(float*)tmp1->imageData;

	cvSet(bina,cvScalar(0));
	int sum=0;
	for(int i=0;i<240;++i)
	{
		for(int j=0;j<320;++j)
		{
			if(checkColorThreshold(dat1+(i*320+j)*3)==0)
				continue;
			bina->imageData[i*320+j]=255;
			++sum;
		}
	}
    //cvNamedWindow("tempImage1");
	//if(images.size()>1)
      //cvShowImage("tempImage1",bina);
	//cvWaitKey(10);

    //clock_gettime(CLOCK_REALTIME,&te);
    //printf("!!---2 time = %f\n",double(te.tv_nsec-ts.tv_nsec)/(1e9));
	//clock_gettime(CLOCK_REALTIME,&ts);
    char* b=bina->imageData;
	int id[240*320];
	memset(id,0,240*320*sizeof(int));
	int newId=1;

	deque<int> waitQueue;
	vector<Area> areas;

	for(int i=0;i<240;++i)
	{
		for(int j=0;j<320;++j)
		{
			if(b[i*320+j]==0)
				continue;
			if(id[i*320+j]>0)
				continue;
			waitQueue.push_back(i*320+j);
			int curPos=waitQueue.front();
			id[i*320+j]=newId;
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
				if(curPos-320>0)
				{
					if((unsigned char)b[curPos-320]==255 && id[curPos-320]==0)
					{
						id[curPos-320]=newId;
						++curArea;
						waitQueue.push_back(curPos-320);
					}
				}
				if(curPos%320+1<320)
				{
					if((unsigned char)b[curPos+1]==255 && id[curPos+1]==0)
					{
						id[curPos+1]=newId;
						++curArea;
						waitQueue.push_back(curPos+1);
					}
				}
				if(curPos+320<240*320)
				{
					if((unsigned char)b[curPos+320]==255 && id[curPos+320]==0)
					{
						id[curPos+320]=newId;
						++curArea;
						waitQueue.push_back(curPos+320);
					}
				}
				waitQueue.pop_front();
			}
			if(curArea<100)
                continue;
			xx/=curArea;
			yy/=curArea;
			if(xmax-xmin<10 || ymax-ymin<10)
			continue;
			areas.push_back(Area(newId,xx,yy,xmin,xmax,ymin,ymax,curArea));
			//printf("id=%d centerPoint=(%d,%d) area=%d\n",newId,xx,yy,curArea);
			++newId;//warning: Id not good
		}
	}
	cvReleaseImage(&bina);
	cvReleaseImage(&tmp1);

    //clock_gettime(CLOCK_REALTIME,&te);
    //printf("!!---3 time = %f\n",double(te.tv_nsec-ts.tv_nsec)/(1e9));
	//clock_gettime(CLOCK_REALTIME,&ts);
	if(areas.size()>0)
	{
	//printf("areas.size=%d\n",areas.size());
		    CvRect bBox=robot->worldMap.getMap_bbox();
		int mx=0;
		for(int i=0;i<areas.size();++i)
		{
            if(areas[mx].area>=areas[i].area)
                continue;
		    cv::Point2f scrPos(areas[i].x,areas[i].y);
		    cv::Point2f roboPos=robot->worldMap.coord_screen2robot(scrPos,false);
		    cv::Point2f worldPos=robot->worldMap.coord_robot2world(roboPos);
		    cv::Point2f imgPos=robot->world2image(worldPos);

		    if(!(imgPos.x>bBox.x && imgPos.x<bBox.x+bBox.width && imgPos.y>bBox.y && imgPos.y<bBox.y+bBox.height))
		    {
		    //printf("areas[%d] out! imgPos=(%f,%f) bBox=(%d,%d)(%d,%d)\n",i,imgPos.x,imgPos.y,bBox.x,bBox.y,bBox.width,bBox.height);
                continue;
                }
			{
            //printf("area[%d] cur max\n",i);
				mx=i;
                }
		}
		//printf("best area found. mx=%d pos=(%f,%f) r=%f\n",mx,areas[mx].x,areas[mx].y,(areas[mx].ymax-areas[mx].ymin)/2.0);
		    cv::Point2f scrPos(areas[mx].x,areas[mx].y);
		    cv::Point2f roboPos=robot->worldMap.coord_screen2robot(scrPos,false);
		    cv::Point2f worldPos=robot->worldMap.coord_robot2world(roboPos);
		    cv::Point2f imgPos=robot->world2image(worldPos);
		//printf("best area roboPos=(%f,%f) worldPos=(%f,%f) imgPos=(%f,%f)\n",roboPos.x,roboPos.y,worldPos.x,worldPos.y,imgPos.x,imgPos.y);


    //clock_gettime(CLOCK_REALTIME,&te);
    //printf("!!----4 time = %f\n",double(te.tv_nsec-ts.tv_nsec)/(1e9));
		return cv::Point3f(areas[mx].x,areas[mx].y,(areas[mx].ymax-areas[mx].ymin)/2);
	}
	return cv::Point3f(-1,-1,-1);
}
