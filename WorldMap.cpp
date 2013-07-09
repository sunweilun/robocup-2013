#include "WorldMap.h"

WorldMap::WorldMap()
{
    wMap = cvCreateImage(cvSize(MAP_LEN,MAP_LEN),IPL_DEPTH_8U,3);
    cvRectangle(wMap,cvPoint(0,0),cvPoint(wMap->width-1,wMap->height-1),cvScalar(0,255,0));
}

void WorldMap::loadCamParms(const char* fileName)
{
        FILE* file = fopen(fileName,"r");
        for(int i=0;i<8;i++)
            fscanf(file,"%f\n",&camParms[i]);
        fclose(file);
}

void WorldMap::setParent(Robot* robot)
{
    this->robot = robot;
}

cv::Point2f WorldMap::coord_robot2screen(const cv::Point2f& rCoord)
{
    cv::Point2f sCoord;
    float p1, p2, p3, p4, q1, q2;
    p1 = rCoord.x * camParms[6] - camParms[0];
    p2 = rCoord.x * camParms[7] - camParms[1];
    p3 = rCoord.y * camParms[6] - camParms[3];
    p4 = rCoord.y * camParms[7] - camParms[4];
    q1 = camParms[2] - rCoord.x;
    q2 = camParms[5] - rCoord.y;
    sCoord.y = (q2*p1-q1*p3)/(p1*p4-p2*p3);
    sCoord.x = (q1-p2*sCoord.y)/p1;
    return sCoord;
}

cv::Point2f WorldMap::coord_robot2world(const cv::Point2f& rCoord)
{
    cv::Point2f wCoord;
    wCoord.x = robot->x+rCoord.x*cos(robot->ori)+rCoord.y*sin(robot->ori);
    wCoord.y = robot->y-rCoord.x*sin(robot->ori)+rCoord.y*cos(robot->ori);
    return wCoord;
}

cv::Point2f WorldMap::coord_world2robot(const cv::Point2f& wCoord)
{
    cv::Point2f rCoord;
    rCoord.x = -robot->x+wCoord.x*cos(robot->ori)-wCoord.y*sin(robot->ori);
    rCoord.y = -robot->y+wCoord.x*sin(robot->ori)+wCoord.y*cos(robot->ori);
    return rCoord;
}

bool WorldMap::legal(const IplImage *img,int x,int y)
{
    bool xLegal = (x>=0 && x<img->width);
    bool yLegal = (y>=0 && y<img->height);
    return xLegal && yLegal && img->imageData[y*img->width+x]!=0;
}

IplImage* WorldMap::getLines(const IplImage* hsv_img)
{
    ip.setBound(GRASS_BOUND);
    IplImage* eli_img = ip.eliminateBackground(hsv_img);
    ip.setBound(LINE_BOUND);
    IplImage* lines = ip.extractColorBlocks(eli_img);
    cvReleaseImage(&eli_img);
    return lines;
}
IplImage* WorldMap::getField(const IplImage* hsv_img){
    ip.setBound(GRASS_BOUND);
    IplImage* eli_img = ip.deleteNoise(hsv_img);
  //  cvNamedWindow("noise",  CV_WINDOW_AUTOSIZE);
    //cvShowImage("noise", eli_img);
    ip.setBound(LINE_BOUND);
    IplImage* lines = ip.extractColorBlocks(eli_img);
 //   cvNamedWindow("pale",  CV_WINDOW_AUTOSIZE);
  //  cvShowImage("pale", lines);
    return lines;
}
IplImage* WorldMap::getGate(const IplImage* hsv_img){
    ip.setBound(BLUE_BOUND);
    IplImage* blue = cvCreateImage(cvGetSize(hsv_img), IPL_DEPTH_32F, 3);
    bool hasGate = ip.getOnlyBlue(hsv_img, blue);
 //        cvNamedWindow("onlyblue",  CV_WINDOW_AUTOSIZE);
  //      cvShowImage("onlyblue", blue);

    if(!hasGate){
        IplImage* black = cvCreateImage(cvGetSize(hsv_img), IPL_DEPTH_8U, 1);
        cvReleaseImage(&blue);
        return black;
    }
    else{
        int* bound = ip.scanUp(blue);
        IplImage* gate = ip.getBound(bound);
        cvReleaseImage(&blue);
  //       cvNamedWindow("gate",  CV_WINDOW_AUTOSIZE);
   //     cvShowImage("gate", gate);
        return gate;
    }
}

void WorldMap::updateMap(const IplImage *img)
{
    IplImage* hsv_img = get_hsv(img);
    //IplImage *lines = getLines(hsv_img);
    //sk add
    IplImage* lines = getField(hsv_img);
    IplImage* gate = getGate(hsv_img);

    for(int i=0;i<MAP_LEN*MAP_LEN;i++)
    {
        int map_x = i%MAP_LEN;
        int map_y = i/MAP_LEN;

        cv::Point2f wc,rc,sc;
        wc.x = map_x-(MAP_LEN>>1);
        wc.y = (MAP_LEN>>1)+1-map_y;
        rc = coord_world2robot(wc);
        sc = coord_robot2screen(rc);

        int left,right,up,down;
        left = sc.x;
        right = sc.x+1;
        down = sc.y;
        up = sc.y+1;

        if(legal(lines,left,down)||legal(lines,left,up)||legal(lines,right,down)||legal(lines,right,up))
        {
            memset(wMap->imageData+3*i,255,3);
        }
        if(legal(gate, left, down) || legal(gate, left, up) || legal(gate, right, down) || legal(gate, right, up)){
                ((uchar*)wMap->imageData)[3*i+0] = 255;
                ((uchar*)wMap->imageData)[3*i+1] = 0;
                ((uchar*)wMap->imageData)[3*i+2] = 0;
        }
    }

#ifdef DEBUG_MAP
    cvNamedWindow("Map");
    cvShowImage("Map",wMap);
    cvWaitKey();
    cvDestroyWindow("Map");
#endif
    cvReleaseImage(&hsv_img);
    cvReleaseImage(&lines);
}

void WorldMap::showMap(const char* wndName)
{
    cvShowImage(wndName,wMap);
}

void WorldMap::saveMap(const char* fn)
{
    cvSaveImage(fn,wMap);
}

WorldMap::~WorldMap()
{
    cvReleaseImage(&wMap);
}
