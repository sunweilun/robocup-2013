#include "WorldMap.h"

WorldMap::WorldMap()
{
    wMap = cvCreateImage(cvSize(MAP_LEN,MAP_LEN),IPL_DEPTH_8U,3);
    cvRectangle(wMap,cvPoint(0,0),cvPoint(wMap->width-1,wMap->height-1),CV_RGB(0,255,0),-1);
}

IplImage* WorldMap::getMap()
{
    IplImage *newMap = cvCreateImage(cvGetSize(wMap),IPL_DEPTH_8U,3);
    memcpy(newMap->imageData,wMap->imageData,3*MAP_LEN*MAP_LEN);
    return newMap;
}



void WorldMap::loadCamParms_l(const char* fileName)
{
    FILE* file = fopen(fileName,"r");
    for(int i=0;i<8;i++)
        fscanf(file,"%f\n",&camParms_l[i]);
    fclose(file);
}

void WorldMap::loadCamParms_r(const char* fileName)
{

    FILE* file = fopen(fileName,"r");
    for(int i=0;i<8;i++)
        fscanf(file,"%f\n",&camParms_r[i]);
    fclose(file);
}

void WorldMap::setParent(Robot* robot)
{
    this->robot = robot;
}

cv::Point2f WorldMap::coord_robot2screen(const cv::Point2f& rCoord, bool isRight)
{
    if(isRight) {
        cv::Point2f sCoord;
        float p1, p2, p3, p4, q1, q2;
        p1 = rCoord.x * camParms_r[6] - camParms_r[0];
        p2 = rCoord.x * camParms_r[7] - camParms_r[1];
        p3 = rCoord.y * camParms_r[6] - camParms_r[3];
        p4 = rCoord.y * camParms_r[7] - camParms_r[4];
        q1 = camParms_r[2] - rCoord.x;
        q2 = camParms_r[5] - rCoord.y;
        sCoord.y = (q2*p1-q1*p3)/(p1*p4-p2*p3);
        sCoord.x = (q1-p2*sCoord.y)/p1;
        return sCoord;
    }
    else {
        cv::Point trans_rCoord(rCoord.y,-rCoord.x);
        cv::Point2f sCoord;
        float p1, p2, p3, p4, q1, q2;
        p1 = rCoord.x * camParms_l[6] - camParms_l[0];
        p2 = rCoord.x * camParms_l[7] - camParms_l[1];
        p3 = rCoord.y * camParms_l[6] - camParms_l[3];
        p4 = rCoord.y * camParms_l[7] - camParms_l[4];
        q1 = camParms_l[2] - trans_rCoord.x;
        q2 = camParms_l[5] - trans_rCoord.y;
        sCoord.y = (q2*p1-q1*p3)/(p1*p4-p2*p3);
        sCoord.x = (q1-p2*sCoord.y)/p1;
        return sCoord;
    }
}

CvRect WorldMap::getMap_bbox()
{
    if(!wMap)
        return cvRect(0,0,0,0);
    int xmin = MAP_LEN,xmax = -1,ymin = MAP_LEN,ymax = -1;
    unsigned char* data = (unsigned char*)wMap->imageData;
    for(int x=0;x<wMap->width;x++)
    {
        for(int y=0;y<wMap->height;y++)
        {
            int idx = 3*(y*wMap->width+x);
            if(data[idx]==255 && data[idx+1]==255 && data[idx+2]==255)
            {
                xmin = x<xmin?x:xmin;
                xmax = x>xmax?x:xmax;
                ymin = y<ymin?y:ymin;
                ymax = y>ymax?y:ymax;
            }
        }
    }
    return cvRect(xmin,ymin,xmax-xmin,ymax-ymin);
}

cv::Point2f WorldMap::coord_screen2robot(const cv::Point2f& sCoord, bool isRight)
{

    if (isRight) {
        cv::Point2f rCoord;
        rCoord.x = (camParms_r[0] * sCoord.x + camParms_r[1] * sCoord.y + camParms_r[2]) /
         (camParms_r[6] * sCoord.x + camParms_r[7] * sCoord.y + 1);
        rCoord.y = (camParms_r[3] * sCoord.x + camParms_r[4] * sCoord.y + camParms_r[5]) /
         (camParms_r[6] * sCoord.x + camParms_r[7] * sCoord.y + 1);
        return rCoord;
    }
    else {
        cv::Point2f rCoord;
        rCoord.x = (camParms_l[0] * sCoord.x + camParms_l[1] * sCoord.y + camParms_l[2]) /
         (camParms_l[6] * sCoord.x + camParms_l[7] * sCoord.y + 1);
        rCoord.y = (camParms_l[3] * sCoord.x + camParms_l[4] * sCoord.y + camParms_l[5]) /
         (camParms_l[6] * sCoord.x + camParms_l[7] * sCoord.y + 1);
        return cv::Point2f(-rCoord.y,rCoord.x);
    }
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
    cv::Point2f temp(wCoord.x-robot->x,wCoord.y-robot->y);
    rCoord.x = temp.x*cos(robot->ori)-temp.y*sin(robot->ori);
    rCoord.y = temp.x*sin(robot->ori)+temp.y*cos(robot->ori);
    return rCoord;
}

bool WorldMap::legal(const IplImage *img,int x,int y)
{
    bool xLegal = (x>=0 && x<img->width);
    bool yLegal = (y>=INVAlID_CAM_HEIGHT && y<img->height);
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
  cvReleaseImage(&eli_img);
    return lines;
}
IplImage* WorldMap::getGate(const IplImage* hsv_img){
    ip.setBound(BLUE_BOUND);
    IplImage* blue = cvCreateImage(cvGetSize(hsv_img), IPL_DEPTH_32F, 3);
    bool hasGate = ip.getOnlyBlue(hsv_img, blue);

    if(!hasGate){
        IplImage* gate = cvCreateImage(cvGetSize(hsv_img), IPL_DEPTH_8U, 1);
        uchar* gate_data =  (uchar*)gate->imageData;
        for(int i = 0; i <  gate->width*gate->height; i++){
                gate_data[i] = 0;
        }
        cvReleaseImage(&blue);
 //       cvNamedWindow("onlyblue",  CV_WINDOW_AUTOSIZE);
 //       cvShowImage("onlyblue", gate);
        return gate;
    }
    else{
        int* bound = ip.scanUp(blue);
        IplImage* gate = ip.getBound(bound);
//        cvNamedWindow("onlyblue",  CV_WINDOW_AUTOSIZE);
//        cvShowImage("onlyblue", gate);
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
    IplImage* gate = getGate(hsv_img);
    IplImage* lines = getField(hsv_img);

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

#if DEBUG_MAP
    cvNamedWindow("Map");
    cvShowImage("Map",wMap);
    cvWaitKey();
    cvDestroyWindow("Map");
#endif
    cvReleaseImage(&hsv_img);
    cvReleaseImage(&lines);
    cvReleaseImage(&gate);
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
