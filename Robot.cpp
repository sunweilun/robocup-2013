#include "Robot.h"
#include "motor_uplayer.h"
#include "getPhoto.h"

Robot::Robot()
{
    worldMap.setParent(this);
    ptInit();
    motor_init();
    imgCounter = 0;
    worldMap.loadCamParms(CAM_PARMS_PATH);
    image = NULL;
    ballLocated = false;
    ownGoalLocated = false;
    oppGoalLocated = false;
    radar = false;
    setCoord(0,0,0);
}

void Robot::radarOn()
{
    radar = true;
    cvNamedWindow(RADAR_WND_NAME);
    cvMoveWindow(RADAR_WND_NAME,0,0);
    updateRadar();
}

void Robot::radarOff()
{
    radar = false;
    cvDestroyWindow(RADAR_WND_NAME);
}

void Robot::setCoord(float x,float y,float ori)
{
    this->x = x;
    this->y = y;
    this->ori = ori;
}

void Robot::turnLeft(float angle)
{
    ori -= angle*M_PI/180;
    ori = ori>2*M_PI?ori-2*M_PI:ori;
    ori = ori<0?ori+2*M_PI:ori;
    motor_turnLeft(angle*M_PI/180);
    updateRadar();
}

void Robot::turnRight(float angle)
{
    ori += angle*M_PI/180;
    ori = ori>2*M_PI?ori-2*M_PI:ori;
    ori = ori<0?ori+2*M_PI:ori;
    motor_turnRight(angle*M_PI/180);
    updateRadar();
}

void Robot::drawMap() {
    printf("drawmap in!\n");
   //getImage();

    for(int i=0;i<12;i++)
    {
        getImage();
        printf("draw start\n");
        worldMap.updateMap(image);
        printf("draw finish\n");
        turnRight(30);
    }

    //getImage();
    //worldMap.updateMap(image);
    //turnRight(180);
    //getImage();
    //worldMap.updateMap(image);
    //moveForward(200,30);
    //getImage();
    worldMap.updateMap(image);
    worldMap.saveMap("result.png");
}

void Robot::getImage()
{
    usleep(SLEEPTIME_BEFORE_PHOTO); // sleep until the camera is still
    getPhoto();
    char dp[] = DATA_PATH;
    char fn[1024];
    sprintf(fn,"%s%d.dat",dp,imgCounter);
    printf("file = %s\n", fn);
    if(image)
        cvReleaseImage(&image);
    image = loadDatImage(fn);
    imgCounter++;
}

void Robot::moveForward(float dist,float max_speed)
{
    x += dist*sin(ori);
    y += dist*cos(ori);
    goWithDistance(dist,max_speed);
    updateRadar();
}

void Robot::updateRadar()
{
    if(!radar)
        return;
    IplImage* wMap = worldMap.getMap();
    int x = this->x+(MAP_LEN>>1);
    int y = (MAP_LEN>>1)+1-this->y;
    cvCircle(wMap,cvPoint(x,y),ROBOT_RADIUS,CV_RGB(0,0,0),-1);
    cvLine(wMap,cvPoint(x,y),cvPoint(x+ROBOT_RADIUS*sin(ori),y-ROBOT_RADIUS*cos(ori)),CV_RGB(0,255,255),3);
    if(ballLocated)
        cvCircle(wMap,cvPoint(x,y),BALL_RADIUS,CV_RGB(255,0,0),-1);
    cvShowImage(RADAR_WND_NAME,wMap);
    cvWaitKey(100);
    cvReleaseImage(&wMap);
}

bool Robot::locateBall()
{
    if(!image)
        return false;
    ip.setBound(BALL_BOUND);
    std::vector<cv::Point3f> circles = ip.extractCircles(image);
    if(circles.size()<=0)
    {
        return false;
    }
    ballLocated = true;
    cv::Point2f rCoord;
    rCoord = worldMap.coord_screen2robot(cv::Point2f(circles[0].x,circles[0].y+circles[0].z));
    ball_coord = worldMap.coord_robot2world(rCoord);
    updateRadar();
    return true;
}

void Robot::findBall()
{
    getImage();
    while(!locateBall())
    {
        turnRight(FIND_BALL_ANGLE);
        getImage();
    }
}

Robot::~Robot()
{
    if(image)
        cvReleaseImage(&image);
}
