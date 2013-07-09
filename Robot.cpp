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
}

void Robot::turnRight(float angle)
{
    ori += angle*M_PI/180;
    ori = ori>2*M_PI?ori-2*M_PI:ori;
    ori = ori<0?ori+2*M_PI:ori;
    motor_turnRight(angle*M_PI/180);
}

void Robot::drawMap() {
    printf("drawmap in!\n");
    /*for(int i=0;i<12;i++)
    {
        printf("for in!\n");
        getImage();
        turnRight(30);
    }*/
    getImage();
    worldMap.updateMap(image);
    turnRight(180);
    getImage();
    worldMap.updateMap(image);
    moveForward(200,30);
    getImage();
    worldMap.updateMap(image);
}

void Robot::getImage()
{
    usleep(1000000);
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
}

Robot::~Robot()
{
    if(image)
        cvReleaseImage(&image);
}
