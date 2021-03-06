#include "Robot.h"
#include "motor_uplayer.h"
#include <math.h>

Robot::Robot()
{
    worldMap.setParent(this);
    ballTracker.setParent(this);
    ptInit();
    motor_init();
    imgCounter = 0;
    worldMap.loadCamParms_l(CAM_PARMS_PATH_LEFT);
    worldMap.loadCamParms_r(CAM_PARMS_PATH_RIGHT);
    image_l = NULL;
    image_r = NULL;
    ballLocated = false;
    ownGoalLocated = false;
    oppGoalLocated = false;
    radar = false;
    setCoord(0,-100,0);
}

void Robot::radarOn()
{
    radar = true;
    cvNamedWindow(RADAR_WND_NAME);
    cvMoveWindow(RADAR_WND_NAME,0,0);
    updateRadar();
}

cv::Point2f Robot::world2image(const cv::Point2f& coord)
{
    return cv::Point2f(coord.x+(MAP_LEN>>1),(MAP_LEN>>1)+1-coord.y);
}

cv::Point2f Robot::image2world(const cv::Point2f& coord)
{
    return cv::Point2f(coord.x-(MAP_LEN>>1),(MAP_LEN>>1)+1-coord.y);
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
    motor_turnLeft((angle+TURN_LEFT_OFFSET*angle/30.0)*M_PI/180);
    updateRadar();
}

void Robot::turnRight(float angle)
{
    ori += angle*M_PI/180;
    ori = ori>2*M_PI?ori-2*M_PI:ori;
    ori = ori<0?ori+2*M_PI:ori;
    motor_turnRight((angle+TURN_RIGHT_OFFSET*angle/30.0)*M_PI/180);
    updateRadar();
}

void Robot::drawMap()
{
    getImage();
    char filename[20];
    strcpy(filename, "0.png");
    for(int i=0; i<12; i++)
    {
        getImage();
        filename[0] = (char)(i+'0');
        filename[5] = '\0';
        cvSaveImage(filename, image_r);
        adjustWorldCoordinate(image_r,0);
        cvWaitKey(100);
        worldMap.updateMap(image_r);
        locateOwnGate();
        turnRight(30);
    }

    getImage();
    adjustWorldCoordinate(image_r,2);
    adjustWorldCoordinate(image_r,1);
    rotateTo(cv::Point2f(0,1));
    /*moveForward(200, 20);
    for(int i=0;i<12;i++)
    {
        getImage();
        adjustWorldCoordinate(image_r,0);
        worldMap.updateMap(image_r);
        turnRight(30);
    }
    getImage();
    adjustWorldCoordinate(image_r,2);
    adjustWorldCoordinate(image_r,1);
    rotateTo(cv::Point2f(0,1));*/
}

void Robot::getImage()
{
    if(!image_l)
        image_l = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    if(!image_r)
        image_r = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    getPhoto2(image_l, image_r);
    cvCvtColor(image_l,image_l,CV_RGB2BGR);
    cvCvtColor(image_r,image_r,CV_RGB2BGR);
}

void Robot::moveForward(float dist,float max_speed)
{
    x += dist*sin(ori);
    y += dist*cos(ori);
    goWithDistance(dist,max_speed);
    updateRadar();
}

void Robot::moveRotate(bool isLeft, float radius, float arc)
{
    if (isLeft)
    {
	//worldmap operation
        cv::Point2f center(x,y);
        cv::Point2f vec_r(radius*cos(ori),-radius*sin(ori));
        center = center - vec_r;
        cv::Point2f new_vec_r;
        new_vec_r.x = vec_r.x*cos(arc)+vec_r.y*sin(arc);
        new_vec_r.y = -vec_r.x*sin(arc)+vec_r.y*cos(arc);
        cv::Point2f shift = new_vec_r - vec_r;
        x += shift.x;
        y += shift.y;
        ori -= arc;
        ori = ori<0?ori+2*M_PI:(ori>2*M_PI?ori-2*M_PI:ori);
	//motor operation
        int min1 = 0, min2 = 0;
        float mind = 10000;
        float radio = (radius + DIST_BETWEEN_WHEELS / 2) / (radius - DIST_BETWEEN_WHEELS / 2);
        float nowr;
        for (int i = 30; i >= 2; i--)//枚举搜索一定速度范围内最接近内外轮半径比的左右轮轮速比
            for (int j = 1; j < i; j++) {
                nowr = i *1.0 / j;
                if (myabs(nowr - radio) < mind) {
                    mind = myabs(nowr - radio);
                    min1 = i;
                    min2 = j;
                }
            }
            float t = (((arc - ARC_DELAY) * (radius + DIST_BETWEEN_WHEELS / 2) / min1)
                                + ((arc - ARC_DELAY) * (radius - DIST_BETWEEN_WHEELS / 2) / min2)) / 2;
            goWithSpeed(min2, min1, t);
    }
    else  //turn right
    {
	//vice vesa
        cv::Point2f center(x,y);
        cv::Point2f vec_r(-radius*cos(ori),radius*sin(ori));
        center = center - vec_r;
        cv::Point2f new_vec_r;
        new_vec_r.x = vec_r.x*cos(-arc)+vec_r.y*sin(-arc);
        new_vec_r.y = -vec_r.x*sin(-arc)+vec_r.y*cos(-arc);
        cv::Point2f shift = new_vec_r - vec_r;
        x += shift.x;
        y += shift.y;
        ori += arc;
        ori = ori<0?ori+2*M_PI:(ori>2*M_PI?ori-2*M_PI:ori);
	//vice vesa
        int min1 = 0, min2 = 0;
        float mind = 10000;
        float radio = (radius + DIST_BETWEEN_WHEELS / 2) / (radius - DIST_BETWEEN_WHEELS / 2);
        float nowr;
        for (int i = 30; i >= 2; i--)
            for (int j = 1; j < i; j++) {
                nowr = i *1.0 / j;
                if (myabs(nowr - radio) < mind) {
                    mind = myabs(nowr - radio);
                    min1 = i;
                    min2 = j;
                }
            }
            float t = (((arc - ARC_DELAY) * (radius + DIST_BETWEEN_WHEELS / 2) / min1)
                                + ((arc - ARC_DELAY) * (radius - DIST_BETWEEN_WHEELS / 2) / min2)) / 2;
            goWithSpeed(min1, min2, t);
    }
}


void Robot::moveTo(const cv::Point2f& wCoord,float max_speed)
{
    cv::Point2f robotPosition(x,y);
    cv::Point2f delta = wCoord - robotPosition;
    float delta_len = length(delta);
    if (delta_len <= 0)
        return;
    cv::Point2f new_dir = delta*(1/delta_len);

    rotateTo(new_dir);
    moveForward(delta_len,max_speed);
}

void Robot::rotateTo(const cv::Point2f &new_dir)
{
    cv::Point2f dir(sin(ori),cos(ori));
    float cross = dir.x*new_dir.y-dir.y*new_dir.x;
    float dot = new_dir.dot(dir);
    dot = dot>1?1:(dot<-1?-1:dot);
    float angle = acos(dot);
    if(cross<0)
        turnRight(angle*180/M_PI);
    else
        turnLeft(angle*180/M_PI);
}

void Robot::updateBallStatus()
{
    cv::Point2f& ballPosition = ball_coord;
    cv::Point2f& ballVelocity = ball_velocity;
        //to be done by zc
    struct timespec t0;
    getImage();
    clock_gettime(CLOCK_REALTIME,&t0);
    ballLocated=false;
    double temptime=double(t0.tv_sec-timeBase.tv_sec)+double(t0.tv_nsec-timeBase.tv_nsec)/(1e9);
    timeBase=t0;
    ballTracker.pushFrame(image_l,temptime);
    ballTracker.pos[0].z=0;
    //printf("t = %f\n",temptime);
    //printf("images.size=%d before processing.\n",ballTracker.images.size());
    if(ballTracker.images.size()==3)
    {
        printf("pos_scr[]=");
        for(int i=0;i<ballTracker.images.size();++i)
            printf("(%f,%f,%f)",ballTracker.pos_scr[i].x,ballTracker.pos_scr[i].y,ballTracker.pos_scr[i].z);
        printf("pos[]=");
        for(int i=0;i<ballTracker.images.size();++i)
            printf("(%f,%f,%f)",ballTracker.pos[i].x,ballTracker.pos[i].y,ballTracker.pos[i].z);
        printf("---------------------------------\n");
    }
    int ret;
    if(ballTracker.pos.size()==2)
        ret=ballTracker.processFrame(1);
    else if(ballTracker.pos.size()==1)
        ret=ballTracker.processFrame(0);
    else
    {
    if(ballTracker.images.size()>0)
     {
            cvShowImage("leftImage",ballTracker.images[ballTracker.images.size()-1]);
    cvWaitKey(10);
    }
        ballTracker.popFrame(ballTracker.images.size());
        return;
    }
    if(ret!=2)
    {
        ballLocated=false;
        //if(ballTracker.images.size()>1)
            //ballTracker.popFrame(ballTracker.images.size()-1);
        //continue;
    }
    /*if(ballTracker.images.size()>=2)
    {
        if(ballTracker.pos[1]==cv::Point3f(-1,-1,-1))
        {
            ballLocated=false;
            ballTracker.popBackFrame(1);
            return;
        }
    }
    if(ballTracker.images.size()>=1)
    {
        if(ballTracker.pos[0]==cv::Point3f(-1,-1,-1))
        {
            ballLocated=false;
            ballTracker.popFrame(1);
            return;
        }
    }*/
    for(int i=0;i<ballTracker.images.size();++i)
    {
        if(ballTracker.pos[i]==cv::Point3f(-1,-1,-1))
        {
    if(ballTracker.images.size()>0)
      {
            cvShowImage("leftImage",ballTracker.images[ballTracker.images.size()-1]);
    cvWaitKey(10);
    }
            ballTracker.popFrame(ballTracker.images.size());
            break;
        }
    }
    //printf("images.size=%d\n",ballTracker.images.size());
    if(ballTracker.images.size()!=2)
    {
    if(ballTracker.images.size()>0)
     {
            cvShowImage("leftImage",ballTracker.images[ballTracker.images.size()-1]);
    cvWaitKey(10);
    }
        return;
    }
    if(ret!=2)
    {
    if(ballTracker.images.size()>0)
    {
            cvShowImage("leftImage",ballTracker.images[ballTracker.images.size()-1]);
    cvWaitKey(10);
    }
        ballTracker.popFrame(1);
        return;
    }
    //if(ballTracker.pos[1].z<ballTracker.pos[0].z)
        //return;
    ballPosition.x=ballTracker.pos[1].x;
    ballPosition.y=ballTracker.pos[1].y;
    ballVelocity.x=(ballTracker.pos[1].x-ballTracker.pos[0].x)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    ballVelocity.y=(ballTracker.pos[1].y-ballTracker.pos[0].y)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    ballLocated=true;
    //printf("ball: world pos=(%f,%f) v=(%f,%f)\n",ballTracker.pos[1].x,ballTracker.pos[1].y,ballVelocity.x,ballVelocity.y);
    //printf("showing\n");

    cvCircle(ballTracker.images[1],cvPoint(ballTracker.pos_scr[1].x,ballTracker.pos_scr[1].y),ballTracker.pos_scr[1].z,CV_RGB(255,255,0),2);
    cvNamedWindow("leftImage");
    cvMoveWindow("leftImage",512,0);
    cvShowImage("leftImage",ballTracker.images[1]);
    cvWaitKey(10);
    //cvDestroyWindow("leftImage");
    //cvNamedWindow("image_l");
    //cvMoveWindow("image_l",512,512);
    //cvShowImage("image_l",image_l);
    //cvWaitKey(10);
    //cvDestroyWindow("image_l");
    //if(ballTracker.images.size()>1)
    {
       // ballLocated=false;
        //ballTracker.popFrame(ballTracker.images.size()-1);
        //continue;
    }
    ballTracker.popFrame(1);
    return;
}

void* keeperMotionThread(void* params)
{
    void ** paramsList = (void**)params;
    float &targetDist = *((float*) paramsList[0]);
    float &moveDist = *((float*) paramsList[1]);
    int &v_level= *((int*) paramsList[2]);
    Robot &robot= *((Robot*) paramsList[3]);
    bool &kmt_abort = *((bool*) paramsList[4]);
    bool &rotating = *((bool*) paramsList[5]);
    while(!robot.abort)
    {
        moveDist += v_level*DELTA_V*DELTA_T/float(1e6);
        int acc = getAcc(v_level,targetDist-moveDist);
        v_level += acc;
        cv::Point2f tar_dir(robot.ownGoal_frontDir.y,-robot.ownGoal_frontDir.x);
        if(v_level==0 && acc==0)
        {
            cv::Point2f ori_dir(sin(robot.ori),cos(robot.ori));
            if(acos(ori_dir.dot(tar_dir))>ORI_TOL*M_PI/180)
            {
                rotating=true;
                robot.rotateTo(tar_dir);
                rotating=false;
            }
        }
        sendAA(v_level*DELTA_V,v_level*DELTA_V);
        //printf("dist = %f\n",targetDist-moveDist);
        usleep(DELTA_T);
    }
    while(v_level)
    {
        v_level += v_level>0?-1:1;
        sendAA(v_level*DELTA_V,v_level*DELTA_V);
            //printf("dist = %f\n",targetDist-moveDist);
        usleep(DELTA_T);
    }
    kmt_abort = true;
}

void Robot::keepGoal()
{
    int v_level = 0;
    abort = false;
    bool rotating=false;
    bool kmt_abort = false;
    cv::Point2f ballVelocity,ballPosition;
    bool vt_ballLocated;
    cv::Point2f keeper_center = ownGoal_coord + ownGoal_frontDir*KEEPER_DIST2GOAL;
    cv::Point2f keeper_dir(ownGoal_frontDir.y,-ownGoal_frontDir.x);
    moveTo(keeper_center,30);
    rotateTo(keeper_dir);
    void *kmt_params[6];
    float targetDist = BOT_CENTER2CAM_CENTER;
    float moveDist = 0;
    kmt_params[0] = &targetDist;
    kmt_params[1] = &moveDist;
    kmt_params[2] = &v_level;
    kmt_params[3] = this;
    kmt_params[4] = &kmt_abort;
    kmt_params[5] = &rotating;
    pthread_t km_thread;
    pthread_create(&km_thread,NULL,&keeperMotionThread,(void*)kmt_params);
    usleep(1000);
    while(!kmt_abort)
    {
        if(v_level==0 && !rotating)
        {
            adjustWorldCoordinate(image_r,1);
        }
        float temp_dist = moveDist;
        cv::Point2f shift = temp_dist*cv::Point2f(ownGoal_frontDir.y,-ownGoal_frontDir.x);
        x = keeper_center.x+shift.x;
        y = keeper_center.y+shift.y;
        float temp_v_level = v_level;
        cv::Point2f robot_velocity(temp_v_level*DELTA_V*sin(ori),temp_v_level*DELTA_V*cos(ori));
        updateBallStatus();
        updateRadar();
        if(!ballLocated)
        {
            continue;
        }
        if(length(ball_velocity)<MIN_BALL_SPEED_TO_KEEP || ball_velocity.dot(ownGoal_frontDir)/length(ball_velocity)>-0.1)
        {
            //targetDist = 0;
            continue;
        }
        float ball2goal_time = getTime(ball_coord,ball_velocity,ownGoal_coord,keeper_dir);
        float ball2keepline_time = getTime(ball_coord,ball_velocity,keeper_center,keeper_dir);
        cv::Point2f goalPoint = ball_coord + ball2goal_time*ball_velocity;
        cv::Point2f moveToPoint = ball_coord + ball2keepline_time*ball_velocity;
        if(ball2keepline_time<0 || length(goalPoint-ownGoal_coord)>ownGoal_width/2+GOAL_WIDTH_DELTA)
        {
            //targetDist = 0;
            continue;
        }
        shootRoute.clear();
        shootRoute.push_back(cv::Point(x,y));
        shootRoute.push_back(moveToPoint);
        targetDist = (moveToPoint - keeper_center).dot(keeper_dir)+BOT_CENTER2CAM_CENTER;
    }
}

bool Robot::getBallInfo(cv::Point2f &ballVelocity,cv::Point2f &ballPosition)
{
    // to be done by zc
    //printf("in BallInfo\n");
    ballTracker.popFrame(ballTracker.images.size());
    struct timespec ts,te;
    getImage();
    clock_gettime(CLOCK_REALTIME,&ts);
    //printf("image_l...OK\n");
    ballTracker.pushFrame(image_l,0);
    //printf("pushFrame..OK\n");
    //printf("in image\n");
    getImage();
    //printf("out image\n");
    clock_gettime(CLOCK_REALTIME,&te);
    ballTracker.pushFrame(image_l,float(te.tv_nsec-ts.tv_nsec)/(1e9));
    printf("getImage time = %f\n",double(te.tv_nsec-ts.tv_nsec)/(1e9));
    int ret=ballTracker.processFrame(1);
    if(ret!=2)
        return false;
    if(ballTracker.pos[1]==cv::Point3f(-1,-1,-1) || ballTracker.pos[0]==cv::Point3f(-1,-1,-1))
        return false;
    ballPosition.x=ballTracker.pos[1].x;
    ballPosition.y=ballTracker.pos[1].y;
    ballVelocity.x=(ballTracker.pos[1].x-ballTracker.pos[0].x)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    ballVelocity.y=(ballTracker.pos[1].y-ballTracker.pos[0].y)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    printf("ball: world pos=(%f,%f) v=(%f,%f)\n",ballTracker.pos[1].x,ballTracker.pos[1].y,ballVelocity.x,ballVelocity.y);
    cvCircle(ballTracker.images[1],cvPoint(ballTracker.pos_scr[1].x,ballTracker.pos_scr[1].y),ballTracker.pos_scr[1].z,CV_RGB(255,255,0),2);
    cvNamedWindow("tempImage");
    //cvShowImage("tempImage",image_l);
    cvShowImage("src",image_r);
    cvShowImage("tempImage",ballTracker.images[1]);
    cvWaitKey(10);
    ballTracker.popFrame(2);

    return true;

}

void Robot::shoot()
{
    findBall();
    float radius = BALL_RADIUS + ROBOT_RADIUS + DELTA_RADIUS;
    cv::Point2f ball2goal = ownGoal_coord - ball_coord;
    float dist_ball2goal = sqrt(pow(ball2goal.x,2)+pow(ball2goal.y,2));
    cv::Point2f prep2ball = ball2goal*(1/dist_ball2goal)*SHOOT_PREP_DIST;
    cv::Point2f shootPrepPosition = ball_coord - prep2ball;
    cv::Point2f targetPosition = ownGoal_coord - prep2ball*(TO_GOAL_DIST/float(SHOOT_PREP_DIST));
    cv::Point2f robotPosition(x,y);
    cv::Point2f robot2prep = shootPrepPosition - robotPosition;
    float dist_robot2prep = sqrt(pow(robot2prep.x,2)+pow(robot2prep.y,2));
    float dot = robot2prep.dot(prep2ball)/dist_robot2prep/SHOOT_PREP_DIST;
    dot = dot>1?1:(dot<-1?-1:dot);
    float dist = SHOOT_PREP_DIST*sin(acos(dot));
    bool hidden = dot<0 && dist<radius && dist_robot2prep>SHOOT_PREP_DIST;
    cv::Point2f turningPoint;
    bool turn = false;
    if(hidden)
    {
        printf("rp = %f,%f\n",robotPosition.x,robotPosition.y);
        printf("bp = %f,%f\n",ball_coord.x,ball_coord.y);
        printf("tp = %f,%f\n",shootPrepPosition.x,shootPrepPosition.y);
        printf("r = %f\n",radius);
        turn = getTurningPoint(robotPosition,ball_coord,shootPrepPosition,turningPoint,radius);
        printf("tp = %f,%f\n",turningPoint.x,turningPoint.y);
    }
    shootRoute.push_back(robotPosition);
    if(turn)
        shootRoute.push_back(turningPoint);
    shootRoute.push_back(shootPrepPosition);
    shootRoute.push_back(targetPosition);
    updateRadar();
    if(turn)
    {
        //getImage();
        //adjustWorldCoordinate(image_r,1);
        moveTo(turningPoint,30);
    }
    //getImage();
    //adjustWorldCoordinate(image_r,1);
    moveTo(shootPrepPosition,30);
    //getImage();
    //adjustWorlfdCoordinate(image_r,1);
    moveTo(targetPosition,50);
    shootRoute.clear();
    updateRadar();
}

void Robot::spin() {//S型过人
    cv::Point2f robot_coord(x, y);
    std::vector<cv::Point2f> ans = findMulBall();
    cv::Point2f ball1 = ans[0];
    cv::Point2f ball2 = ans[1];
    //printf("%f %f\n %f %f\n", ball1.x, ball1.y, ball2.x, ball2.y);
    float delta =  30;
    float r12 =  cal_distance(ball1, ball2) / 2;
    float rspin = (delta * delta + r12 * r12) / (2 * delta);
    rspin += 10;//误差修正
    float disr1 =  cal_distance(ball1, robot_coord);
    int  inrspin = 0;
    while(inrspin < rspin)
        inrspin++;
    //printf("rspin:%f, disr1:%f, inrspin:%d\n", rspin, disr1, inrspin);
    cv::Point2f rto1 = ball1 - robot_coord;
    cv::Point2f b1to2 = ball2 - ball1;

    float arc = asin(r12 / rspin);
    cv::Point2f pturn1(ball1.x -  b1to2.x * 0.5, ball1.y - b1to2.y *0.5);

    moveTo(pturn1, 20);
    cv::Point2f robotPosition(x,y);
    cv::Point2f dir= ball1 - robotPosition;
    float dir_len = length(dir);
    if (dir_len <= 0)
        return;
    cv::Point2f new_dir = dir*(1/dir_len);
    rotateTo(new_dir);
    turnRight(arc * 180 / M_PI);
    moveRotate(true, rspin, arc * 2 + 0.2);//误差修正
    moveRotate(false, rspin, arc * 2 + 0.2);
}

void Robot::updateRadar()
{
    if(!radar)
        return;
    IplImage* wMap = worldMap.getMap();
    int x = this->x+(MAP_LEN>>1);
    int y = (MAP_LEN>>1)+1-this->y;
    cv::Point2f ownGoal_coord = world2image(this->ownGoal_coord);
    cv::Point2f ball_coord = world2image(this->ball_coord);


    if(shootRoute.size()>1)
    {
        for(int i=0; i<shootRoute.size()-1; i++)
        {
            cvLine(wMap,world2image(shootRoute[i]),world2image(shootRoute[i+1]),CV_RGB(255,128,189),3);
            //printf("coord(%f,%f)\n",shootRoute[i].x,shootRoute[i].y);
        }
        //printf("ball(%f,%f)\n",ball_coord.x,ball_coord.y);
    }

    cvCircle(wMap,cvPoint(x,y),ROBOT_RADIUS,CV_RGB(0,0,0),-1);
    cvLine(wMap,cvPoint(x,y),cvPoint(x+ROBOT_RADIUS*sin(ori),y-ROBOT_RADIUS*cos(ori)),CV_RGB(255,255,0),3);
    if(ballLocated)
    {
        cvCircle(wMap,cvPoint(ball_coord.x,ball_coord.y),BALL_RADIUS,CV_RGB(255,0,0),-1);
        cvLine(wMap,cvPoint(ball_coord.x,ball_coord.y),cvPoint(ball_coord.x+ball_velocity.x,ball_coord.y-ball_velocity.y),CV_RGB(0,255,255),2);
    }
    if(ownGoalLocated)
        cvCircle(wMap,cvPoint(ownGoal_coord.x,ownGoal_coord.y),5,CV_RGB(0,0,255),-1);
    cvShowImage(RADAR_WND_NAME,wMap);
    cvSaveImage("Radar.png",wMap);
    cvWaitKey(10);
    cvReleaseImage(&wMap);
}

bool Robot::locateBall()
{
    if(!image_r)
        return false;
    ip.setBound(BALL_BOUND);
    std::vector<cv::Point3f> circles = ip.extractCircles(image_r);
    if(circles.size()<=0)
    {
        return false;
    }

    cv::Point2f rCoord;
    rCoord = worldMap.coord_screen2robot(cv::Point2f(circles[0].x,circles[0].y+circles[0].z));
    ball_coord = worldMap.coord_robot2world(rCoord);
    CvRect bbox = worldMap.getMap_bbox();
    cv::Point2f ball_coord_img = world2image(ball_coord);
    if(ball_coord_img.x<bbox.x || ball_coord_img.x>bbox.x+bbox.width || ball_coord_img.y<bbox.y || ball_coord_img.y>bbox.y+bbox.height)
    {
        return false;
    }
    ballLocated = true;
    updateRadar();
    return true;
}

/*
 * 找多个球，改编自单个球，返回vector数组
 * 为了支持绕障碍功能
 */
std::vector<cv::Point2f> Robot::findMulBall()
{
    getImage();
    std::vector<cv::Point2f> ans;
    while (true) {
        if (!image_r)
            return ans;
        ip.setBound(BALL_BOUND);
        std::vector<cv::Point3f> tmp = ip.extractCircles(image_r);
        if (tmp.size() <= 0) {
            turnRight(FIND_BALL_ANGLE);
            getImage();
            continue;
        }
        cv::Point2f rCoord;
        rCoord = worldMap.coord_screen2robot(cv::Point2f(tmp[0].x, tmp[0].y + tmp[0].z));
        cv::Point2f ball_coord_sub = worldMap.coord_robot2world(rCoord);
        CvRect bbox = worldMap.getMap_bbox();
        cv::Point2f ball_coord_img = world2image(ball_coord_sub);
        if (ball_coord_img.x >= bbox.x && ball_coord_img.x <= bbox.x + bbox.width &&
            ball_coord_img.y >= bbox.y && ball_coord_img.y <= bbox.y + bbox.height) {
            turnRight(FIND_BALL_ANGLE);
            getImage();
            continue;
        }
        if (ans.size() == 1) {
            //判断找到的是不是同一个球
            if (abs(ball_coord_sub.x - ans[0].x) <= 10 && abs(ball_coord_sub.y - ans[0].y) <= 10) {
                turnRight(FIND_BALL_ANGLE);
                getImage();
                continue;
            }
        }
        ans.push_back(ball_coord_sub);
        if (ans.size() > 1)
            break;
        turnRight(FIND_BALL_ANGLE);
        getImage();
    }

    return ans;
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
    if(image_l)
        cvReleaseImage(&image_l);
    if(image_r)
        cvReleaseImage(&image_r);
	//ptEnd();
}

bool Robot::locateOwnGate()
{
    IplImage* wMap = worldMap.getMap();
    uchar* wMap_data = (uchar*)wMap->imageData;
    float center_x = 0, center_y = 0;
    float n = 0;
    for(int i = 0; i < MAP_LEN; i++)
    {
        for(int j = 0; j < MAP_LEN; j++)
        {
            int b = wMap_data[i*MAP_LEN*3+j*3+0];
            int g = wMap_data[i*MAP_LEN*3+j*3+1];
            int r = wMap_data[i*MAP_LEN*3+j*3+2];
            if(b == 255 && g==0 && r==0)
            {
                center_x += j;
                center_y += i;
                n++;
            }
        }
    }
    if(n < 10)
    {
        cvReleaseImage(&wMap);
        return false;
    }
    else
    {
        center_x = center_x/n;
        center_y = center_y/n;
        ownGoal_coord.x = center_x - (MAP_LEN>>1);
        ownGoal_coord.y = (MAP_LEN>>1)+1-center_y;
        ownGoalLocated = true;

        cv::Point2f end1,end2,center(center_x,center_y);
        float dist_max = 0;
        for(int i = 0; i < MAP_LEN; i++)
        {
            for(int j = 0; j < MAP_LEN; j++)
            {
                int b = wMap_data[i*MAP_LEN*3+j*3+0];
                int g = wMap_data[i*MAP_LEN*3+j*3+1];
                int r = wMap_data[i*MAP_LEN*3+j*3+2];
                cv::Point2f p(j,i);
                float dist = length(center-p);
                if(b == 255 && g==0 && r==0 && dist>dist_max)
                {
                    dist_max = dist;
                    end1 = p;
                }
            }
        }
        dist_max = 0;
        for(int i = 0; i < MAP_LEN; i++)
        {
            for(int j = 0; j < MAP_LEN; j++)
            {
                int b = wMap_data[i*MAP_LEN*3+j*3+0];
                int g = wMap_data[i*MAP_LEN*3+j*3+1];
                int r = wMap_data[i*MAP_LEN*3+j*3+2];
                cv::Point2f p(j,i);
                float dist = length(center-p);
                if(b == 255 && g==0 && r==0 && dist>dist_max && (p-center).dot(end1-center)<0)
                {
                    dist_max = dist;
                    end2 = p;
                }
            }
        }
        cv::Point2f goalWidthDir = end1-end2;
        ownGoal_width = length(goalWidthDir);
        ownGoal_frontDir = cv::Point2f(-goalWidthDir.y,goalWidthDir.x);
        ownGoal_frontDir = ownGoal_frontDir*(1/length(ownGoal_frontDir));
        ownGoal_frontDir.y = -ownGoal_frontDir.y;

        cv::Point2f robot_coord(x,y);
        if(ownGoal_frontDir.dot(robot_coord - ownGoal_coord)<0)
        {
            ownGoal_frontDir = -ownGoal_frontDir;
        }
        cvReleaseImage(&wMap);
        return true;
    }
}
int Robot::init_socket(const char* ipStr, const int host){
    int socket_fd;
    struct sockaddr_in s_add;
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

	if(-1 == socket_fd){
		printf("socket fail \n");
		exit(-1);
	}
	//printf("socket ok \n");

	memset(&s_add, 0, sizeof(s_add));
	s_add.sin_family = AF_INET;
	s_add.sin_addr.s_addr = inet_addr(ipStr);
	s_add.sin_port = htons(host);

	//printf("s_addr = %#x ,port : %#x\r\n",s_add.sin_addr.s_addr,s_add.sin_port);

	if(-1 == connect(socket_fd,(struct sockaddr *)(&s_add), sizeof(struct sockaddr))){
		printf("connect fail \n");
		return 0;
	}

	printf("connect ok !\n");
	socket_robo =  socket_fd;
    return socket_fd;
}
void* stopKeeperThread(void * args){
    Robot *rb = (Robot*) args;
    int socket_fd = rb->socket_robo;
     if(socket_fd){
        int recvbytes = 0;
        int BUFFER_SIZE = 2048;
        char read_buff[BUFFER_SIZE];
          while(1){
            bzero(read_buff, sizeof(read_buff));
            if(-1 == (recvbytes = recv(socket_fd,read_buff,BUFFER_SIZE, 0))){
                printf("read data fail \n");
            }
            else
            {
                string recvCmd(read_buff);

                if(recvCmd == "stopKeeper"){
                    printf("receive scan\n");
                    //do something
                    rb->abort = true;
                    char tmp[BUFFER_SIZE];
                    strcpy(tmp, "stopKeeper_ack");
                    if (send(socket_fd, tmp, (int)strlen(tmp), 0) < 0){
                        printf("send stopKeeper_ack fail\n");
                    }
                    else{
                        printf("send stopKeeper_ack success\n");
                    }
                    return NULL;
                }//if
            }//else
        }//while
    }
    return NULL;
}
void  Robot::listenAndAct(){
    int socket_fd = socket_robo;
    if(socket_fd){
        int recvbytes = 0;
        int BUFFER_SIZE = 2048;
        char read_buff[BUFFER_SIZE];

        while(1){
            bzero(read_buff, sizeof(read_buff));
            if(-1 == (recvbytes = recv(socket_fd,read_buff,BUFFER_SIZE, 0))){
                printf("read data fail \n");
            }
            else
            {
                string recvCmd(read_buff);

                if(recvCmd == "scan"){
                    printf("receive scan\n");
                    //do something
                    drawMap();
                    char tmp[BUFFER_SIZE];
                    strcpy(tmp, "scan_ack");
                    if (send(socket_fd, tmp, (int)strlen(tmp), 0) < 0){
                        printf("send scan_ack fail\n");
                    }
                    else{
                        printf("send scan_ack success\n");
                    }
                }
                else if(recvCmd == "keeper"){
                    printf("receive keeper\n");
                    //do something
                    pthread_t t1;
                    pthread_create(&t1, NULL, stopKeeperThread, this);
                    keepGoal();
                    char tmp[BUFFER_SIZE];
                    strcpy(tmp, "keeper_ack");
                    if (send(socket_fd, tmp, (int)strlen(tmp), 0) < 0){
                        printf("send keeper_ack fail\n");
                    }
                    else{
                        printf("send keeper_ack success\n");
                    }
                }
                else if(recvCmd == "shoot"){
                    printf("receive shoot\n");
                    //do something
                    shoot();
                    char tmp[BUFFER_SIZE];
                    strcpy(tmp, "shoot_ack");
                    if (send(socket_fd, tmp, (int)strlen(tmp), 0) < 0){
                        printf("send shoot_ack fail\n");
                    }
                    else{
                        printf("send shoot_ack success\n");
                    }
                }
                else if(recvCmd == "spin"){
                    printf("receive spin\n");
                    //do something
                    spin();
                    char tmp[BUFFER_SIZE];
                    strcpy(tmp, "spin_ack");
                    if (send(socket_fd, tmp, (int)strlen(tmp), 0) < 0){
                        printf("send spin_ack fail\n");
                    }
                    else{
                        printf("send spin_ack success\n");
                    }
                }
                else if(recvCmd == "exit"){
                    printf("receive exit\n");
                    close(socket_fd);
                    break;
                }
                else if(recvCmd == "pic"){
                    char cmd[200];
                    bzero(cmd, sizeof(cmd));
                    strcpy(cmd, "sendpicstart");
                    if(send(socket_fd, cmd, (int)strlen(cmd), 0) < 0){
                        printf("send start cmd fail\n");
                        continue;
                    }
                    if(-1 == (recvbytes = recv(socket_fd,read_buff,BUFFER_SIZE, 0))){
                        printf("receive start cmd ack fail\n");
                        continue;
                    }
                    if(strcmp(read_buff, "fileack") != 0){
                        printf("receive not fileack\n");
                        continue;
                    }

                    int sendContent = 1;
                    int width = 128, height = 128;
                    IplImage* src = cvLoadImage("Radar.png");
                    if(src == NULL){
                        printf("no Radar.png\n");
                        continue;
                    }
                    int dst_width = 128, dst_height = 128;
                    IplImage* img = cvCreateImage(cvSize(dst_width, dst_height), src->depth, 3);
                    cvResize(src, img, CV_INTER_LINEAR);

                    uchar* img_data = (uchar *)img->imageData;

                    int sendRgbSize = 128*3;
                    int dataCharSize = width*height*3*3;
                    int sendCharSize = 128*3*3;
                    int sendTime = dataCharSize/sendCharSize;
                    printf("sendTime: %d\n", sendTime);

                    uchar sendChardata[sendCharSize];
                    for(int i = 0; i < sendTime; i++){
                        bzero(sendChardata, sendCharSize);
                        for(int j = 0; j < sendRgbSize; j++){
                            int x = (int)img_data[i*sendRgbSize+j];
                            sendChardata[j*3+0] = x/100+'0';
                            sendChardata[j*3+1] = (x%100)/10+'0';
                            sendChardata[j*3+2] = (x%10)+'0';
                        }
                        //cout << i << endl;

                        if(send(socket_fd, sendChardata, sendCharSize, 0) < 0){
                            printf("%d send rgb_data fail\n", i);
                            sendContent = 0;
                            break;
                        }

                        //cout << i << " send rgb_data success" << endl;
                        if(-1 == (recvbytes = recv(socket_fd,read_buff,BUFFER_SIZE, 0))){
                            printf("%d receive data ack fail\n", i);
                            sendContent = 0;
                            break;
                        }
                        if(strcmp(read_buff, "fileack") != 0){
                            printf("%d receive not ackfile\n", i);
                            sendContent = 0;
                            break;
                        }
                    }
                    if(sendContent == 0){
                        printf("sendContent = false\n");
                        continue;
                    }
                    bzero(cmd, sizeof(cmd));
                    strcpy(cmd, "sendpicfinish");
                    if(send(socket_fd, cmd, (int)strlen(cmd), 0) < 0){
                        printf("send file finish fail\n");
                        continue;
                    }
                    printf("data all send\n");
                    cvReleaseImage(&src);
                    cvReleaseImage(&img);
                }//else if

            }//else
        }
    }
}

bool Robot::adjustWorldCoordinate(IplImage* image, double coordAdjustRate)
{
    IplImage *img;
    IplImage* src1=cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);
    if(image->nChannels==3)
    {
        IplImage *hsv_img = get_hsv(image);
        img=worldMap.getField(hsv_img);
        cvReleaseImage(&hsv_img);
        src1=img;
    }
    else
    {
        img=image;
        src1=img;
            //cvCvtColor(img, src1, CV_BGR2GRAY);
    }
		if( img != 0 )
		{
			IplImage* dst = cvCreateImage( cvGetSize(img), 8, 1 );
			IplImage* color_dst = cvCreateImage( cvGetSize(img), 8, 3 );
			CvMemStorage* storage = cvCreateMemStorage(0);
			CvSeq* ls = 0;
			int i;
			cvCanny( src1, dst, 50, 200, 3 );

			cvCvtColor( dst, color_dst, CV_GRAY2BGR );

			ls = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 2, CV_PI/90, 20, 5, 30 );
			//ls = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 5, CV_PI/30, 10, 20, 5 );
			vector<myLine> tmplines;
			for( i = 0; i < ls->total; i++ )
			{
				CvPoint* tmpl = (CvPoint*)cvGetSeqElem(ls,i);
				cvLine( color_dst, tmpl[0], tmpl[1], CV_RGB(255,0,0), 1, 8 );

                cv::Point2f tmpp[2];
                cv::Point2f scrPos(tmpl[0].x,tmpl[0].y);
                cv::Point2f roboPos=worldMap.coord_screen2robot(scrPos,true);
                cv::Point2f worldPos=worldMap.coord_robot2world(roboPos);
                tmpp[0]=worldPos;
                scrPos=cv::Point2f(tmpl[1].x,tmpl[1].y);
                roboPos=worldMap.coord_screen2robot(scrPos,true);
                worldPos=worldMap.coord_robot2world(roboPos);
                tmpp[1]=worldPos;
                myLine templ(tmpp[0],tmpp[1]);
                if(templ.l>LINE_LENGTH_LBOUND)
                    tmplines.push_back(templ);
//				//printf("length=%f angle=%f\n",sqrt(float((tmpl[1].y-tmpl[0].y)*(tmpl[1].y-tmpl[0].y));
				//	+float((tmpl[1].x-tmpl[0].x)*(tmpl[1].x-tmpl[0].x)))
				//	,atan2(float(tmpl[1].y-tmpl[0].y),float(tmpl[1].x-tmpl[0].x)));
			}
			//printf("\n");
			cvNamedWindow( "Source", 1 );
			cvShowImage( "Source", img );

			cvNamedWindow( "Hough", 1 );
			cvShowImage( "Hough", color_dst );

			cvWaitKey(10);
			cvReleaseImage(&dst);
			cvReleaseImage(&src1);
			cvReleaseImage(&color_dst);
			cvReleaseMemStorage(&storage);
			if(coordAdjustRate==0)
			{
                for(i=0;i<tmplines.size();++i)
                {
                    lines.push_back(tmplines[i]);
                }
			}
			else if(coordAdjustRate==2)
			{
                for(i=0;i<tmplines.size();++i)
                {
                    lines.push_back(tmplines[i]);
                }
                //vector<double> oris;
                vector<int> lineNums;
                vector<double> lineValues;
                int groupId=0;
			    for(i=0;i<lines.size();++i)
			    {
			        bool classified=false;
			        int j;
			        for(j=0;j<i;++j)
			        {
			            double angle=lines[i].theta-lines[j].theta+CV_PI/4.0;   //to make the process simple, add 45 degree
                                                                                //to turn the cared angles to the middle of a phase
			            if(angle<0)
                            angle+=CV_PI*2.0;
			            int phase=(int)(angle/(CV_PI/2.0));
			            double angle90=angle-CV_PI/2.0*(double)phase;
			            phase%=2;
			            if(abs(angle90-CV_PI/4.0)<CV_PI/60.0)//subtract the added 45 degree
			            {
			                lines[i].clsId=lines[j].clsId/2*2+phase;
			                ++lineNums[lines[i].clsId];
			                lineValues[lines[i].clsId]+=lines[i].l;
			                classified=true;
			                break;
			            }
			        }
			        if(classified==false)
			        {
			            lines[i].clsId=groupId;
                        lineNums.push_back(1);
                        lineNums.push_back(0);
                        lineValues.push_back(lines[i].l);
                        lineValues.push_back(0);
			            groupId+=2;
			        }
			    }
			    int maxValueGroup=0;
			    double maxValue=0;
			    for(i=0;i<lineNums.size();i+=2)
			    {
			        if(lineValues[i]+lineValues[i+1]>maxValue)
			        {
			            maxValue=lineValues[i]+lineValues[i+1];
			            maxValueGroup=i;
			        }
			    }
			    maxValueGroup/=2;
			    double sumAngle=0;
			    double sumL=0;
			    for(i=0;i<lines.size();++i)
			    {
			        if(lines[i].clsId/2==maxValueGroup)
			        {
			            double angle=lines[i].theta+CV_PI/4.0;//similar strategy, add 45 degree
			            if(angle<0)
                            angle+=CV_PI*2.0;
			            double angle90=angle-CV_PI/2.0*(double)((int)(angle/(CV_PI/2.0)));
			            sumAngle+=(angle90-CV_PI/4.0)*lines[i].l;//subtract 45 degree
			            sumL+=lines[i].l;
			        }
			    }
			    if(sumL==0)
			    {
                    //printf("false 2 sumL=0\n");
			        return false;
			    }
			    mainAngle=sumAngle/sumL;
			    mainGroupId=maxValueGroup;
			    //printf("mainAngle=%f mainGroupId=%d\n",mainAngle,mainGroupId);
			}
			else if(coordAdjustRate==1)
            {
                CvRect bBox=worldMap.getMap_bbox();
                    //printf("in func param=1\n");
                    //printf("tmplines.size=%d\n",tmplines.size());
                for(i=0;i<tmplines.size();++i)
                {
                    cv::Point2f imgPos=world2image(tmplines[i].p[0]);
                    if(!(imgPos.x>bBox.x-BBOX_DELTA && imgPos.x<bBox.x+bBox.width+BBOX_DELTA && imgPos.y>bBox.y-BBOX_DELTA && imgPos.y<bBox.y+bBox.height+BBOX_DELTA))
                        continue;
			        bool classified=false;
			        double minAngle=CV_PI;
			        int minAnglePhase=0;
			        int bestJ=-1;
			        int j;
			        for(j=0;j<lines.size();++j)
			        {
			            if(lines[j].clsId/2!=mainGroupId)
                            continue;
			            double angle=tmplines[i].theta-lines[j].theta+CV_PI/4.0;   //to make the process simple, add 45 degree
                                                                                //to turn the cared angles to the middle of a phase
			            if(angle<0)
                            angle+=CV_PI*2.0;
			            int phase=(int)(angle/(CV_PI/2.0));
			            double angle90=angle-CV_PI/2.0*(double)phase;
			            phase%=2;
			            if(abs(angle90-CV_PI/4.0)<minAngle)//subtract the added 45 degree
			            {
			                minAngle=abs(angle90-CV_PI/4.0);
			                bestJ=j;
                                minAnglePhase=phase;
			            }
			        }
			        if(bestJ>-1)
			        {
			            //if(minAngle<CV_PI/6.0)
                        tmplines[i].clsId=mainGroupId*2+minAnglePhase;
                        classified=true;
                        //printf("nearest main ori found. angle diff=%f\n",minAngle);
			        }
			    }
			    double sumAngle=0;
			    double sumL=0;
			    for(i=0;i<tmplines.size();++i)
			    {
			        if(tmplines[i].clsId/2==mainGroupId)
			        {
                    //printf("comparing with a main line..i=%d\n",i);
			            double angle=tmplines[i].theta+CV_PI/4.0;//similar strategy, add 45 degree
			            if(angle<0)
                            angle+=CV_PI*2.0;
			            double angle90=angle-CV_PI/2.0*double((int)(angle/(CV_PI/2.0)));
			            sumAngle+=angle90*tmplines[i].l;//use the 45 degree to balance the unwanted lines
			            sumL+=tmplines[i].l;
			        }
			    }
			    if(sumL<LINE_LENGTH_SUM_LBOUND)
			    {
                    //printf("false sumL=%f<%d\n",sumL,LINE_LENGTH_SUM_LBOUND);
			        return false;
			    }
			    double curAngle=sumAngle/sumL-CV_PI/4.0;//subtract 45 degree
			    ori+=curAngle-mainAngle;
                    //printf("true oriChange=%f\n",curAngle-mainAngle);
            }
		}

    return true;
}
