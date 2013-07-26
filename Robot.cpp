#include "Robot.h"
#include "motor_uplayer.h"
#include "getPhoto.h"
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

void Robot::drawMap()
{
    printf("in\n");
    getImage();
    printf("out\n");
    for(int i=0; i<12; i++)
    {
        getImage();
        cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("src", 512, 512);
        cvShowImage("src", image_r);
        cvWaitKey(100);

        printf("draw start\n");
        worldMap.updateMap(image_r);
        locateOwnGate();
        printf("draw finish\n");
        turnRight(30);
    }
    /*moveForward(200, 20);
    for(int i=0;i<12;i++)
    {
        getImage();
        printf("draw start\n");
        worldMap.updateMap(image);
        printf("draw finish\n");
        turnRight(30);
    }*/

    //getImage();
    //worldMap.updateMap(image);
    //turnRight(180);
    //getImage();
    //worldMap.updateMap(image);
    //moveForward(200,30);
    //getImage();
    // worldMap.updateMap(image);
    //worldMap.saveMap("result.png");
}

void Robot::getImage()
{
    usleep(SLEEPTIME_BEFORE_PHOTO); // sleep until the camera is still
    if(!image_l)
        image_l = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    if(!image_r)
        image_r = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    getPhoto(image_l, image_r);
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
       /* x += 2 * radius * sin(arc / 2) * cos(ori + arc / 2);
        y += 2 * radius * sin(arc / 2) * sin(ori + arc / 2);
        ori = ori - arc;*/
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

        int min1 = 0, min2 = 0, mind = 10000;
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
            printf("%d %d %f", min1, min2, t);
            goWithSpeed(min2, min1, t);
    }
    else  //turn right
    {
        //x += 2 * radius * sin(arc / 2) * cos(ori + arc / 2 - M_PI_2);
        //y += 2 * radius * sin(arc / 2) * sin(ori + arc / 2 - M_PI_2);
        //ori = ori + arc;
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

        int min1 = 0, min2 = 0, mind = 10000;
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
            printf("%d %d %f", min1, min2, t);
            goWithSpeed(min1, min2, t);
    }

}


void Robot::moveTo(const cv::Point2f& wCoord,float max_speed)
{
    cv::Point2f robotPosition(x,y);
    cv::Point2f delta = wCoord - robotPosition;
    //printf("%f %f\n", wCoord.x, wCoord.y);
    //printf("%f %f\n", delta.x, delta.y);
    float delta_len = length(delta);
    if (delta_len <= 0)
        return;
    cv::Point2f new_dir = delta*(1/delta_len);

    rotateTo(new_dir);
    moveForward(delta_len,max_speed);
}

void Robot::rotateTo(const cv::Point2f &new_dir)
{
    //printf("%f %f\n", new_dir.x, new_dir.y);
    cv::Point2f dir(sin(ori),cos(ori));
    float cross = dir.x*new_dir.y-dir.y*new_dir.x;
    float dot = new_dir.dot(dir);
    dot = dot>1?1:(dot<-1?-1:dot);
    float angle = acos(dot);
    //printf("angle = %f\n",angle);
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
    printf("t = %f\n",temptime);
    printf("images.size=%d before processing.\n",ballTracker.images.size());
    int ret;
    if(ballTracker.pos.size()==2)
        ret=ballTracker.processFrame(1);
    else if(ballTracker.pos.size()==1)
        ret=ballTracker.processFrame(0);
    else
    {
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
            ballTracker.popFrame(ballTracker.images.size());
            break;
        }
    }
    printf("images.size=%d\n",ballTracker.images.size());
    if(ballTracker.images.size()!=2 || ret!=2)
        return;
    //if(ballTracker.pos[1].z<ballTracker.pos[0].z)
        //return;
    ballPosition.x=ballTracker.pos[1].x;
    ballPosition.y=ballTracker.pos[1].y;
    ballVelocity.x=(ballTracker.pos[1].x-ballTracker.pos[0].x)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    ballVelocity.y=(ballTracker.pos[1].y-ballTracker.pos[0].y)/(ballTracker.pos[1].z-ballTracker.pos[0].z);
    ballLocated=true;
    printf("ball: world pos=(%f,%f) v=(%f,%f)\n",ballTracker.pos[1].x,ballTracker.pos[1].y,ballVelocity.x,ballVelocity.y);
    printf("showing\n");

    cvCircle(ballTracker.images[1],cvPoint(ballTracker.pos_scr[1].x,ballTracker.pos_scr[1].y),ballTracker.pos_scr[1].z,CV_RGB(255,255,0),2);
    cvNamedWindow("leftImage");
    cvMoveWindow("leftImage",512,0);
    cvShowImage("leftImage",ballTracker.images[1]);
    cvWaitKey(10);
    //cvDestroyWindow("leftImage");
    cvNamedWindow("image_l");
    cvMoveWindow("image_l",512,512);
    cvShowImage("image_l",image_l);
    cvWaitKey(10);
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
    while(!robot.abort)
    {
        //moveDist += v_level*DELTA_V*DELTA_T/float(1e6);
        v_level += getAcc(v_level,targetDist-moveDist);
       // sendAA(v_level*DELTA_V,v_level*DELTA_V);
        usleep(DELTA_T);
    }
}

void Robot::keepGoal()
{
    int v_level = 0;
    abort = false;
    cv::Point2f ballVelocity,ballPosition;
    bool vt_ballLocated;
    cv::Point2f keeper_center = ownGoal_coord + ownGoal_frontDir*KEEPER_DIST2GOAL;
    cv::Point2f keeper_dir(ownGoal_frontDir.y,-ownGoal_frontDir.x);
    moveTo(keeper_center,30);
    rotateTo(keeper_dir);
    void *kmt_params[4];
    float targetDist = 0;
    float moveDist = 0;
    kmt_params[0] = &targetDist;
    kmt_params[1] = &moveDist;
    kmt_params[2] = &v_level;
    kmt_params[3] = this;
    pthread_t km_thread;
    pthread_create(&km_thread,NULL,&keeperMotionThread,(void*)kmt_params);
    usleep(1000);
    while(!abort)
    {
        float temp_dist = moveDist;
        x = keeper_center.x+temp_dist*sin(ori);
        y = keeper_center.y+temp_dist*cos(ori);
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
        targetDist = (moveToPoint - keeper_center).dot(keeper_dir);
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
        moveTo(turningPoint,30);
    moveTo(shootPrepPosition,30);
    moveTo(targetPosition,50);
    shootRoute.clear();
    updateRadar();
}

void Robot::spin() {//}std::vector<cv::Point2f> balls) {
    cv::Point2f robot_coord(x, y);
    //printf("find %d balls\n", balls.size());
    //if (balls.size() < 2)
     //   return;
    //std::vector<cv::Point2f> ans = findMulBall();
    cv::Point2f ball1(0, -40);// = ans[0];// = balls[0];
    cv::Point2f ball2(0, 80);// = ans[1];// = balls[1];
    printf("%f %f\n %f %f\n", ball1.x, ball1.y, ball2.x, ball2.y);
    //cvWaitKey();
    //get them by some means of find balls
    float delta =  40;
    float r12 =  cal_distance(ball1, ball2) / 2;
    float rspin = (delta * delta + r12 * r12) / (2 * delta);
    float disr1 =  cal_distance(ball1, robot_coord);
    int  inrspin = 0;
    while(inrspin < rspin)
        inrspin++;
    printf("rspin:%f, disr1:%f, inrspin:%d\n", rspin, disr1, inrspin);
    //cvWaitKey();
    cv::Point2f rto1 = ball1 - robot_coord;
    cv::Point2f b1to2 = ball2 - ball1;

    float arc = asin(r12 / rspin);
    //float turn = a.x * b.y - a.y * b.x;
    //printf("a.x:%f, a.y:%f\n", a.x, a.y);
    //cvWaitKey();
    cv::Point2f pturn1(ball1.x -  b1to2.x * 0.5, ball1.y - b1to2.y *0.5);

     //cvWaitKey();
    moveTo(pturn1, 20);
    cv::Point2f robotPosition(x,y);
    cv::Point2f dir= ball1 - robotPosition;
    //printf("%f %f\n", wCoord.x, wCoord.y);
    //printf("%f %f\n", delta.x, delta.y);
    float dir_len = length(dir);
    if (dir_len <= 0)
        return;
    cv::Point2f new_dir = dir*(1/dir_len);
    rotateTo(new_dir);
    //cvWaitKey();
    turnRight(arc * 180 / M_PI);
    //cvWaitKey()
    moveRotate(true, rspin, arc * 2);//!!!
    cvWaitKey();
    moveRotate(false, rspin, arc * 2);
    /*if (turn > 0) {
        turnRight(arc);
        moveRotate(true, inrspin, 2 * M_PI - arc);
        usleep(5000000);
        moveRotate(false, inrspin, M_PI);
    }
    else {
        turnLeft(90);
        moveRotate(false, inrspin, 2 * M_PI - arc);
        usleep(5000000);
        moveRotate(true, inrspin, M_PI);
    }*/
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
    if(ball_coord_img.x>=bbox.x && ball_coord_img.x<=bbox.x+bbox.width && ball_coord_img.y>=bbox.y && ball_coord_img.y<=bbox.y+bbox.height)
    {
        return false;
    }
    ballLocated = true;
    updateRadar();
    return true;
}

std::vector<cv::Point2f> Robot::findMulBall()
{
    getImage();
    std::vector<cv::Point2f> ans;
    // std::vector<cv::Point3f> tmp;
    while (true) {
        if (!image_r)
            return ans;
        ip.setBound(BALL_BOUND);
        std::vector<cv::Point3f> tmp = ip.extractCircles(image_r);
//        std::cout << "tmp size: " << tmp.size() << std::endl;
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
            if (abs(ball_coord_sub.x - ans[0].x) <= 10 && abs(ball_coord_sub.y - ans[0].y) <= 10) {
//                std::cout << "find the same ball!" << std::endl;
                turnRight(FIND_BALL_ANGLE);
                getImage();
                continue;
            }
        }
        ans.push_back(ball_coord_sub);
        if (ans.size() > 1)
            break;
//        cout << "ans size: " << ans.size() << std::endl;
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
	ptEnd();
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

void Robot::adjustWorldCoordinate()
{

    return;
}
