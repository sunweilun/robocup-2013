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
        cvShowImage("src", image);
        cvWaitKey(100);

        printf("draw start\n");
        worldMap.updateMap(image);
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
    getPhoto();
    char dp[] = DATA_PATH;
    char fn[1024];
    sprintf(fn,"%s%d_r.dat",dp,imgCounter);
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

        float rinner = radius - DIST_BETWEEN_WHEELS / 2;
        float rout = radius + DIST_BETWEEN_WHEELS / 2;
        float t = (arc) * rout / 20;
        int vin = 20 * rinner / rout;
        goWithSpeed(vin, 20, t);
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


        float rinner = radius - DIST_BETWEEN_WHEELS / 2;
        float rout = radius + DIST_BETWEEN_WHEELS / 2;
        float t = (arc) * rout / 20;
        int vin = 20 * rinner / rout;
        goWithSpeed(20, vin, t);
    }
}


void Robot::moveTo(const cv::Point2f& wCoord,float max_speed)
{
    cv::Point2f robotPosition(x,y);
    cv::Point2f delta = wCoord - robotPosition;
    float delta_len = length(delta);
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
    printf("angle = %f\n",angle);
    if(cross<0)
        turnRight(angle*180/M_PI);
    else
        turnLeft(angle*180/M_PI);
}

void Robot::keepGoal()
{
    bool abort = false;
    cv::Point2f keeper_center = ownGoal_coord + ownGoal_frontDir*KEEPER_DIST2GOAL;
    cv::Point2f keeper_dir(ownGoal_frontDir.y,-ownGoal_frontDir.x);
    moveTo(keeper_center,30);
    rotateTo(keeper_dir);
    while(!abort)
    {
        cv::Point2f ballVelocity,ballPosition;
        getBallInfo(ballVelocity,ballPosition);
        if(length(ballVelocity)<MIN_BALL_SPEED_TO_KEEP || ballVelocity.dot(ownGoal_frontDir)/length(ballVelocity)>-0.1)
            continue;
        float ball2goal_time = getTime(ballPosition,ballVelocity,ownGoal_coord,keeper_dir);
        float ball2keepline_time = getTime(ballPosition,ballVelocity,keeper_center,keeper_dir);
        cv::Point2f crossPoint = ballPosition + ball2goal_time*ballVelocity;
        if(ball2keepline_time<0 || length(crossPoint-ownGoal_coord)<ownGoal_width/2)
            continue;
        struct timespec ts,te;
        clock_gettime(CLOCK_REALTIME,&ts);
        float fwd_dist = ((ballPosition + ballVelocity*ball2keepline_time)-ownGoal_coord).dot(keeper_dir);
        moveForward(fwd_dist,50);
        clock_gettime(CLOCK_REALTIME,&te);
        float move_cost = float(te.tv_nsec-ts.tv_nsec)/(1e9);
        float sleepTime = EXTRA_WAIT_TIME+ball2keepline_time-move_cost;
        if(sleepTime>0)
            usleep(sleepTime*1e6);
        moveForward(-fwd_dist,50);
    }
}

bool Robot::getBallInfo(cv::Point2f &ballVelocity,cv::Point2f &ballPosition)
{
    // to be done by zc
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

void Robot::spin(){//}std::vector<cv::Point2f> balls) {
    cv::Point2f robot_coord(x, y);
    //printf("find %d balls\n", balls.size());
    //if (balls.size() < 2)
     //   return;
    cv::Point2f ball1(0, 0);// = balls[0];
    cv::Point2f ball2(0,100);// = balls[1];
    printf("%f %f\n %f %f\n", ball1.x, ball1.y, ball2.x, ball2.y);
    //get them by some means of find balls
    float rspin =  cal_distance(ball1, ball2) / 2;
    if (rspin < ROBOT_RADIUS +BALL_RADIUS +DELTA_RADIUS)
        return;
    float disr1 =  cal_distance(ball1, robot_coord);
    //printf("%f, %f\n", rspin, disr1);
    cv::Point2f a = ball1 - robot_coord;
    cv::Point2f b = ball2 - ball1;
    float arc = M_PI - acos((a.x *b.x + a.y * b.y) / (2 *rspin * disr1));
    float turn = a.x * b.y - a.y * b.x;
    printf("%f, %f\n", a.x, a.y);
    //cvWaitKey();
    cv::Point2f pturn(x + 0.01 *a.x, y + 0.01*a.y);
    moveTo(pturn, 5);
    moveForward(disr1 * 0.99 - rspin, 20);
    if (turn > 0) {
        turnRight(90);
        moveRotate(true, rspin, 2 * M_PI - arc);
        usleep(10000);
        moveRotate(false, rspin, M_PI);
    }
    else {
        turnLeft(90);
        moveRotate(false, rspin, 2 * M_PI - arc);
        usleep(10000);
        moveRotate(true, rspin, M_PI);
    }
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
        cvCircle(wMap,cvPoint(ball_coord.x,ball_coord.y),BALL_RADIUS,CV_RGB(255,0,0),-1);
    if(ownGoalLocated)
        cvCircle(wMap,cvPoint(ownGoal_coord.x,ownGoal_coord.y),5,CV_RGB(0,0,255),-1);
    cvShowImage(RADAR_WND_NAME,wMap);
    cvSaveImage("Radar.png",wMap);
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
    if (!image)
        return ans;
    while (true) {
        ip.setBound(BALL_BOUND);
        std::vector<cv::Point3f> circles = ip.extractMulCircles(image);
        if (circles.size() <= 0)
        {
            turnRight(FIND_BALL_ANGLE);
            getImage();
            continue;
        }
        for (int i = 0; i != circles.size(); ++i) {
            cv::Point2f rCoord;
            rCoord = worldMap.coord_screen2robot(cv::Point2f(circles[i].x,circles[i].y+circles[i].z));
            ball_coord = worldMap.coord_robot2world(rCoord);
            CvRect bbox = worldMap.getMap_bbox();
            cv::Point2f ball_coord_img = world2image(ball_coord);
            if(ball_coord_img.x>=bbox.x && ball_coord_img.x<=bbox.x+bbox.width && ball_coord_img.y>=bbox.y && ball_coord_img.y<=bbox.y+bbox.height)
            {
                continue;
                //return false;
            }
            ans.push_back(ball_coord);
        }
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
    if(image)
        cvReleaseImage(&image);
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
    cvReleaseImage(&wMap);
    if(n < 10)
    {
        return false;
    }
    else
    {
        center_x = center_x/n;
        center_y = center_y/n;
        ownGoal_coord.x = center_x - (MAP_LEN>>1);
        ownGoal_coord.y = (MAP_LEN>>1)+1-center_y;
        ownGoalLocated = true;
        return true;
    }
}
