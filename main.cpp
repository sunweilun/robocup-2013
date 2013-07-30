#include <stdio.h>
#include "Robot.h"
#include "calib.h"

#define SOCKETBOT 0
#define CALIB 0
#define ROBOTTEST 1
#define BALLTEST 0

#define WORLD_COORD_ADJUST 0

int main()
{
#if SOCKETBOT
    Robot robot;
    robot.radarOn();
    char ipStr[50];
    strcpy(ipStr, "101.5.146.224");
    int host = 9999;
    int socket_fd = robot.init_socket(ipStr, host);
    if(socket_fd == 0){
        printf("connect failed!");
        return 0;
    }
    if(socket_fd){
        robot.listenAndAct();
    }
#endif
#if ROBOTTEST
    Robot robot;
    robot.radarOn();
    robot.drawMap();
    robot.keepGoal();
    //robot.findBall();
    //robot.shoot();
    //robot.findBall();
    //robot.shoot();
    //robot.spin();
    //robot.moveRotate(true, 65, M_PI / 2);
    //robot.radarOff();


#endif

#if CALIB
    calib(true);//@parm:isRight
#endif

#if BALLTEST
    int cnt = 0;\
    ptInit();
    while (true) {
        getPhoto();
        printf("%d\n", cnt++);
        usleep(500000);
    }
#endif
    return 0;
}
