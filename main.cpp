#include <stdio.h>
#include "Robot.h"
#include "calib.h"

#define CALIBTEST 0
#define ROBOTTEST 1
#define BALLTEST 0

int main()
{
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

#if CALIBTEST
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
