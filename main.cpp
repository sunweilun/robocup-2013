#include <stdio.h>
#include "Robot.h"
//#include "getPhoto.h"
#include "calib.h"

#define CALIBTEST 0
#define ROBOTTEST 1
#define BALLTEST 0

int main()
{
#if ROBOTTEST
    Robot robot;
    /*robot.radarOn();
    robot.drawMap();
    robot.findBall();
    robot.shoot();
    //robot.findBall();
    //robot.shoot();
    robot.radarOff();
    */
        std::vector<cv::Point2f> ans = robot.findMulBall();
        //printf("1\n");
        for (int i = 0; i != ans.size(); ++i)
            printf("x = %f, y = %f\n", ans[i].x, ans[i].y);
        //robot.spin();
#endif

#if CALIBTEST
    calib();
#endif

#if BALLTEST
    int cnt = 0;
    ptInit();
    while (true) {
        getPhoto();
        printf("%d\n", cnt++);
        usleep(500000);
    }
#endif
    return 0;
}
