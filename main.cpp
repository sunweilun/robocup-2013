#include <stdio.h>
#include "Robot.h"
#include "calib.h"

#define CALIBTEST 0
#define ROBOTTEST 1

int main()
{
#if ROBOTTEST
    Robot robot;
    robot.radarOn();
    robot.drawMap();
    robot.findBall();
    /*cv::Point2f wCoord(50, 50);
    robot.moveTo(wCoord, 30);
    wCoord.x = 100;
    wCoord.y = 200;
    robot.moveTo(wCoord, 30);*/
    robot.radarOff();
#endif

#if CALIBTEST
    calib();
#endif
    return 0;

}
