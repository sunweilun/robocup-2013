#include <stdio.h>
#include "Robot.h"

int main()
{
    calib();
    Robot robot;
    robot.radarOn();
    robot.drawMap();
    robot.findBall();

    robot.radarOff();
    return 0;

}
