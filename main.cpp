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
    robot.shoot();
    robot.radarOff();
#endif

#if CALIBTEST
    calib();
#endif
    return 0;
}
