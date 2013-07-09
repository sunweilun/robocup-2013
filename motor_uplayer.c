/*#include "motor_uplayer.h"
#include "motor/motor_control.h"

double getRotateTime (int v, double arc)
{
    double dis = 31.5 / 2;// rotate weel radius
    double t = arc * dis / v;
    return t * 1000000;
}

void turnLeft(double arc) {
    int v = 10;
    double t  = getRotateTime(v, arc);
    sendAA(-v, v, 0);
    usleep(t);
    sendAA(0, 0, 0);
}

void turnRight(double arc) {
    int v = 10;
    double t  = getRotateTime(v, arc);
    sendAA(v, -v, 0);
    usleep(t);
    sendAA(0, 0, 0);
}

void goWithSpeed(int l, int r) {
    sendAA(l, r, 0);
}*/
