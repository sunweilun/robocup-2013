#include "motor/include/cmd.h"
#include "motor/include/tty.h"
#include "motor/include/ctrl.h"

#include <stdlib.h>
#include <sys/poll.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>

#ifndef MOTOR_UPLAYER_H
#define MOTOR_UPLAYER_H

#define TIME_DELAY 1.00
#define ARC_DELAY 0.5*3.1415926/180

int fd;
struct termios tio;

static inline void cleanup()
{
    tcsetattr(0, TCSANOW, &tio);
}

void motor_init()
{
    if ((fd = ttys_init(0)) < 0) {
        exit(-1);
    }
    else
	printf("serial init ok\n");
    ctrl_init(fd);
    tcgetattr(0, &tio);
    if (atexit(cleanup)) {
        exit(-1);
    }
}

double getRotateTime (int v, double arc)
{
    if (arc - ARC_DELAY <= 0)
        return 0;
    double dis = DIST_BETWEEN_WHEELS / 2;// rotate weel radius
    double t = (arc-ARC_DELAY) * dis / v;//ARC_DELAY用于减少惯性带来的误差
    return t * 1000000;
}

static void msend(void *buf, int n) {
    write(fd, buf, n);
}

void sendAA(int left, int right, int a = 0)
{
    char buf[16];
    buf[0] = 102;
    buf[1] = left;
    buf[2] = right;
    buf[3] = a;
    buf[4] = -101;
    msend(buf, 5);
}

void motor_turnLeft(double arc) {
    int v = 5;
    double t  = getRotateTime(v, arc);
    sendAA(-v, v, 0);
    usleep(t);
    sendAA(0, 0, 0);
    return;
}

void motor_turnRight(double arc) {
    int v = 5;
    double t  = getRotateTime(v, arc);
    sendAA(v, -v, 0);
    usleep(t);
    sendAA(0, 0, 0);
    return;
}

void goWithDistance(int dis, int spd) {//梯形加减速带来精确的直线运动
//dis is in cm; spd <= SPEED_LIMIT && spd mod 5 = 0
    if (spd > SPEED_LIMIT)
        return;
    int ori = (dis >= 0) ? 1 : -1;
    if (spd % 5 > 0)
        spd -= spd % 5;
    int fuldis = 0;
    for (int i = 1; i <= spd / 5; i++)
        fuldis += i;
    fuldis = fuldis << 1;
    if (dis >= fuldis) {
        for (int i = 5; i <= spd; i = i + 5) {
            sendAA(i * ori, i * ori, 0);
            usleep(200000);
        }
        usleep(1000000 * (dis - fuldis) / spd);
        for (int i = spd; i >= 0; i = i - 5) {
            sendAA(i * ori, i * ori, 0);
            usleep(200000);
        }
    }
    else {
        if (dis >= 6)
            goWithDistance(dis, spd - 5);
        else {
            sendAA(5 * ori, 5 * ori, 0);
            usleep(1000000 * dis / 5);
            sendAA(0, 0, 0);
            return;
        }
    }
    return;
}

void goWithSpeed(int l, int r, float time) {//time is in second
    sendAA(l, r, 0);
    usleep(time * 1000000);
    sendAA(0, 0, 0);
    return;
}

#endif
