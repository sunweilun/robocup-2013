//#define DEBUG_MAP

#define DELTA_RADIUS 10
#define SHOOT_PREP_DIST 20

#define DATA_PATH "/home/csai123/桌面/RoboCup/data/"
//!!!#define CAM_PARMS_PATH "/home/csai123/robocup-test/calib/left-right"
#define CAM_PARMS_PATH "/home/csai123/桌面/RoboCup/cam_parms"

#define INVAlID_CAM_HEIGHT 50

#define MAP_LEN 512

#define SLEEPTIME_BEFORE_PHOTO 1000000
#define FIND_BALL_ANGLE 30
#define RADAR_WND_NAME "Radar"

#define BALL_RADIUS 10
#define ROBOT_RADIUS 20

#define GRASS_H_LB 150
#define GRASS_H_UB 190
#define GRASS_S_LB 0.4
#define GRASS_S_UB 0.6
#define GRASS_V_LB 0.5
#define GRASS_V_UB 0.85

#define LINE_H_LB 180
#define LINE_H_UB 380
#define LINE_S_LB 0.01
#define LINE_S_UB 0.5
#define LINE_V_LB 0.9
#define LINE_V_UB 1

#define BLUE_H_LB 209
#define BLUE_H_UB 220
#define BLUE_S_LB 0.58
#define BLUE_S_UB 0.78
#define BLUE_V_LB 0.89
#define BLUE_V_UB 1

#define BALL_H_LB 235
#define BALL_H_UB 331
#define BALL_S_LB 0.18
#define BALL_S_UB 0.495
#define BALL_V_LB 0.23
#define BALL_V_UB 0.58

#define GRASS_BOUND cv::Point2f(GRASS_H_LB,GRASS_H_UB),cv::Point2f(GRASS_S_LB,GRASS_S_UB),cv::Point2f(GRASS_V_LB,GRASS_V_UB)
#define LINE_BOUND cv::Point2f(LINE_H_LB,LINE_H_UB),cv::Point2f(LINE_S_LB,LINE_S_UB),cv::Point2f(LINE_V_LB,LINE_V_UB)
#define BALL_BOUND cv::Point2f(BALL_H_LB,BALL_H_UB),cv::Point2f(BALL_S_LB,BALL_S_UB),cv::Point2f(BALL_V_LB,BALL_V_UB)
#define BLUE_BOUND cv::Point2f(BLUE_H_LB,BLUE_H_UB),cv::Point2f(BLUE_S_LB,BLUE_S_UB),cv::Point2f(BLUE_V_LB,BLUE_V_UB)
#define BALL_DEBUG 1

#define DEBUG
