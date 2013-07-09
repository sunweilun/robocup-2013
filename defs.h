#define DEBUG_MAP
#define DATA_PATH "/home/csai123/桌面/RoboCup/"
#define CAM_PARMS_PATH "/home/csai123/robocup-test/calib/left-right"

#define MAP_LEN 512

#define SLEEPTIME_BEFORE_PHOTO 1000000
#define FIND_BALL_ANGLE 30
#define RADAR_WND_NAME "Radar"

#define BALL_RADIUS 10
#define ROBOT_RADIUS 20

#define GRASS_H_LB 150
#define GRASS_H_UB 210
#define GRASS_S_LB 0.4
#define GRASS_S_UB 0.7
#define GRASS_V_LB 0.5
#define GRASS_V_UB 0.8

#define LINE_H_LB 0
#define LINE_H_UB 360
#define LINE_S_LB 0
#define LINE_S_UB 0.3
#define LINE_V_LB 0.5
#define LINE_V_UB 1

#define BALL_H_LB 210
#define BALL_H_UB 253
#define BALL_S_LB 0.17
#define BALL_S_UB 0.27
#define BALL_V_LB 0.1
#define BALL_V_UB 0.5

#define GRASS_BOUND cv::Point2f(GRASS_H_LB,GRASS_H_UB),cv::Point2f(GRASS_S_LB,GRASS_S_UB),cv::Point2f(GRASS_V_LB,GRASS_V_UB)
#define LINE_BOUND cv::Point2f(LINE_H_LB,LINE_H_UB),cv::Point2f(LINE_S_LB,LINE_S_UB),cv::Point2f(LINE_V_LB,LINE_V_UB)
#define BALL_BOUND cv::Point2f(BALL_H_LB,BALL_H_UB),cv::Point2f(BALL_S_LB,BALL_S_UB),cv::Point2f(BALL_V_LB,BALL_V_UB)

#define DEBUG
