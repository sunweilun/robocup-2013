#define DEBUG_MAP 0
#define MIN_BALL_SPEED_TO_KEEP 20
#define KEEPER_DIST2GOAL 50
#define EXTRA_WAIT_TIME 3
#define ORI_TOL 10
#define LINE_LENGTH_LBOUND 5
#define LINE_LENGTH_SUM_LBOUND 10

#define TURN_RIGHT_OFFSET 0
#define TURN_LEFT_OFFSET 0

#define DELTA_T 50000
#define DELTA_V 5
#define SPEED_LIMIT 50

#define BBOX_DELTA 10

#define MAX_SPEED_LEVEL (SPEED_LIMIT/DELTA_V)

#define GOAL_WIDTH_DELTA 20

#define BOT_CENTER2CAM_CENTER 15

#define DELTA_RADIUS 15
#define SHOOT_PREP_DIST 60
#define TO_GOAL_DIST 40
#define DIST_BETWEEN_WHEELS 31.5

#define DATA_PATH "/home/csai123/桌面/RoboCup/data/"

//#define CAM_PARMS_PATH_RIGHT "/home/csai123/桌面/RoboCup/cam_parms_right"
//#define CAM_PARMS_PATH_LEFT "/home/csai123/桌面/RoboCup/cam_parms_left"
#define CAM_PARMS_PATH_RIGHT "cam_parms_right"
#define CAM_PARMS_PATH_LEFT "cam_parms_left"

#define INVAlID_CAM_HEIGHT 50

#define MAP_LEN 512

#define SLEEPTIME_BEFORE_PHOTO 1000
#define FIND_BALL_ANGLE 30
#define RADAR_WND_NAME "Radar"

#define BALL_RADIUS 10
#define ROBOT_RADIUS 20



#define GRASS_H_LB 150
#define GRASS_H_UB 200
#define GRASS_S_LB 0.4
#define GRASS_S_UB 0.65
#define GRASS_V_LB 0.5
#define GRASS_V_UB 0.85

#define LINE_H_LB 180
#define LINE_H_UB 360
#define LINE_S_LB 0.01
#define LINE_S_UB 0.4
#define LINE_V_LB 0.82
#define LINE_V_UB 1

#define BLUE_H_LB 209
#define BLUE_H_UB 220
#define BLUE_S_LB 0.58
#define BLUE_S_UB 0.78
#define BLUE_V_LB 0.89
#define BLUE_V_UB 1

#define BALL_H_LB 222
#define BALL_H_UB 331
#define BALL_S_LB 0.175
#define BALL_S_UB 0.495
#define BALL_V_LB 0.23
#define BALL_V_UB 0.65

#define BALLTRACKER_AREA_LBOUND 50
#define BALLTRACKER_AREA_WIDTH_LBOUND 5
#define BALLTRACKER_AREA_HEIGHT_LBOUND 5
#define BALL_MOVING_H_LB 215
#define BALL_MOVING_H_UB 270
#define BALL_MOVING_S_LB 0.2
#define BALL_MOVING_S_UB 0.55
#define BALL_MOVING_V_LB 0.2
#define BALL_MOVING_V_UB 0.7

#define GRASS_BOUND cv::Point2f(GRASS_H_LB,GRASS_H_UB),cv::Point2f(GRASS_S_LB,GRASS_S_UB),cv::Point2f(GRASS_V_LB,GRASS_V_UB)
#define LINE_BOUND cv::Point2f(LINE_H_LB,LINE_H_UB),cv::Point2f(LINE_S_LB,LINE_S_UB),cv::Point2f(LINE_V_LB,LINE_V_UB)
#define BALL_BOUND cv::Point2f(BALL_H_LB,BALL_H_UB),cv::Point2f(BALL_S_LB,BALL_S_UB),cv::Point2f(BALL_V_LB,BALL_V_UB)
#define BLUE_BOUND cv::Point2f(BLUE_H_LB,BLUE_H_UB),cv::Point2f(BLUE_S_LB,BLUE_S_UB),cv::Point2f(BLUE_V_LB,BLUE_V_UB)
#define BALL_DEBUG 1

#define DEBUG
