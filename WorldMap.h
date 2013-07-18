#include <stdio.h>
#include "ImageProcessor.h"
#include "defs.h"
#include "Robot.h"

#ifndef WORLDMAP_H
#define WORLDMAP_H

class Robot;

class WorldMap
{
private:
    Robot *robot; // robot that holds this map
    IplImage *wMap; // map image
    ImageProcessor ip;
    float camParms_l[8]; // 8 cam parameters
    float camParms_r[8];
    bool legal(const IplImage* cb,int x,int y); // check if a pixel is in block
public:
    cv::Point2f coord_screen2robot(const cv::Point2f& rCoord, bool isRight = true);
    cv::Point2f coord_robot2screen(const cv::Point2f& rCoord, bool isRight = true);
    cv::Point2f coord_robot2world(const cv::Point2f& rCoord);
    cv::Point2f coord_world2robot(const cv::Point2f& wCoord);
    IplImage* getMap();
    void loadCamParms_l(const char* fileName);
    void loadCamParms_r(const char* fileName);
    void setParent(Robot* robot);
    IplImage *getLines(const IplImage* img);
    void updateMap(const IplImage *img); // update map according to *img
    void showMap(const char* wndName);
    void saveMap(const char* fn);
    WorldMap();
    ~WorldMap();
    CvRect getMap_bbox();
    //sk add
    IplImage *getField(const IplImage*img);
    IplImage *getGate(const IplImage*img);
};

#endif
