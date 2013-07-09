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
    float camParms[8]; // 8 cam parameters
    bool legal(const IplImage* cb,int x,int y); // check if a pixel is in block
public:
    cv::Point2f coord_screen2robot(const cv::Point2f& rCoord);
    cv::Point2f coord_robot2screen(const cv::Point2f& rCoord);
    cv::Point2f coord_robot2world(const cv::Point2f& rCoord);
    cv::Point2f coord_world2robot(const cv::Point2f& wCoord);
    IplImage* getMap();
    void loadCamParms(const char* fileName);
    void setParent(Robot* robot);
    IplImage *getLines(const IplImage* img);
    void updateMap(const IplImage *img); // update map according to *img
    void showMap(const char* wndName);
    void saveMap(const char* fn);
    WorldMap();
    ~WorldMap();
    //sk add
    IplImage *getField(const IplImage*img);
    IplImage *getGate(const IplImage*img);
};

#endif
