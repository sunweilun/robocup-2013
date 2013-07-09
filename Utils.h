#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <vector>

#ifndef UTILS_H
#define UTILS_H

using namespace std;

IplImage *loadDatImage(char *fn);

IplImage* get_hsv(const IplImage* img);

#endif // UTILS_H
