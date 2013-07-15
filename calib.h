#pragma once

#include "getPhoto.h"

#define MAX_POINTS 100

void mouse_cb(int event,int x,int y,int flags,void* param)
{
    void** ptrs = (void**) param;
    CvPoint* sc = (CvPoint*) ptrs[0];
    CvPoint* rc = (CvPoint*) ptrs[1];
    int *np = (int*) ptrs[2];
    bool *next = (bool*) ptrs[3];
    IplImage *image = (IplImage*) ptrs[4];
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        printf("NewPoint:\n    (x,y):");
        sc[*np].x = x;
        sc[*np].y = y;
        cvCircle(image,sc[*np],3,CV_RGB(255,255,0));
        cvShowImage("Calib",image);
        cvWaitKey(100);
        scanf("%d %d",&rc[*np].x,&rc[*np].y);
        cvCircle(image,sc[*np],3,CV_RGB(0,0,255));
        (*np)++;
        //printf("*np=%d\n",*np);
        *next = true;
        break;
    case CV_EVENT_RBUTTONDOWN:
        *next = false;
        break;
    }
}

//*************Calculate TransMatrix Start****************

void PrintMat(CvMat *A)
{
FILE* file = fopen(CAM_PARMS_PATH,"w");


	int i, j;
	for(i = 0; i < A->rows; i++)
	{
		//putchar('\n');
		switch(CV_MAT_DEPTH(A->type))
		{
		case CV_32F:
		case CV_64F:
			for(j = 0; j < A->cols; j++)
			{
				fprintf(file, "%9.4f\n", (float)cvGetReal2D(A, i, j));
			}
			break;
		case CV_8U:
		case CV_16U:
			for(j = 0; j < A->cols; j++)
			{
				fprintf(file, "%6d\n", (int )cvGetReal2D(A, i, j));
			}
			break;
		}
	}
	//putchar('\n');
	 fclose(file);
	return;
}


void MakeMatViaRes(CvMat *MatA, CvMat *MatB, CvPoint *ImagPosition, CvPoint *RealPosition, int points)
{
	int i, j;
	double points_u[MAX_POINTS];
	double points_y[MAX_POINTS];
	double points_x[MAX_POINTS];
	double points_v[MAX_POINTS];

	for(i = 0;i< points; i++)
	{
		points_u[i] = (double)ImagPosition[i].x;
		points_v[i] = (double)ImagPosition[i].y;
		points_x[i] = (double)RealPosition[i].x;
		points_y[i] = (double)RealPosition[i].y;
	}
	for(i = 0; i < points; i++)
	{
		cvmSet(MatA, i, 0, points_u[i]);
		cvmSet(MatA, i, 1, points_v[i]);
		cvmSet(MatA, i, 2, 1);
		cvmSet(MatA, i, 6, - points_u[i] * points_x[i]);
		cvmSet(MatA, i, 7, - points_v[i] * points_x[i]);

		cvmSet(MatB, i, 0, points_x[i]);
	}
	for(i = points; i < 2 * points; i++)
	{
		cvmSet(MatA, i, 3, points_u[i - points]);
		cvmSet(MatA, i, 4, points_v[i - points]);
		cvmSet(MatA, i, 5, 1);
		cvmSet(MatA, i, 6, - points_u[i - points] * points_y[i - points]);
		cvmSet(MatA, i, 7, - points_v[i - points] * points_y[i - points]);

		cvmSet(MatB, i, 0, points_y[i - points]);
	}
	return;
}

void CalTransMat(CvMat *TranMat, int points, CvPoint *ImagPosition, CvPoint *RealPosition)
{
    printf("pointnum:%d\n", points);
	CvMat * MatA= cvCreateMat(2 * points, 8, CV_64FC1);
	CvMat * MatB= cvCreateMat(2 * points, 1, CV_64FC1);
	CvMat * MatInvA = cvCreateMat(8, 2 * points, CV_64FC1);
	cvZero(MatA);
	cvZero(MatB);
	cvZero(MatInvA);

	void PrintMat(CvMat*);
    printf("in cal\n");
	MakeMatViaRes(MatA, MatB, ImagPosition, RealPosition, points);
    printf("out cal\n");
	cvInvert(MatA, MatInvA, CV_SVD);
	cvMatMul(MatInvA, MatB, TranMat);

	return;
}
//***************Calculate TransMatrixEnd****************

void calib()
{
    CvPoint sc[100],rc[100];
    int np = 0;
    bool next = true;
    printf("in\n");
    ptInit();
    getPhoto();
    printf("out\n");
    char dp[] = DATA_PATH;
    char fn[1024];
    sprintf(fn,"%s0.dat",dp);
    IplImage* image = loadDatImage(fn);
    void* ptrs[5];
    ptrs[0] = (void*) sc;
    ptrs[1] = (void*) rc;
    ptrs[2] = (void*) &np;
    ptrs[3] = (void*) &next;
    ptrs[4] = (void*) image;
    cvNamedWindow("Calib");
    cvSetMouseCallback("Calib",mouse_cb,ptrs);
    while(next)
    {
        cvShowImage("Calib",image);
        cvWaitKey(100);
    }
    cvDestroyWindow("Calib");
    CvMat * Tran_Mat= cvCreateMat(8, 1, CV_64FC1);
    printf("np:%d\n", np);

    CalTransMat(Tran_Mat, np, sc, rc);
    printf("Transform Matrix:");
    PrintMat(Tran_Mat);
}
