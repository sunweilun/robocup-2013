//#pragma once

#ifndef GETPHOTO_H
#define GETPHOTO_H


#include "motor/include/net.h"
#include <pthread.h>

extern "C" {
	#include "motor/include/jpegdec.h"
}

#define MAX_MJPEG_SIZE 200000
#define MAX_NET_WIDTH	320
#define MAX_NET_HEIGHT	240


static pthread_mutex_t ca_mutex;
static struct net nst[2];
static int photoCnt = 0;
static int size = 0;
static unsigned char raw_buf1[MAX_MJPEG_SIZE];
static unsigned char raw_buf2[MAX_MJPEG_SIZE];
//static unsigned char tempbuf[2][MAX_MJPEG_SIZE];
//static IplImage *image_l, *image_r;

static void* myInit(void* arg) {
	pthread_mutex_init(&ca_mutex, NULL);
	//raw_buff = raw_buff1;
	int port1 = 8000, port2 = 8001;
	nets_init(&nst[0], port1);
	nets_init(&nst[1], port2);

    while (true) {
        //int size = 0;
        //printf("3\n");
        usleep(2000);
        pthread_mutex_lock(&ca_mutex);
        if ((size = net_recv(raw_buf1, MAX_MJPEG_SIZE, &(nst[0]))) <= 0) {
            fprintf(stderr, "net_recv1 error\n");
            return (void*)0;
        }
        if ((size = net_recv(raw_buf2, MAX_MJPEG_SIZE, &(nst[1]))) <= 0) {
            fprintf(stderr, "net_recv2 error\n");
            return (void*)0;
        }
        pthread_mutex_unlock(&ca_mutex);
        //printf("4\n");
        usleep(2000);
    }
}

static void ptInit() {
    pthread_t pt;
    pthread_create(&pt, NULL, myInit, NULL);
    usleep(2000);
}

static void ptEnd() {
    net_close(&(nst[0]));
	net_close(&(nst[1]));
}

static void getPhoto(IplImage *image_l, IplImage *image_r) {
    //printf("1\n");
    usleep(2000);
	int gsize = 0;
	unsigned char tmpbuf[2][MAX_MJPEG_SIZE];

	pthread_mutex_lock(&ca_mutex);
	memcpy(tmpbuf[0], raw_buf1, MAX_MJPEG_SIZE);
	memcpy(tmpbuf[1], raw_buf2, MAX_MJPEG_SIZE);
	gsize = size;
	pthread_mutex_unlock(&ca_mutex);
    //printf("2\n");
	usleep(2000);

	struct jpeg_decompress_struct* jpeg_decompressor = newDecompressor ( MAX_NET_WIDTH );
	long rgbbuffersize = MAX_NET_WIDTH * MAX_NET_HEIGHT * 3;
	unsigned char rgbbuffer[rgbbuffersize];
	for (int i = 0; i != 2; ++i) {
		if (read_JPEG_buffer(jpeg_decompressor, tmpbuf[i], gsize, rgbbuffer, rgbbuffersize, NULL, 0) != 1) {
			fprintf(stderr, "\nerror while decoding jpeg files.\n");
			if (isfatalerror()) {
				fprintf(stderr, "\nwarning: fatal error occur. reconstructing decompress process.\n");
				deleteDecompressor(jpeg_decompressor);
				jpeg_decompressor = newDecompressor(MAX_NET_WIDTH);
			}
			return;
		}

		//char dataName[300];
		int width = 320, height = 240;
		if (i == 0) {	//left
			//sprintf(dataName, "./data/%d_l.dat", photoCnt);
			image_l = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
			/*
			int r,g,b;
		    for (int i = 0; i < width * height; i++) {
		        sprintf(image_l->imageData[3 * i], "%x", rgbbuffer[3 * i]);
		        sprintf(image_l->imageData[3 * i + 1], "%x", rgbbuffer[3 * i + 1]);
		        sprintf(image_l->imageData[3 * i + 2], "%x", rgbbuffer[3 * i + 2]);
		    }
		    */
		    memcpy(image_l->imageData, rgbbuffer, rgbbuffersize);
		} else {
			image_r = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
			memcpy(image_r->imageData, rgbbuffer, rgbbuffersize);
		}
		//sprintf(dataName, "./data/%d_r.dat", photoCnt++);
		//FILE* p = fopen(dataName, "w");
		//fprintf(p, "%d %d\n", 320, 240);
		//for (long k = 0; k < rgbbuffersize; k += 3)
		//	sprintf(p, "%.2x %.2x %.2x\n", rgbbuffer[k], rgbbuffer[k + 1], rgbbuffer[k + 2]);
		//fclose(p);
		//memcpy(tempbuf[i], rgbbuffer, rgbbuffersize);
		//image = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	    
	}
}

#endif
