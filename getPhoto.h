#include "motor/include/net.h"
#include <pthread.h>

extern "C" {
	#include "motor/include/jpegdec.h"
}

#define MAX_MJPEG_SIZE 200000
#define MAX_NET_WIDTH	320
#define MAX_NET_HEIGHT	240

#ifndef GETPHOTO_H
#define GETPHOTO_H

static pthread_mutex_t ca_mutex;
struct net nst[2];
int photoCnt = 0;
int size = 0;
unsigned char raw_buff1[MAX_MJPEG_SIZE];
unsigned char* raw_buff;

void* myInit(void* arg);

void ptInit() {
        pthread_t pt;
        pthread_create(&pt, NULL, myInit, NULL);
        usleep(2000);
}

void* myInit(void* arg) {
	pthread_mutex_init(&ca_mutex, NULL);
	raw_buff = raw_buff1;
	int port1 = 8000, port2 = 8001;
	nets_init(&nst[0], port1);
	nets_init(&nst[1], port2);

    while (true) {
        //int size = 0;
        //printf("3\n");
        usleep(2000);
        pthread_mutex_lock(&ca_mutex);
        if ((size = net_recv(raw_buff, MAX_MJPEG_SIZE, &(nst[1]))) <= 0) {
            fprintf(stderr, "net_recv error\n");
            return (void*)0;
        }
        pthread_mutex_unlock(&ca_mutex);
        //printf("4\n");
        usleep(2000);
    }
}

/*
int getInfo(int num) {
	int size = 0;
	if ((size = net_recv(raw_buff, MAX_MJPEG_SIZE, &(nst[num]))) <= 0) {
		fprintf(stderr, "net_recv error\n");
		return -1;
	}
	struct jpeg_decompress_struct* jpeg_decompressor = newDecompressor ( MAX_NET_WIDTH );

	long rgbbuffersize = MAX_NET_WIDTH * MAX_NET_HEIGHT * 3;
	unsigned char rgbbuffer[rgbbuffersize];

	if (read_JPEG_buffer(jpeg_decompressor, raw_buff, size, rgbbuffer, rgbbuffersize, NULL, 0) != 1) {
		fprintf(stderr, "\nerror while decoding jpeg files.\n");
		if (isfatalerror()) {
			fprintf(stderr, "\nwarning: fatal error occur. reconstructing decompress process.\n");
			deleteDecompressor(jpeg_decompressor);
			jpeg_decompressor = newDecompressor(MAX_NET_WIDTH);
		}
		return -1;
	}

	char dataName[300];
	sprintf(dataName, "%d.dat", photoCnt, num);
	photoCnt++;
	FILE* p = fopen(dataName, "w");
	fprintf(p, "%d %d\n", 320, 240);
	long i;
	for (i = 0; i < rgbbuffersize; i += 3)
		fprintf(p, "%.2x %.2x %.2x\n", rgbbuffer[i], rgbbuffer[i + 1], rgbbuffer[i + 2]);
	fclose(p);
	return 0;
}
*/

void ptEnd() {
    net_close(&(nst[0]));
	net_close(&(nst[1]));
}

void getPhoto() {
    //printf("1\n");
    usleep(2000);
	int gsize = 0;
	unsigned char tmpbuf[MAX_MJPEG_SIZE];

	pthread_mutex_lock(&ca_mutex);
	memcpy(tmpbuf, raw_buff, MAX_MJPEG_SIZE);
	gsize = size;
	pthread_mutex_unlock(&ca_mutex);
    //printf("2\n");
	usleep(2000);
	struct jpeg_decompress_struct* jpeg_decompressor = newDecompressor ( MAX_NET_WIDTH );
	long rgbbuffersize = MAX_NET_WIDTH * MAX_NET_HEIGHT * 3;
	unsigned char rgbbuffer[rgbbuffersize];

	if (read_JPEG_buffer(jpeg_decompressor, tmpbuf, gsize, rgbbuffer, rgbbuffersize, NULL, 0) != 1) {
		fprintf(stderr, "\nerror while decoding jpeg files.\n");
		if (isfatalerror()) {
			fprintf(stderr, "\nwarning: fatal error occur. reconstructing decompress process.\n");
			deleteDecompressor(jpeg_decompressor);
			jpeg_decompressor = newDecompressor(MAX_NET_WIDTH);
		}
		return;
	}

	char dataName[300];
	sprintf(dataName, "%d.dat", photoCnt);
	photoCnt++;
	FILE* p = fopen(dataName, "w");
	fprintf(p, "%d %d\n", 320, 240);
	long i;
	for (i = 0; i < rgbbuffersize; i += 3)
		fprintf(p, "%.2x %.2x %.2x\n", rgbbuffer[i], rgbbuffer[i + 1], rgbbuffer[i + 2]);
	fclose(p);
}

#endif
