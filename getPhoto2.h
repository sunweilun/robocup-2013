#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/mman.h>

#include <linux/videodev.h>

#include <sys/poll.h>
#include <stdlib.h>

extern "C" {
	#include "motor/include/jpegdec.h"
}

struct vd_capture {
	//设备信息
	int	fd;	//设备对应文件描述符
	char	*dev_name; //设备名称

	//图像采集相应参数
	unsigned int	fps;
	unsigned int 	width;
	unsigned int	height;
	unsigned int    framesize;
	int	format;
	
	//v4l2 对应的数据结构
	struct v4l2_capability	cap;
	struct v4l2_buffer	buf;
	struct v4l2_format	fmt;
	struct v4l2_requestbuffers	rbuf;

	//存放数据的变量
	void * mem[NB_BUFFERS];
//	unsigned char	*decbuffer;
	unsigned char	*framebuffer;
	unsigned char 	*rgbbuffer;
	unsigned char   *yuvbuffer;
//	unsigned char 	*tmpbuffer;

	//控制变量，用于是否退出采集
	int	quit;
	int 	streaming;
};

static vd_capture *c1, *c2;
static pthread_mutex_t ca_mutex;

int init_data(struct vd_capture * vd_cap, char *dev_name,
	      unsigned int fps,
	      unsigned int width,
	      unsigned int height,
	      int format); 
int init_device(struct vd_capture * vd_cap);
int capture(struct vd_capture * vd_cap);
int close_device(struct vd_capture * vd_cap);
int enable_device(struct vd_capture * vd_cap);
int disable_device(struct vd_capture * vd_cap);

void init_video_work(struct vd_capture* c, int framerate, char * instrument_path);
