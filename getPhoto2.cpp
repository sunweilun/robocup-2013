#include "getPhoto2.h"

int init_data(struct vd_capture *vd_cap, 
	       char *dev_name, 
	       unsigned int fps,
	       unsigned int width, 
	       unsigned int height,
	       int format) {
	if(vd_cap == NULL || dev_name == NULL) return -1;
	if(fps <= 0 || width <= 0 || height <= 0) return -1;
	vd_cap->quit=0;
	vd_cap->streaming=0;
	vd_cap->dev_name=strdup(dev_name);
	vd_cap->fps=fps;
	vd_cap->width=width;
	vd_cap->height=height;
	vd_cap->format=format;
	vd_cap->framesize=((vd_cap->width*vd_cap->height) << 1);
	//根据图片格式，进行不同的处理，yuyv的格式需要的是
	//两个像素对应的是四个bytes.
	switch(vd_cap->format) {
	case V4L2_PIX_FMT_MJPEG:
		//vd_cap->framebuffer=(unsigned char*) calloc(1, (size_t)vd_cap->framesize);
		//break;
	case V4L2_PIX_FMT_YUYV:
		vd_cap->framebuffer=(unsigned char*) calloc(1, (size_t) vd_cap->framesize);
		break;
	default:
		printf("fatal error!");
		goto fatal;
		break;
	};
	if(!vd_cap->framebuffer) goto fatal;
	return 0;
	fatal:
		free(vd_cap->dev_name);
		return -1;
}

int init_device(struct vd_capture *vd_cap) {
	int rval=0;
	//验证参数的合法性.需要指定的设备存在
	if(vd_cap==NULL || vd_cap->dev_name==NULL) return -1;
	//打开视频设备，获得相应的描述符
	if((vd_cap->fd = open(vd_cap->dev_name, O_RDWR)) == -1) {
		perror("Error when open the device\n");
		exit(1);
	}
	//开始查询设备的 capability相应的属性
	memset(&vd_cap->cap, 0, sizeof(struct v4l2_capability));
	rval=ioctl(vd_cap->fd, VIDIOC_QUERYCAP, &vd_cap->cap);
	if(rval < 0) {
		printf("Error when query the capability of the device %s\n", vd_cap->dev_name);
		goto error;
	}

	//逐项验证capability的属性，看是否符合采集的要求
	if((vd_cap->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
		printf("device %s doesn't support capture\n", vd_cap->dev_name);
		goto error;
	}
	if(!(vd_cap->cap.capabilities & V4L2_CAP_STREAMING)) {
		printf("device %s doesn't support streaming\n", vd_cap->dev_name);
		goto error;
	}
	//设置相应的属性值,即开始进行
	//设备的初始化工作
	
	//设置格式
	memset(&vd_cap->fmt, 0, sizeof(struct v4l2_format));
	vd_cap->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd_cap->fmt.fmt.pix.width = vd_cap->width;
	vd_cap->fmt.fmt.pix.height = vd_cap->height;
	vd_cap->fmt.fmt.pix.pixelformat = vd_cap->format;
	vd_cap->fmt.fmt.pix.field = V4L2_FIELD_ANY;
	rval = ioctl(vd_cap->fd, VIDIOC_S_FMT, &vd_cap->fmt);
	if(rval < 0) {
		printf("unable to set the format for the device %s\n", vd_cap->dev_name);
		goto error;
	}
	if((vd_cap->width != vd_cap->fmt.fmt.pix.width) ||
	   (vd_cap->height!= vd_cap->fmt.fmt.pix.height)) {
		vd_cap->width = vd_cap->fmt.fmt.pix.width;
		vd_cap->height= vd_cap->fmt.fmt.pix.height;
	 }
	//设置码率
	struct v4l2_streamparm * setfps;
	setfps = (struct v4l2_streamparm *)calloc(1, sizeof(struct v4l2_streamparm));
	memset(setfps, 0, sizeof(struct v4l2_streamparm));
	setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps->parm.capture.timeperframe.numerator = 1;
	setfps->parm.capture.timeperframe.denominator = vd_cap->fps;
	rval = ioctl(vd_cap->fd, VIDIOC_S_PARM, setfps);

	//申请缓冲区
	memset(&vd_cap->rbuf, 0, sizeof(struct v4l2_requestbuffers));
	vd_cap->rbuf.count = NB_BUFFERS;
	vd_cap->rbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd_cap->rbuf.memory = V4L2_MEMORY_MMAP;

	rval = ioctl(vd_cap->fd, VIDIOC_REQBUFS, &vd_cap->rbuf);
	if(rval < 0) {
		printf("Unable to allocate buffers in the device %s\n", vd_cap->dev_name);
		goto error;
	}

	//将主存的空间和设备的内存空间进行映射
	int i=0;
	for (i=0;i<NB_BUFFERS;i++) {
		memset(&vd_cap->buf, 0, sizeof(struct v4l2_buffer));
		vd_cap->buf.index = i;
		vd_cap->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd_cap->buf.memory = V4L2_MEMORY_MMAP;
		rval = ioctl(vd_cap->fd, VIDIOC_QUERYBUF, &vd_cap->buf);
		if(rval < 0) {
			printf("error when map the buffers: %d \n", errno);
			goto error;
		}

		//开始映射
		vd_cap->mem[i] = mmap(0, //表示可从主存的任意位置开始
				vd_cap->buf.length, PROT_READ, MAP_SHARED, vd_cap->fd,
				vd_cap->buf.m.offset);
		if(vd_cap->mem[i] == MAP_FAILED) {
			printf("map failed\n");
			goto error;
		}
	}
	//将buffer加入队列
	for(i=0;i<NB_BUFFERS;i++) {
		memset(&vd_cap->buf, 0, sizeof(struct v4l2_buffer));
		vd_cap->buf.index = i;
		vd_cap->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vd_cap->buf.memory = V4L2_MEMORY_MMAP;
		rval = ioctl(vd_cap->fd, VIDIOC_QBUF, &vd_cap->buf);
		if (rval < 0) {
			printf("error when queue the buffers\n");
			goto error;
		}
	}
	return 0;
	error:
		close(vd_cap->fd);
		return -1;
}

int capture(struct vd_capture *vd_cap) {
	int rval=0, capturedsize=0;
	if(!vd_cap->streaming)
	if(enable_device(vd_cap)) goto errcap;
	
	memset(&vd_cap->buf, 0, sizeof(struct v4l2_buffer));
	vd_cap->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vd_cap->buf.memory = V4L2_MEMORY_MMAP;
	rval = ioctl(vd_cap->fd, VIDIOC_DQBUF, &vd_cap->buf);
	if (rval < 0) {
		printf("error when dequeue the buffer: %d \n", errno);
		goto errcap;
	}
	//开始采集视频数据
	capturedsize=vd_cap->buf.bytesused;
	if(capturedsize > vd_cap->framesize) capturedsize=vd_cap->framesize;
	switch(vd_cap->format) {
	case V4L2_PIX_FMT_MJPEG: 
		if(vd_cap->buf.bytesused<0xaf) return 0;
		//memcpy(vd_cap->framebuffer, vd_cap->mem[vd_cap->buf.index], (size_t) vd_cap->buf.bytesused);
		/*if(jpeg_decode(&vd_cap->decbuffer, vd_cap->tmpbuffer, &vd_cap->width, &vd_cap->height)<0) {
			printf("Error when decode the jpeg pic\n");
			goto errcap;
		}
		yuv422to420p(vd_cap->decbuffer, vd_cap->framebuffer, vd_cap->width, vd_cap->height);*/
		//break;
	case V4L2_PIX_FMT_YUYV:
		if(vd_cap->buf.bytesused > vd_cap->framesize) 
			memcpy(vd_cap->framebuffer, vd_cap->mem[vd_cap->buf.index], (size_t) vd_cap->framesize);
		else memcpy(vd_cap->framebuffer, vd_cap->mem[vd_cap->buf.index], (size_t)vd_cap->buf.bytesused);
		break;
	default:
		goto errcap;
		break;
	};
	rval = ioctl(vd_cap->fd, VIDIOC_QBUF, &vd_cap->buf);
	if(rval < 0) {
		printf("error when requeue the buffer: %d \n", errno);
		goto errcap;
	}
	return capturedsize;
	errcap:
		vd_cap->quit=0;
		return -1;
}

/* close_device()
 *
 * 关闭视频设备
 */
int close_device(struct vd_capture * vd_cap) {
	if (vd_cap->streaming) disable_device(vd_cap);
	//if (vd_cap->decbuffer) free(vd_cap->decbuffer);
	if (vd_cap->fd) close(vd_cap->fd);
	free(vd_cap->framebuffer);
	vd_cap->framebuffer=NULL;
	//vd_cap->decbuffer=NULL;
	free(vd_cap->dev_name);
	return 0;
}

/* enable_device()
 *
 * 使设备工作的函数，即使能作用
 */

int enable_device(struct vd_capture * vd_cap) {
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int rval;
	
	rval=ioctl(vd_cap->fd, VIDIOC_STREAMON, &type);
	if(rval < 0) {
		printf("error when enable the device: %d \n", errno);
		return rval;
	}
	vd_cap->streaming = 1;
	return 0;
}

/*disable_device()
 *
 * 使设备无法工作的函数，即起到禁止的作用
 */
int disable_device(struct vd_capture * vd_cap) {
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	int rval;
	rval=ioctl(vd_cap->fd, VIDIOC_STREAMOFF, &type);
	if(rval < 0) {
		printf("error when disable the device: %d \n", errno);
		return rval;
	}
	vd_cap->streaming = 0;
	return 0;
}

void init_video_work(struct vd_capture* c, int framerate, char * instrument_path) {
	if(c == NULL) {
		//mylogfd(2,"[Version] When get the memory for vd_cap\n");
		exit(1);
	}
	if(init_data(c, instrument_path, framerate, 320, 240, V4L2_PIX_FMT_MJPEG)==-1) {
		//mylogfd(2,"[Version] When initial the data struct\n");
		exit(1);
	}
	//调用init_device()进行初始化视频硬件环境
	if(init_device(c) < 0) {
		//mylogfd(2,"[Version] When initial the device %s\n",instrument_path);
		exit(1);
	}
}

void ptInit() {
	pthread_mutex_init(&ca_mutex, NULL);

	c1 = (struct vd_capture*) calloc(1, sizeof(struct vd_capture));
	init_video_work(c1, framerate, "/dev/video0");
	c2 = (struct vd_capture*) calloc(1, sizeof(struct vd_capture));
	init_video_work(c2, framerate, "/dev/video1");

	int captured1, captured2;
	while(!c1->quit && !c2->quit) {
		//开始采集视频数据
		//waiting
		usleep(2000);
		pthread_mutex_lock(&ca_mutex);
		if((captured1=capture(c1)) < 0) {
			break;
		}
		if((captured2=capture(c2)) < 0) {
			break;
		}
		pthread_mutex_unlock(&ca_mutex);
		usleep(2000);
	}
}

void getPhoto2(IplImage *image_l, IplImage *image_r) {
	unsigned char tmpbuf[2][250000];

	usleep(2000);
	pthread_mutex_lock(&ca_mutex);
	memcpy(tmpbuf[0], c1->framebuffer, (size_t)c1->framesize);
	memcpy(tmpbuf[1], c2->framebuffer, (size_t)c2->framesize);
	pthread_mutex_lock(&ca_mutex);
	usleep(2000);
	
	struct jpeg_decompress_struct* jpeg_decompressor = newDecompressor ( MAX_NET_WIDTH );
	long rgbbuffersize = 320*240*3;
	unsigned char rgbbuffer[rgbbuffersize];
	for (int i = 0; i != 2; ++i) {
		if (read_JPEG_buffer(jpeg_decompressor, tmpbuf[i], 320*240*2, rgbbuffer, rgbbuffersize, NULL, 0) != 1) {
			fprintf(stderr, "\nerror while decoding jpeg files.\n");
			if (isfatalerror()) {
				fprintf(stderr, "\nwarning: fatal error occur. reconstructing decompress process.\n");
				deleteDecompressor(jpeg_decompressor);
				jpeg_decompressor = newDecompressor(MAX_NET_WIDTH);
			}
			return;
		}

		if (i == 0) {	//left
		    memcpy(image_l->imageData, rgbbuffer, rgbbuffersize);
		} else {	//right
			memcpy(image_r->imageData, rgbbuffer, rgbbuffersize);
		}
	}
}