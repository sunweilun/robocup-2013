#include "ImageProcessor.h"

void ImageProcessor::setBound(const cv::Point2f &h_bound,const cv::Point2f &s_bound,const cv::Point2f &v_bound)
{
    this->h_bound = h_bound;
    this->s_bound = s_bound;
    this->v_bound = v_bound;
}

bool ImageProcessor::inBound(float h,float s,float v)
{
    bool h_inbound = h>=h_bound.x && h<=h_bound.y;
    bool s_inbound = s>=s_bound.x && s<=s_bound.y;
    bool v_inbound = v>=v_bound.x && v<=v_bound.y;
    return h_inbound && s_inbound && v_inbound;
}

IplImage* ImageProcessor::eliminateBackground(const IplImage* hsv_img)
{
    IplImage* eli_image = cvCreateImage(cvGetSize(hsv_img),IPL_DEPTH_32F,3);
    memcpy(eli_image->imageData,hsv_img->imageData,4*3*hsv_img->width*hsv_img->height);
    float *hsv_data = (float*) hsv_img->imageData;
    float *eli_data = (float*) eli_image->imageData;
    for(int x=0;x<eli_image->width;x++)
    {
        bool inb = false;
        for(int y=0;!inb && y<eli_image->height;y++)
        {
            int idx = 3*(y*eli_image->width+x);
            float h = hsv_data[idx];
            float s = hsv_data[idx+1];
            float v = hsv_data[idx+2];
            inb = inBound(h,s,v);
            if(!inb)
            {
                eli_data[idx+2] = 0;
            }
        }
    }
    return eli_image;
}
bool ImageProcessor::leftBlue(IplImage* blue, int i, int j){
    int width = 320;
    int horiStep = 5;
    float* blue_data = (float*)blue->imageData;
    float h = 0, s= 0, v = 0;
    for(int k = j-1; k >=0 && k >= j-horiStep; k--){
        h = blue_data[i*width*3+k*3+0];
        s = blue_data[i*width*3+k*3+1];
        v = blue_data[i*width*3+k*3+2];
        if(!inBound(h, s, v)){
            return false;
        }
    }
    return true;
}
bool ImageProcessor::rightBlue(IplImage* blue, int i, int j){
    int width = 320;
    int horiStep = 5;
    float* blue_data = (float*)blue->imageData;
    float h = 0, s= 0, v = 0;
    for(int k = j+1; k < 320 && k <= j+horiStep; k++){
        h = blue_data[i*width*3+k*3+0];
        s = blue_data[i*width*3+k*3+1];
        v = blue_data[i*width*3+k*3+2];
        if(!inBound(h, s, v)){
            return false;
        }
    }
    return true;
}
bool ImageProcessor::stepUpJudge(IplImage* blue, int i, int j){
    int upStep = 3;
        if(i-upStep >= 0){
        if(leftBlue(blue, i-upStep, j) && rightBlue(blue, i-upStep, j))
            return true;
    }
    return false;
}

int* ImageProcessor::scanUp(IplImage* blue){
    int width = 320, height = 240;
    int* bound = new int[320];
    for(int i = 0; i < 320; i++){
        bound[i] = -1;
    }
    float* blue_data = (float*)blue->imageData;

    float h = 0, s= 0, v = 0;
    for(int j = 0; j < width; j++){
        for(int i = height-1; i >= 0; i--){
            h = blue_data[i*width*3+j*3+0];
            s = blue_data[i*width*3+j*3+1];
            v = blue_data[i*width*3+j*3+2];
            if(inBound(h, s, v)){
                if(stepUpJudge(blue, i, j)){
                    bound[j] = i;
                }
                break;
            }
        }
    }
    return bound;
}
IplImage* ImageProcessor::getBound(int *bound){
    int head = -1, tail = 320;
    int width = 320, height = 240;
    for(int j = 0; j < 320; j++){
        if(bound[j] >= 0){
            head = j;
            break;
        }
    }
    for(int j = 319; j >= 0; j--){
        if(bound[j] > 0){
            tail = j;
            break;
        }
    }
    //cout << "head: " << head << "," << bound[head] << "\ttail: " << tail << "," << bound[tail] << endl;
    IplImage* gate = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
    uchar* gate_data = (uchar*)gate->imageData;
    for(int j = 0; j < width; j++){
        int i = bound[j];
        if(i > 0 && i < 239){
            gate_data[i*width+j+0] = 255;
            gate_data[(i+1)*width+j+0] = 255;
            gate_data[(i-1)*width+j+0] = 255;
        }
    }
    return gate;
}
bool ImageProcessor::getOnlyBlue(const IplImage * hsv, IplImage * blue){
    int width = 320, height = 240;
    float* hsv_data = (float*)hsv->imageData;
    float* blue_data = (float*)blue->imageData;

    float h = 0, s = 0, v = 0;
    int n = 0;
    for(int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            h = hsv_data[i*width*3+j*3+0];
            s = hsv_data[i*width*3+j*3+1];
            v = hsv_data[i*width*3+j*3+2];
            if(inBound(h, s, v)){
                blue_data[i*width*3+j*3+0] = h;
                blue_data[i*width*3+j*3+1] = s;
                blue_data[i*width*3+j*3+2] = v;
                n++;
            }
            else{
                blue_data[i*width*3+j*3+0] = 0;
                blue_data[i*width*3+j*3+1] = 0;
                blue_data[i*width*3+j*3+2] = 0;
            }
        }
    }
    if(width*height/(n+1) <= 40)
        return true;
    return false;
}
//sk add
IplImage* ImageProcessor::deleteNoise(const IplImage*image){//输入3通道32位浮点HSV, 输出3通道32位浮点HSV
    int colFlag[320];
    for(int i = 0; i < 320; i ++){
        colFlag[i] = 0;
    }
    int width = 320, height = 240;

    IplImage* noise = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
    memcpy(noise->imageData, image->imageData, sizeof(float)*3*width*height);

    float* image_data = (float*)image->imageData;
    float* noise_data = (float*)noise->imageData;

    int deep = 0, shallow = 500, avg = 0;    //从上自下扫描高度最大的边界和最小的边界
    for(int j = 0; j < width; j++){
        bool foundGreen = false;
        for(int i = 0; i < height; i++){
            float h = image_data[i*width*3+j*3+0];
            float s = image_data[i*width*3+j*3+1];
            float v = image_data[i*width*3+j*3+2];
            if(!inBound(h, s, v)){
                noise_data[i*width*3+j*3+0] = 0;
                noise_data[i*width*3+j*3+1] = 0;
                noise_data[i*width*3+j*3+2] = 0;
            }
            else{
                //cout << "yes" << endl;
                foundGreen = true;
                avg += i;
                colFlag[j] = i;
                if(i > deep)
                    deep = i;
                if(i < shallow)
                    shallow = i;
                break;
            }
        }
        if(!foundGreen){
            avg += 239;
            colFlag[j] = 239;
            deep = 239;
        }
    }

    avg /= width;
    int avglim = avg*2/3;
    int stab_little = 2, stab_big = 30;
    int j = 1;
    while(j < width){
        int start_j = -1, end_j = -1;
        bool foundStab = false;
        if(colFlag[j] > avglim && colFlag[j-1] > avglim){
            int det_plus = colFlag[j] - colFlag[j-1];
            if(det_plus > stab_little && det_plus < stab_big){
                start_j = j;
                //cout << "start_j " << j << endl;
                //have found the start of stab
                for(int jj = j; jj < width-1; jj++){
                    int det_minus = colFlag[jj]-colFlag[jj+1];
                    if(det_minus > stab_little && det_minus < stab_big){
                        end_j = jj;
                        break;
                    }
                }
            }
            //found a stab
            if(start_j > 0 && end_j > 0){
                foundStab = true;
                double slop = (double)(colFlag[end_j+1]-colFlag[start_j-1])/(end_j-start_j+2);
                for(int k = start_j; k <= end_j; k++){
                    int oldColk = colFlag[k];
                    colFlag[k] = colFlag[start_j-1]+slop*(k-start_j+1);
                    for(int t = colFlag[k]; t < oldColk; t++){
                        noise_data[t*width*3+k*3+0] = image_data[t*width*3+k*3+0];
                        noise_data[t*width*3+k*3+1] = image_data[t*width*3+k*3+1];
                        noise_data[t*width*3+k*3+2] = image_data[t*width*3+k*3+2];
                    }
                }
            }
        }//if
        if(foundStab)
            j = end_j+1;
        else
            j++;
    }//while
    //更新colFlag
    for(int j = 0; j < width; j++){
        for(int i = 0; i < height; i++){
            float h = noise_data[i*width*3+j*3+0];
            float s = noise_data[i*width*3+j*3+1];
            float v = noise_data[i*width*3+j*3+2];
            if(h*s*v != 0){
                colFlag[j] = i;
                break;
            }
        }
    }
//    cvNamedWindow("noise_stab", CV_WINDOW_AUTOSIZE);
//    cvShowImage("noise_stab", noise);

    if(deep < 239){
        for(int j = 1; j < width; j++){
            //if(colFlag[j] < avglim && colFlag[j-1] > avglim){
            if(colFlag[j-1]-colFlag[j] > deep/5 || (colFlag[j] < avg && colFlag[j-1] > avg)){
                int start = colFlag[j-1];
                for(int i = start; i >= 0 ; i--){
                    float h = noise_data[i*width*3+j*3+0];
                    float s = noise_data[i*width*3+j*3+1];
                    float v = noise_data[i*width*3+j*3+2];
                    if(!inBound(h, s, v)){
                        for(int k = i; k >= colFlag[j]; k--){
                            noise_data[k*width*3+j*3+0] = 0;
                            noise_data[k*width*3+j*3+1] = 0;
                            noise_data[k*width*3+j*3+2] = 0;
                        }
                        if(i < 239)
                            colFlag[j] = i+1;
                        break;
                    }
                }
            }//if
        }
    }

    return noise;
}

IplImage* ImageProcessor::extractColorBlocks(const IplImage* hsv_img)
{
    IplImage* cb = cvCreateImage(cvGetSize(hsv_img),IPL_DEPTH_8U,1);
    memset(cb->imageData,0,cb->width*cb->height);
    float* hsv_data = (float*) hsv_img->imageData;
    for(int i=0;i<hsv_img->width*hsv_img->height;i++)
    {
        float h = hsv_data[i*3];
        float s = hsv_data[i*3+1];
        float v = hsv_data[i*3+2];
        if(inBound(h,s,v))
        {
            cb->imageData[i] = 255;
        }
    }
    return cb;
}

std::vector<cv::Point3f> ImageProcessor::extractCircles(const IplImage* img)
{
    return ball_detection(img,h_bound,s_bound,v_bound);
}
