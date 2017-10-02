#ifndef STEREO_MATCH_
#define STEREO_MATCH_

#include <stdlib.h>
#include <math.h>
#include <time.h> 
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>      
#include <opencv2/imgproc/imgproc.hpp>      
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/contrib/contrib.hpp> 
#include <opencv2/highgui/highgui.hpp>  
//#include <opencv2/legacy/legacy.hpp>
//#include <opencv2/core/internal.hpp>
//#include <cv.h>  
//#include <cxcore.h>  
#include <iostream>  
using namespace std;
using namespace cv;

#define DOMAIN    64
#define WndWidth  9
#define CODEWIDTH 80
typedef struct
{
	int codebit[80];
}CodeStruct;

void SAD(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity);//像素点灰度差的绝对值和

void SSD(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity);//像素点灰度差的平方和

void Census(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity);//是根据窗口内中心像素灰度与其余像素灰度值的大小关系得一串位码，位码长度等于窗口内像素个数减一

void Rank(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity);//是以窗口内灰度值小于中心像素灰度值的像素个数来代替中心像素的灰度值

void CensusTransform(unsigned char *Image, CodeStruct Code[]);

int HammingDist(CodeStruct Code1, CodeStruct Code2);

void RankTransform(unsigned char *Image, unsigned char *RankValue);

void stereomatch(cv::Mat img1, cv::Mat img2);

//void BM(IplImage * img1, IplImage * img2);

//void SGBM(IplImage * img1, IplImage * img2);

//void GC(IplImage * img1, IplImage * img2);


#endif