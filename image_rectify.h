#ifndef IMAGE_RECTIFY_
#define IMAGE_RECTIFY_
#include "stereo_match.h"
#include "matrix_cal.h"

void ComputeDisparity(CvMat imgl_rec, CvMat imgr_rec, CvSize imSize);

void get_subMat(Mat imgl_rectified, Mat imgr_rectified, Mat &imgl_stereo, Mat &imgr_stereo);

void get_subMat_census(Mat img1_rectified, Mat img2_rectified, Mat &img1_stereo, Mat &img2_stereo);

void stereo_image_rectify(IplImage *image_L, IplImage *image_R, CvSize imageSize, Mat &imgL_rectified, Mat &imgR_rectified);

void monocular_image_rectify(IplImage *image1, IplImage *image2, CvMat intrinsic, CvMat rotation, CvMat translation, CvMat distortion, CvSize imageSize, Mat &imgL_rectified, Mat &imgR_rectified);

#endif