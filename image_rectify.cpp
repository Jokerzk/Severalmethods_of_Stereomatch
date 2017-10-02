#include "image_rectify.h"

double caml_intrinsic[9] = { 439.82920402256644, 0, 314.01121535161167, 0, 440.36608099367356, 244.7449018100555, 0, 0, 1 };
double camr_intrinsic[9] = { 440.74035858064656, 0, 318.1932757692425, 0, 441.07976975874016, 238.10174126102112, 0, 0, 1 };
double cam0_intrinsic[9] = { 4152.073, 0, 1288.147, 0, 4152.073, 973.571, 0, 0, 1 };
double cam1_intrinsic[9] = { 4152.073, 0, 1501.231, 0, 4152.073, 973.571, 0, 0, 1 };
double rotation[9] = { 0.9999025039640448, -0.008294064949866559, 0.011233479115471226, 0.0081572314432995, 0.9998926404116564, 0.01217239605761272, -0.011333231737275079, -0.012079575208190673, 0.9998628114502388 };
//double translation[3] = { -0.1282294115692713, -0.0011493263301578485, -0.0004523103345721074 };
double translation[3] = { -0.1282294115692713, -0.0011493263301578485, -0.0004523103345721074 };
//double rotation[9] = { 1,0,0,0,1,0,0,0,1};
//double translation[3] = { 0.176252, 0, 0 };
double caml_distortion[5] = { 0.0027952110681112047, -0.03242641428960264, 0.0007953171465510378, 0.0007002791454423, 0 };
double camr_distortion[5] = { 0.012615173559998458, -0.03617723460511374, -0.001949689858536896, 0.003034887537942637, 0 };
double cam0_distortion[5] = { 0, 0, 0, 0, 0 };
double cam1_distortion[5] = { 0, 0, 0, 0, 0 };

void stereo_image_rectify(IplImage *image_L, IplImage *image_R, CvSize imageSize, Mat &imgL_rectified, Mat &imgR_rectified)
{
	//读入立体标定参数；
	CvMat intri_L = cvMat(3, 3, CV_64FC1, caml_intrinsic);
	CvMat intri_R = cvMat(3, 3, CV_64FC1, camr_intrinsic);
	CvMat Rotation = cvMat(3, 3, CV_64FC1, rotation);
	//cvInvert(&Rotation, &Rotation, CV_LU);
	CvMat Translation = cvMat(3, 1, CV_64FC1, translation);

	CvMat distor_L = cvMat(1, 5, CV_64FC1, caml_distortion);
	CvMat distor_R = cvMat(1, 5, CV_64FC1, camr_distortion);

	//初始化；
	CvMat* mxl = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* myl = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* mxr = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* myr = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* img1r = cvCreateMat(imageSize.height, imageSize.width, CV_8U);
	CvMat* img2r = cvCreateMat(imageSize.height, imageSize.width, CV_8U);
	CvMat* disp = cvCreateMat(imageSize.height, imageSize.width, CV_16S);
	CvMat* vdisp = cvCreateMat(imageSize.height, imageSize.width, CV_8U);
	CvMat* pair = cvCreateMat(imageSize.height, imageSize.width * 2, CV_8UC3);

	double _Rl[3][3], _Rr[3][3], _Pl[3][4], _Pr[3][4];
	CvMat Rl = cvMat(3, 3, CV_64FC1, _Rl);
	CvMat Rr = cvMat(3, 3, CV_64FC1, _Rr);
	CvMat Pl = cvMat(3, 4, CV_64FC1, _Pl);
	CvMat Pr = cvMat(3, 4, CV_64FC1, _Pr);

	cvStereoRectify(&intri_L, &intri_R, &distor_L, &distor_R, imageSize, &Rotation, &Translation, &Rl, &Rr, &Pl, &Pr, 0, 0, -1.0, cvSize(0, 0), (CvRect*)0, (CvRect*)0);//get Rotation_l/r and calibrated camera matrix
	cvInitUndistortRectifyMap(&intri_L, &distor_L, &Rl, &Pl, mxl, myl);// camera calibration
	cvInitUndistortRectifyMap(&intri_R, &distor_R, &Rl, &Pr, mxr, myr);

	CvMat part_ori;

	cvGetCols(pair, &part_ori, 0, imageSize.width);
	cvCvtColor(image_L, &part_ori, CV_GRAY2BGR);

	cvGetCols(pair, &part_ori, imageSize.width, imageSize.width * 2);
	cvCvtColor(image_R, &part_ori, CV_GRAY2BGR);


	for (int j = 0; j < imageSize.height; j += 44)
	{
		cvLine(pair, cvPoint(0, j), cvPoint(imageSize.width * 2, j), CV_RGB(255, 0, 0));
	}

	CvMat *pair_show_ori = cvCreateMat(cvGetSize(pair).height, cvGetSize(pair).width, CV_8UC3);

	cvResize(pair, pair_show_ori);

	cvNamedWindow("originalImage", 1);
	cvShowImage("originalImage", pair_show_ori);

	CvMat part;
	cvRemap(image_L, img1r, mxl, myl);
	cvRemap(image_R, img2r, mxr, myr);

	IplImage *m_imggraylr, *m_imggrayrr;
	m_imggraylr = cvCreateImage(imageSize, 8, 1);
	m_imggrayrr = cvCreateImage(imageSize, 8, 1);

	//将CvMat格式转化为IplImage格式
	cvConvert(img1r, m_imggraylr);
	cvConvert(img2r, m_imggrayrr);

	//然后进行灰度图像向彩色图像的转换
	IplImage * m_imglr = cvCreateImage(imageSize, 8, 3);
	IplImage * m_imgrr = cvCreateImage(imageSize, 8, 3);

	cvCvtColor(m_imggraylr, m_imglr, CV_GRAY2BGR);
	cvCvtColor(m_imggrayrr, m_imgrr, CV_GRAY2BGR);

	cvGetCols(pair, &part, 0, imageSize.width);
	cvCvtColor(img1r, &part, CV_GRAY2BGR);

	cvGetCols(pair, &part, imageSize.width, imageSize.width * 2);
	cvCvtColor(img2r, &part, CV_GRAY2BGR);


	for (int j = 0; j < imageSize.height; j += 44)
	{
		cvLine(pair, cvPoint(0, j), cvPoint(imageSize.width * 2, j), CV_RGB(0, 255, 0));
	}

	CvMat *pair_show = cvCreateMat(cvGetSize(pair).height, cvGetSize(pair).width, CV_8UC3);

	cvResize(pair, pair_show);

	cvNamedWindow("StereoRectifyImage", 1);

	cvShowImage("StereoRectifyImage", pair_show);

	imgL_rectified = img1r;
	imgR_rectified = img2r;
}

void monocular_image_rectify(IplImage *image1, IplImage *image2, CvMat intrinsic, CvMat rotation, CvMat translation, CvMat distortion, CvSize imageSize, Mat &imgL_rectified, Mat &imgR_rectified)
{
	//初始化；
	CvMat* mx1 = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* my1 = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* img1 = cvCreateMat(imageSize.height, imageSize.width, CV_8U);
	CvMat* mx2 = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* my2 = cvCreateMat(imageSize.height, imageSize.width, CV_32F);
	CvMat* img2 = cvCreateMat(imageSize.height, imageSize.width, CV_8U);
	CvMat* pair = cvCreateMat(imageSize.height, imageSize.width * 2, CV_8UC3);

	double _R1[3][3], _P1[3][4], _R2[3][3], _P2[3][4];
	CvMat R1 = cvMat(3, 3, CV_64FC1, _R1);
	CvMat P1 = cvMat(3, 4, CV_64FC1, _P1);
	CvMat R2 = cvMat(3, 3, CV_64FC1, _R2);
	CvMat P2 = cvMat(3, 4, CV_64FC1, _P2);

	cvStereoRectify(&intrinsic, &intrinsic, &distortion, &distortion, imageSize, &rotation, &translation, &R1, &R2, &P1, &P2, 0, 1024, -1.0, cvSize(0, 0), (CvRect*)0, (CvRect*)0);//get Rotation_l/r and calibrated camera matrix
	cvInitUndistortRectifyMap(&intrinsic, &distortion, &R1, &P1, mx1, my1);// camera calibration
	cvInitUndistortRectifyMap(&intrinsic, &distortion, &R2, &P2, mx2, my2);

	CvMat part_ori;

	cvGetCols(pair, &part_ori, 0, imageSize.width);
	cvCvtColor(image1, &part_ori, CV_GRAY2BGR);
	cvGetCols(pair, &part_ori, imageSize.width, imageSize.width * 2);
	cvCvtColor(image2, &part_ori, CV_GRAY2BGR);


	for (int j = 0; j< imageSize.height; j += 44)
	{
		cvLine(pair, cvPoint(0, j), cvPoint(imageSize.width * 2, j), CV_RGB(255, 0, 0));
	}

	CvMat *pair_show_ori = cvCreateMat(cvGetSize(pair).height, cvGetSize(pair).width, CV_8UC3);

	cvResize(pair, pair_show_ori);
	cvNamedWindow("originalImage", 1);
	cvShowImage("originalImage", pair_show_ori);

	CvMat part;
	cvRemap(image1, img1, mx1, my1);
	cvRemap(image2, img2, mx2, my2);

	IplImage *m_imggraylr, *m_imggrayrr;
	m_imggraylr = cvCreateImage(imageSize, 8, 1);
	m_imggrayrr = cvCreateImage(imageSize, 8, 1);

	//将CvMat格式转化为IplImage格式
	cvConvert(img1, m_imggraylr);
	cvConvert(img2, m_imggrayrr);

	//然后进行灰度图像向彩色图像的转换
	IplImage * m_imglr = cvCreateImage(imageSize, 8, 3);
	IplImage * m_imgrr = cvCreateImage(imageSize, 8, 3);
	cvCvtColor(m_imggraylr, m_imglr, CV_GRAY2BGR);
	cvCvtColor(m_imggrayrr, m_imgrr, CV_GRAY2BGR);

	/*cvNamedWindow("imglr", 1);
	cvNamedWindow("imgrr", 1);
	cvShowImage("imglr", m_imglr);
	cvShowImage("imgrr", m_imgrr);*/

	cvGetCols(pair, &part, 0, imageSize.width);
	cvCvtColor(img1, &part, CV_GRAY2BGR);
	cvGetCols(pair, &part, imageSize.width, imageSize.width * 2);
	cvCvtColor(img2, &part, CV_GRAY2BGR);

	for (int j = 0; j< imageSize.height; j += 44)
	{
		cvLine(pair, cvPoint(0, j), cvPoint(imageSize.width * 2, j), CV_RGB(0, 255, 0));
	}

	CvMat *pair_show = cvCreateMat(cvGetSize(pair).height, cvGetSize(pair).width, CV_8UC3);

	cvResize(pair, pair_show);
	cvNamedWindow("StereoRectifyImage", 1);
	cvShowImage("StereoRectifyImage", pair_show);

	imgL_rectified = img1;
	imgR_rectified = img2;
	cvWaitKey(0);
}

void get_subMat(Mat img1_rectified, Mat img2_rectified, Mat &img1_stereo, Mat &img2_stereo)		//从校正的左右两幅图片中获取中间N行，生成一行用于立体匹配生成视差图；
{
	assert(img1_rectified.rows == img2_rectified.rows || img1_rectified.cols == img2_rectified.cols);
	int height = img1_rectified.rows;
	int width = img1_rectified.cols;
	cv::Mat _img1_stereo = Mat(1, width, CV_64FC1, Scalar(0));
	cv::Mat _img2_stereo = Mat(1, width, CV_64FC1, Scalar(0));
	for (int i = height / 2 + 0.5 - 20; i <= height / 2 + 20.5; i++)
	{
		for (int j = 0; j < width; j += 1)
		{
			_img1_stereo.at<double>(0, j) = _img1_stereo.at<double>(0, j) + img1_rectified.data[i*width + j];
			_img2_stereo.at<double>(0, j) = _img2_stereo.at<double>(0, j) + img2_rectified.data[i*width + j];

		}
	}
	_img1_stereo.convertTo(_img1_stereo, CV_8UC1);
	_img2_stereo.convertTo(_img2_stereo, CV_8UC1);

	cv::imshow("_imgl_stereo", _img1_stereo);
	cv::imshow("_imgr_stereo", _img2_stereo);
	waitKey();

	img1_stereo = _img1_stereo / 40;
	img2_stereo = _img2_stereo / 40;
}

void get_subMat_census(Mat img1_rectified, Mat img2_rectified, Mat &img1_stereo, Mat &img2_stereo)		//生成三行像素,可用于census立体匹配;
{
	assert(img1_rectified.rows == img2_rectified.rows || img1_rectified.cols == img2_rectified.cols);
	int height = img1_rectified.rows;
	int width = img1_rectified.cols;
	cv::Mat _img1_stereo = Mat(3, width, CV_64FC1, Scalar(0));
	cv::Mat _img2_stereo = Mat(3, width, CV_64FC1, Scalar(0));
	for (int j = 0; j < width; j += 1)
	{
		for (int i = height / 2 + 0.5 + 10; i < height / 2 + 30.5; i++)
		{
			_img1_stereo.at<double>(0, j) = _img1_stereo.at<double>(0, j) + img1_rectified.data[i*width + j];
			_img1_stereo.at<double>(0, j) = _img1_stereo.at<double>(0, j) + img1_rectified.data[i*width + j];
		}
		for (int i = height / 2 + 0.5 - 10; i < height / 2 + 10; i++)
		{
			_img1_stereo.at<double>(1, j) = _img1_stereo.at<double>(1, j) + img1_rectified.data[i*width + j];
			_img1_stereo.at<double>(1, j) = _img1_stereo.at<double>(1, j) + img1_rectified.data[i*width + j];
		}
		for (int i = height / 2 + 0.5 - 30; i < height / 2 - 10; i++)
		{
			_img1_stereo.at<double>(2, j) = _img1_stereo.at<double>(1, j) + img1_rectified.data[i*width + j];
			_img1_stereo.at<double>(2, j) = _img1_stereo.at<double>(1, j) + img1_rectified.data[i*width + j];
		}
	}
	_img1_stereo.convertTo(_img1_stereo, CV_8UC1);
	_img2_stereo.convertTo(_img2_stereo, CV_8UC1);
	img1_stereo = _img1_stereo / 20;
	img2_stereo = _img2_stereo / 20;

	cv::imshow("img1_stereo", img1_stereo);
	cv::imshow("img2_stereo", img2_stereo);
	waitKey();
}