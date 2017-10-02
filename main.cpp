#include "image_rectify.h"
#include "matrix_cal.h"
#include "stereo_match.h"

//#define MONOCULAR
#define STEREO

double quaternion1[4] = { 0.2042, 0.1046, 0.0006, 0.9733 };
double quaternion2[4] = { 0.2041, 0.0835, -0.0039, 0.9753 };
//double quaternion1[4] = { -0.3578, -0.0967, 0.0147, -0.9286 };

//double quaternion1[4] = { -0.3620, -0.1830, 0.0024, -0.9140 };
//double quaternion2[4] = { -0.3538, -0.0062, 0.0031, -0.9352 };

//double location1[3] = { 22.5966816, 113.9956970, 73.68 };
//double location2[3] = { 22.5965023, 113.9957886, 73.65 };

double location1[3] = { 90.7, -41.56, 73.68 };
double location2[3] = { 70.8, -31.84, 73.65 };

//double camr_intrin[9] = { 439.82920402256644, 0, 314.01121535161167, 0, 440.36608099367356, 244.7449018100555, 0, 0, 1 };

double camr_intrin[9] = { 440.74035858064656, 0, 318.1932757692425, 0, 441.07976975874016, 238.10174126102112, 0, 0, 1 };
double camr_distort[5] = { 0.012615173559998458, -0.03617723460511374, -0.001949689858536896, 0.003034887537942637, 0 };
//double camr_distort[5] = { 0, 0, 0, 0, 0 };

int main(int argc, char **argv)
{
	string img1_str,img2_str;
	if (argc <= 1)
	{
		printf("plz input a couple of images\n");
		cin >> img1_str;
		cin >> img2_str;
	}
	else if (argc == 2)
	{
		img1_str = argv[1];
		printf("plz input another image\n");
		cin >> img2_str;
	}
	else
	{
		img1_str = argv[1];
		img2_str = argv[2];
	}


#ifdef MONOCULAR

	IplImage *img_1 = cvLoadImage(img1_str.c_str(), 0);
	IplImage *img_2 = cvLoadImage(img2_str.c_str(), 0);
	CvSize imgSize = { 640, 480 };

	CvMat Rotation, Translation;
	get_matrix_from_log(quaternion1, quaternion2, location1, location2, Rotation, Translation);
	CvMat intri_R = cvMat(3, 3, CV_64FC1, camr_intrin);
	CvMat distor_R = cvMat(1, 5, CV_64FC1, camr_distort);
	cv::Mat imgrect_1, imgrect_2;
	monocular_image_rectify(img_1, img_2, intri_R, Rotation, Translation, distor_R, imgSize, imgrect_1, imgrect_2);

	cv::Mat euler1 = get_Euler_from_Quaternion(quaternion1);
	cv::Mat euler2 = get_Euler_from_Quaternion(quaternion2);
	euler1 = 180 / PI*euler1;
	euler2 = 180 / PI*euler2;
	cout << euler1 << endl;
	cout << euler2 << endl;
	
	IplImage *img1 = cvCreateImage(imgSize, img_1->depth, img_1->nChannels);
	IplImage *img2 = cvCreateImage(imgSize, img_2->depth, img_2->nChannels);
	//旋转中心为图像中心
	CvPoint2D32f center;
	center.x = double(img_1->width / 2.0 + 0.5);
	center.y = double(img_1->height / 2.0 + 0.5);
	//计算二维旋转的仿射变换矩阵
	double m1[6], m2[6];
	CvMat M1 = cvMat(2, 3, CV_32F, m1);
	CvMat M2 = cvMat(2, 3, CV_32F, m2);
	cv2DRotationMatrix(center, -euler1.at<double>(0, 0), 1, &M1);
	cv2DRotationMatrix(center, -euler2.at<double>(0, 0), 1, &M2);
	cvWarpAffine(img_1, img1, &M1, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	cvWarpAffine(img_2, img2, &M2, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

	CvMat* pair1 = cvCreateMat(imgSize.height, imgSize.width * 2, CV_8UC3);
	CvMat* pair2 = cvCreateMat(imgSize.height, imgSize.width * 2, CV_8UC3);
	CvMat* pair3 = cvCreateMat(imgSize.height, imgSize.width * 2, CV_8UC3);
	CvMat part1_ori, part2_ori, part3_ori;

	cvGetCols(pair1, &part1_ori, 0, imgSize.width);
	cvCvtColor(img_1, &part1_ori, CV_GRAY2BGR);

	cvGetCols(pair1, &part1_ori, imgSize.width, imgSize.width * 2);
	cvCvtColor(img1, &part1_ori, CV_GRAY2BGR);


	cvGetCols(pair2, &part2_ori, 0, imgSize.width);
	cvCvtColor(img_2, &part2_ori, CV_GRAY2BGR);

	cvGetCols(pair2, &part2_ori, imgSize.width, imgSize.width * 2);
	cvCvtColor(img2, &part2_ori, CV_GRAY2BGR);


	cv::Mat img1_mat = cvarrToMat(img1, true);
	cv::Mat img2_mat = cvarrToMat(img2, true);
	//根据偏航计算像素平移量
	int dx_differ = camr_intrin[0] * (tan((euler2.at<double>(2, 0) - euler1.at<double>(2, 0))*PI / 180));
	int dy1 = camr_intrin[4] * (tan(-euler1.at<double>(1, 0)*PI / 180));
	int dy2 = camr_intrin[4] * (tan(-euler2.at<double>(1, 0)*PI / 180));
	Mat Image1_moved(imgSize.height, imgSize.width, CV_8UC1, Scalar(0, 0, 0));
	Mat Image2_moved(imgSize.height, imgSize.width, CV_8UC1, Scalar(0, 0, 0));
	/*for (int i = 0; i<imgSize.height; i++)
	{
		for (int j = 0; j<imgSize.width - dx_differ; j++)
		{

			Image2_moved.at<uchar>(i, j) = img2_mat.at<uchar>(i, j + dx_differ);
		}
	}*/
	for (int i = 0; i<imgSize.height; i++)
	{
		for (int j = 0; j<imgSize.width; j++)
		{

			if (i < dy1)
				Image1_moved.at<uchar>(i, j) = 0;
			else
				Image1_moved.at<uchar>(i, j) = img1_mat.at<uchar>(i - dy1, j);
		}
	}
	for (int i = 0; i<imgSize.height; i++)
	{
		for (int j = 0; j<imgSize.width; j++)
		{
			if (i < dy2)
				Image2_moved.at<uchar>(i, j) = 0;
			else
				Image2_moved.at<uchar>(i, j) = img2_mat.at<uchar>(i - dy2, j);
		}
	}
	/*cv::imshow("img1_mat", img1_mat);
	cv::imshow("Image1_moved", Image1_moved);
	cv::imshow("img2_mat", img2_mat);
	cv::imshow("Image2_moved", Image2_moved);
	waitKey();
	*/
	IplImage Image1_proc(Image1_moved);
	IplImage Image2_proc(Image2_moved);

	cvGetCols(pair3, &part3_ori, 0, imgSize.width);
	cvCvtColor(&Image1_proc, &part3_ori, CV_GRAY2BGR);

	cvGetCols(pair3, &part3_ori, imgSize.width, imgSize.width * 2);
	cvCvtColor(&Image2_proc, &part3_ori, CV_GRAY2BGR);

	for (int j = 0; j< imgSize.width * 2; j += 320)
	{
		cvLine(pair1, cvPoint(j, 0), cvPoint(j, imgSize.height), CV_RGB(255, 0, 0));
	}
	for (int j = 0; j< imgSize.height; j += 240)
	{
		cvLine(pair1, cvPoint(0, j), cvPoint(imgSize.width * 2, j), CV_RGB(255, 0, 0));
	}
	for (int j = 0; j< imgSize.width * 2; j += 320)
	{
		cvLine(pair2, cvPoint(j, 0), cvPoint(j, imgSize.height), CV_RGB(255, 0, 0));
	}
	for (int j = 0; j< imgSize.height; j += 240)
	{
		cvLine(pair2, cvPoint(0, j), cvPoint(imgSize.width * 2, j), CV_RGB(255, 0, 0));
	}

	for (int j = 0; j< imgSize.width * 2; j += 320)
	{
		cvLine(pair3, cvPoint(j, 0), cvPoint(j, imgSize.height), CV_RGB(255, 0, 0));
	}
	for (int j = 0; j< imgSize.height; j += 240)
	{
		cvLine(pair3, cvPoint(0, j), cvPoint(imgSize.width * 2, j), CV_RGB(255, 0, 0));
	}

	CvMat *pair1_show_ori = cvCreateMat(cvGetSize(pair1).height, cvGetSize(pair1).width, CV_8UC3);
	CvMat *pair2_show_ori = cvCreateMat(cvGetSize(pair2).height, cvGetSize(pair2).width, CV_8UC3);
	CvMat *pair3_show_ori = cvCreateMat(cvGetSize(pair3).height, cvGetSize(pair3).width, CV_8UC3);

	cvResize(pair1, pair1_show_ori);
	cvResize(pair2, pair2_show_ori);
	cvResize(pair3, pair3_show_ori);

	cvNamedWindow("Image1_roll_back", 1);
	cvShowImage("originalImage1", pair1_show_ori);

	cvNamedWindow("Image2_roll_back", 1);
	cvShowImage("originalImage2", pair2_show_ori);

	cvNamedWindow("Imageprocessed", 1);
	cvShowImage("Imageprocessed", pair3_show_ori);
	cvWaitKey(0);
#endif


#ifdef STEREO

	IplImage *img_1 = cvLoadImage(img1_str.c_str(), 0);
	IplImage *img_2 = cvLoadImage(img2_str.c_str(), 0);
	//cv::Mat imgstereo_1, imgstereo_2;
	//imgstereo_1 = Mat(480, 640, CV_32FC3, Scalar(0));
	//imgstereo_2 = Mat(480, 640, CV_32FC3, Scalar(0));
	//CvSize imgSize = cvSize(640,480);
	//imgSize.height = 480;
	//imgSize.width = 640;
	//stereo_image_rectify(img_1, img_2, imgSize, imgstereo_1, imgstereo_2);

	//cv::Mat img1_ste, img2_ste;
	//get_subMat_census(imgstereo_1, imgstereo_2, img1_ste, img2_ste);

	stereomatch(img_1, img_2);

#endif

	/*string img_floder;
	if (argc <= 1)
	{
		printf("plz input an image folder\n");
		cin >> img_floder;
	}
	vector <vector <double>> picname[3];
	vector <string> foldername, picfile[3];

	get_data_picturestamp(img_floder, foldername, picfile, picname);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < picfile[i].size(); j++)
		{
			cout << picfile[i][j] << endl;
			cv::Mat img = imread(picfile[i][j]);
			cv::imshow("img", img);
			waitKey(10);
		}
		waitKey();

	}*/

	


	//Acquire matrix to use;


	//Image rectify;tailor

	//Image processing;tailor

	//stereo match to compute disparity;

	//assert is there a obstacle on the flying direction

	//if necessary,calculate the distance;

	//give status and results calculated.

	

	return 0;
}



