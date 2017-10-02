#ifndef MATRIX_CAL_
#define MATRIX_CAL_

#include <stdio.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <io.h>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define EARTH_A 6378137 //地球长半轴，单位：米
#define EARTH_B 6356752.3141
#define PI 3.14159265358979

typedef Vec<uchar, 3> Vec3b;

vector<string> listdir(const string &path, int flag);

void findfile(const string &str, vector <string> &res, int falg);

void get_data_picturestamp(string str, vector <string> &foldername, vector <string> picfile[3], vector <vector <double>> picstap[3]);

cv::Mat get_Rmatrix_from_Quaternion(double Qarray[4]);

cv::Mat get_Euler_from_Quaternion(double Qarray[4]);

void get_matrix_from_log(double Quaternion1[4], double Quaternion2[4], double location1[3], double location2[3], CvMat &Rotation, CvMat &Translation);

void Geographic_to_Geocentric(double geographic[3], double Geocentric[3]);

void Geocentric_to_Geographic(double geocentric[3], double Geographic[3]);

#endif