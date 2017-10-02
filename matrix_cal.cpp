#include "matrix_cal.h"
double body2cam[9] = { 0, 1, 0, 0, 0, 1, 1, 0, 0 };

vector<string> listdir(const string &path, int flag)
{

	string dir = path;
	vector<string> s1, s2;
	_finddata_t fileDir;
	long lfDir;
	char *p = ".jpg";
	if ((lfDir = _findfirst(dir.c_str(), &fileDir)) == -1l)
		printf("No file is found\n");
	else{
		do{

			string str(fileDir.name);
			char A[64];
			strcpy(A, str.c_str());
			if (str.find('.') == -1)
				s1.push_back(str);
			else if (strstr(A, p) != NULL)
				s2.push_back(str);


		} while (_findnext(lfDir, &fileDir) == 0);
	}
	_findclose(lfDir);
	if (flag == 1)
		return s1;
	else
		return s2;
}

void findfile(const string &str, vector <string> &res, int falg)
{
	string s = str;
	vector<string> tmp = listdir(s + "\\*", falg);//+ "\\*"
	for (int i = 0; i<tmp.size(); i++)
	{
		string temp = s + "\\" + tmp[i];
		res.push_back(temp);
		findfile(temp, res, falg);
	}
}

void get_data_picturestamp(string str, vector <string> &foldername, vector <string> picfile[3], vector <vector <double>> picstap[3])
{
	vector <double> timestamp;
	findfile(str, foldername, 1);
	for (int i = 0; i < foldername.size(); i++)
	{
		findfile(foldername[i], picfile[i], 2);
		vector <string> filename = picfile[i];
		vector<char*> vec_str;
		const char * split = "_";
		for (int j = 0; j < filename.size(); j++)
		{
			char *p = strtok((char*)filename[j].c_str(), split);
			while (p)
			{
				vec_str.push_back(p);
				p = strtok(NULL, split);
			}
			//char* sequence_name1 = vec_str[1];
			char* sequence_name2 = vec_str[3];
			char* sequence_name3 = vec_str[5];
			vec_str.resize(NULL);

			timestamp.push_back(atof(sequence_name2));
			timestamp.push_back(atof(sequence_name3));
			picstap[i].push_back(timestamp);
			timestamp.resize(NULL);
		}
	}
}

cv::Mat get_Rmatrix_from_Quaternion(double Qarray[4])
{
	Mat rotation_matrix = Mat(3, 3, CV_64FC1, Scalar(0));

	rotation_matrix.at<double>(0, 0) = 1 - 2 * (pow(Qarray[2], 2) + pow(Qarray[3], 2));
	rotation_matrix.at<double>(0, 1) = 2 * (Qarray[1] * Qarray[2] - Qarray[0] * Qarray[3]);
	rotation_matrix.at<double>(0, 2) = 2 * (Qarray[1] * Qarray[3] + Qarray[0] * Qarray[2]);
	rotation_matrix.at<double>(1, 1) = 1 - 2 * (pow(Qarray[1], 2) + pow(Qarray[3], 2));
	rotation_matrix.at<double>(1, 0) = 2 * (Qarray[1] * Qarray[2] + Qarray[0] * Qarray[3]);
	rotation_matrix.at<double>(1, 2) = 2 * (Qarray[2] * Qarray[3] - Qarray[0] * Qarray[1]);
	rotation_matrix.at<double>(2, 2) = 1 - 2 * (pow(Qarray[1], 2) + pow(Qarray[2], 2));
	rotation_matrix.at<double>(2, 0) = 2 * (Qarray[1] * Qarray[3] - Qarray[0] * Qarray[2]);
	rotation_matrix.at<double>(2, 1) = 2 * (Qarray[3] * Qarray[2] + Qarray[0] * Qarray[1]);

	//cout << rotation_matrix << endl;
	return rotation_matrix;
}
cv::Mat get_Euler_from_Quaternion(double Qarray[4])
{
	Mat Euler = Mat(3, 1, CV_64FC1, Scalar(0));
	// roll (x-axis rotation)
	double sinr = +2.0 * (Qarray[0] * Qarray[1] + Qarray[2] * Qarray[3]);
	double cosr = +1.0 - 2.0 * (Qarray[1] * Qarray[1] + Qarray[2] * Qarray[2]);
	Euler.at<double>(0, 0) = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (Qarray[0] * Qarray[2] - Qarray[3] * Qarray[1]);
	if (fabs(sinp) >= 1)
		Euler.at<double>(1, 0) = copysign(PI / 2, sinp); // use 90 degrees if out of range
	else
		Euler.at<double>(1, 0) = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (Qarray[0] * Qarray[3] + Qarray[1] * Qarray[2]);
	double cosy = +1.0 - 2.0 * (Qarray[2] * Qarray[2] + Qarray[3] * Qarray[3]);
	Euler.at<double>(2, 0) = atan2(siny, cosy);

	return Euler;
}

void get_matrix_from_log(double Quaternion1[4], double Quaternion2[4], double location1[3], double location2[3], CvMat &Rotation, CvMat &Translation)
{
	Mat bodytocam = Mat(3, 3, CV_64FC1, body2cam);

	cv::Mat R1 = get_Rmatrix_from_Quaternion(Quaternion1);
	cv::Mat R2 = get_Rmatrix_from_Quaternion(Quaternion2);
	cout << R1 << endl;
	cout << R2 << endl;
	cv::Mat R1_cam, R2_cam, _Rotation;
	R1_cam = R1*bodytocam.inv();
	R2_cam = R2*bodytocam.inv();
	cout << R1_cam << endl;
	cout << R2_cam << endl;

	_Rotation = R1_cam.inv()*R2_cam; //from the latter one to the former one,like from the right one to left one
	Rotation = _Rotation;
	cout << _Rotation << endl;

	cv::Mat loc1_ned = Mat(3, 1, CV_64FC1, location1);
	cv::Mat loc2_ned = Mat(3, 1, CV_64FC1, location2);
	cout << loc1_ned << endl;
	cout << loc2_ned << endl;

	cv::Mat loc1_body, loc2_body, loc1_cam, loc2_cam;
	loc1_body = R1.inv()*loc1_ned;
	loc2_body = R2.inv()*loc2_ned;
	cout << loc1_body << endl;
	cout << loc2_body << endl;
	loc1_cam = bodytocam * loc1_body;
	loc2_cam = bodytocam * loc2_body;
	cout << loc1_cam << endl;
	cout << loc1_cam << endl;

	cv::Mat _Translation = loc1_cam - loc2_cam;//the same as Rotation
	cout << _Translation << endl;

	Translation = _Translation;
}

void Geographic_to_Geocentric(double geographic[3], double Geocentric[3])//大地--->球心

{
	double e = sqrt(1 - pow(EARTH_B, 2) / pow(EARTH_A, 2)); //e为椭球的第一偏心率
	double m = PI / 180;//经度维度需要转换成弧度.


	double H = geographic[2] + EARTH_A;
	double B = geographic[0] * m;
	double L = geographic[1] * m;

	double W = sqrt(1 - pow(e, 2)*pow(sin(B), 2));
	double N = EARTH_A / W;

	Geocentric[0] = (N + H)*cos(B)*cos(L);
	Geocentric[1] = (N + H)*cos(B)*sin(L);
	Geocentric[2] = (N*(1 - pow(e, 2)) + H)*sin(B);

}

void Geocentric_to_Geographic(double geocentric[3], double Geographic[3])//球心--->大地

{
	double v0 = geocentric[2] / sqrt(pow(geocentric[0], 2) + pow(geocentric[1], 2));
	double e = sqrt(1 - pow(EARTH_B, 2) / pow(EARTH_A, 2)); //e为椭球的第一偏心率

	double N = 0; //N为椭球的卯酉圈曲率半径
	double B1 = atan(v0), B2 = 0;
	double H = 0;

	while (abs(B2 - B1)>1E-5)

	{

		N = EARTH_A / sqrt(1 - pow(e, 2)*pow(sin(B1), 2));
		H = geocentric[2] / sin(B1) - N*(1 - pow(e, 2));
		B2 = atan(geocentric[2] * (N + H) / sqrt((pow(geocentric[0], 2) + pow(geocentric[1], 2))*(N*(1 - pow(e, 2)) + H)));
		B1 = B2;

	}

	double m = PI / 180;

	Geographic[0] = (B1 / m);
	Geographic[1] = (atan(geocentric[1] / geocentric[0]) / m);
	Geographic[2] = (H - EARTH_A);

}