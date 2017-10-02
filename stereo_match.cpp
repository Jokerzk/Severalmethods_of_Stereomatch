#include "stereo_match.h"
int IMGHEIGHT = 480;
int IMGWIDTH = 640;

void SAD(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity)//grey image
{
	int i, j, d;
	int absDiff;
	int minSum;
	int disp;
	int *Col;
	int *Wnd;

	Col = new int[IMGWIDTH*DOMAIN];//
	Wnd = new int[IMGWIDTH*DOMAIN];

	for (i = 0; i<IMGWIDTH*DOMAIN; i++)
	{
		Col[i] = 0;
		Wnd[i] = 0;
	}

	for (j = 0; j<IMGHEIGHT; j++)
	{
		for (i = 0; i<IMGWIDTH; i++)
		{
			minSum = 100000;
			disp = 0;
			for (d = 0; d<DOMAIN; d++)
			{
				if (i >= DOMAIN)
					absDiff = abs(LeftImg[j*IMGWIDTH + i] - RightImg[j*IMGWIDTH + i - d]);
				else
					absDiff = 0;

				Col[i*DOMAIN + d] = Col[i*DOMAIN + d] + absDiff;

				if (j>WndWidth)
				{
					if (i>DOMAIN)
						absDiff = abs(LeftImg[(j - WndWidth)*IMGWIDTH + i] - RightImg[(j - WndWidth)*IMGWIDTH + i - d]);
					else
						absDiff = 0;
					Col[i*DOMAIN + d] = Col[i*DOMAIN + d] - absDiff;
				}

				if (i<DOMAIN || j<WndWidth)
					Wnd[i*DOMAIN + d] = 0;
				else
					Wnd[i*DOMAIN + d] = Wnd[(i - 1)*DOMAIN + d] + Col[i*DOMAIN + d]
					- Col[(i - WndWidth)*DOMAIN + d];

				/*
				if( i>WndWidth )
				Wnd[i*DOMAIN+d]=Wnd[(i-1)*DOMAIN+d]+Col[i*DOMAIN+d]-Col[(i-WndWidth)*DOMAIN+d];
				else
				{
				if( i>d )
				absDiff=abs(LeftImg[j*IMGWIDTH+i]-RightImg[j*IMGWIDTH+i-d]);
				else
				absDiff=0;

				Wnd[i*DOMAIN+d]=Wnd[i*DOMAIN+d]+absDiff;
				if( j>WndWidth )
				{
				if( i>d )
				absDiff=abs(LeftImg[(j-WndWidth)*IMGWIDTH+i]-RightImg[(j-WndWidth)*IMGWIDTH+i-d]);
				else
				absDiff=0;
				Wnd[i*DOMAIN+d]=Wnd[i*DOMAIN+d]-absDiff;
				}
				}
				*/
				if (Wnd[i*DOMAIN + d]<minSum)
				{
					minSum = Wnd[i*DOMAIN + d];
					disp = d * 15;
				}

			}

			if (j>WndWidth && i>DOMAIN)
				Disparity[j * IMGWIDTH + i] = disp;

		}

	}

	delete[]Col;
	delete[]Wnd;

}

void SSD(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity)
{
	int i, j, d;
	int minSum;
	int sqDiff;
	int disp;
	int *Col;
	int *Wnd;

	Col = new int[IMGWIDTH*DOMAIN];
	Wnd = new int[IMGWIDTH*DOMAIN];

	for (i = 0; i<IMGWIDTH*DOMAIN; i++)
	{
		Col[i] = 0;
		Wnd[i] = 0;
	}

	for (j = 0; j<IMGHEIGHT; j++)
	for (i = 0; i<IMGWIDTH; i++)
	{
		minSum = 100000;
		disp = 0;
		for (d = 0; d<DOMAIN; d++)
		{
			if (i>DOMAIN)
				sqDiff = (int)pow((double)(LeftImg[j*IMGWIDTH + i] - RightImg[j*IMGWIDTH + i - d]), 2);
			else
				sqDiff = 0;

			Col[i*DOMAIN + d] = Col[i*DOMAIN + d] + sqDiff;

			if (j>WndWidth)
			{
				if (i>DOMAIN)
					sqDiff = (int)pow((double)(LeftImg[(j - WndWidth)*IMGWIDTH + i] - RightImg[(j - WndWidth)*IMGWIDTH + i - d]), 2);
				else
					sqDiff = 0;
				Col[i*DOMAIN + d] = Col[i*DOMAIN + d] - sqDiff;
			}


			if (i<DOMAIN || j<WndWidth)
				Wnd[i*DOMAIN + d] = 0;
			else
				Wnd[i*DOMAIN + d] = Wnd[(i - 1)*DOMAIN + d] + Col[i*DOMAIN + d]
				- Col[(i - WndWidth)*DOMAIN + d];

			if (Wnd[i*DOMAIN + d]<minSum)
			{
				minSum = Wnd[i*DOMAIN + d];
				disp = d * 15;
			}

		}
		if (j>WndWidth && i>DOMAIN)
			Disparity[j * IMGWIDTH + i] = disp;

	}

	delete[]Col;
	delete[]Wnd;

}

void Rank(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity)
{
	unsigned char *lfRankValue, *rtRankValue;

	lfRankValue = new unsigned char[IMGHEIGHT*IMGWIDTH];
	rtRankValue = new unsigned char[IMGHEIGHT*IMGWIDTH];

	RankTransform(LeftImg, lfRankValue);
	RankTransform(RightImg, rtRankValue);
	SAD(lfRankValue, rtRankValue, Disparity);

}

void RankTransform(unsigned char *Image, unsigned char *RankValue)
{
	int i, j, p, q;
	int TRANSWIND;
	int count;
	unsigned char center, neighbor;

	TRANSWIND = 4;
	for (i = 0; i<IMGHEIGHT; i++)
	for (j = 0; j<IMGWIDTH; j++)
	{
		center = Image[i*IMGWIDTH + j];
		count = 0;

		for (p = i - TRANSWIND; p <= i + TRANSWIND; p++)//9*9 wnd
		{
			for (q = j - TRANSWIND; q <= j + TRANSWIND; q++)
			{
				if (!((p == i) && (q == j)))
				{
					if (p<0 || p >= IMGHEIGHT || q<0 || q >= IMGWIDTH)
						neighbor = 0;
					else
						neighbor = Image[p*IMGWIDTH + q];

					if (neighbor > center)
						count++;
				}
			}
		}
		RankValue[i*IMGWIDTH + j] = count;
	}
}

void Census(unsigned char *LeftImg, unsigned char *RightImg, unsigned char *Disparity)
{
	int i, j, d;
	int p, q;
	int Hdist;
	int minSum;
	int disp;
	int *Col;
	int *Wnd;

	CodeStruct *lfCode, *rtCode;

	lfCode = new CodeStruct[IMGHEIGHT*IMGWIDTH];
	rtCode = new CodeStruct[IMGHEIGHT*IMGWIDTH];

	/*/////CENSUS TRANSFORM/////*/
	for (i = 0; i<IMGHEIGHT*IMGWIDTH; i++)
	for (j = 0; j<CODEWIDTH; j++)
	{
		lfCode[i].codebit[j] = 0;
		rtCode[i].codebit[j] = 0;
	}


	CensusTransform(LeftImg, lfCode);
	CensusTransform(RightImg, rtCode);
	/*
	int p_col=0;
	for( int i_code=0 ; i_code<IMGHEIGHT*IMGWIDTH ; i_code++ )
	{
	p = i_code/IMGWIDTH;
	q = i_code%IMGWIDTH;

	minSum = 1000;
	d=0;
	while( d<32 )
	{
	disp = 0;
	for( i=p-5 ; i<=(p+5) ; i++ )//11*11 wnd
	for( j=q-5 ; j<=(q+5) ; j++ )
	{
	if( i<0 || i>=IMGHEIGHT || j<d || j>=IMGWIDTH )
	disp +=0;
	else
	disp += HammingDist( lfCode[i*IMGWIDTH+j], rtCode[i*IMGWIDTH+j-d] );
	}
	if( disp < minSum )
	{
	minSum = disp;
	p_col = d;
	}
	d++;
	}

	Disparity[p*IMGWIDTH+q]=p_col*12;

	}
	*/

	Col = new int[IMGWIDTH*DOMAIN];
	Wnd = new int[IMGWIDTH*DOMAIN];

	for (i = 0; i<IMGWIDTH*DOMAIN; i++)
	{
		Col[i] = 0;
		Wnd[i] = 0;
	}

	for (j = 0; j<IMGHEIGHT; j++)
	for (i = 0; i<IMGWIDTH; i++)
	{
		minSum = 100000;
		disp = 0;
		for (d = 0; d<DOMAIN; d++)
		{
			if (i>DOMAIN)
				Hdist = HammingDist(lfCode[j*IMGWIDTH + i], rtCode[j*IMGWIDTH + i - d]);
			else
				Hdist = 0;

			Col[i*DOMAIN + d] = Col[i*DOMAIN + d] + Hdist;

			if (j >= WndWidth)
			{
				if (i>DOMAIN)
					Hdist = HammingDist(lfCode[(j - WndWidth)*IMGWIDTH + i], rtCode[(j - WndWidth)*IMGWIDTH + i - d]);
				else
					Hdist = 0;
				Col[i*DOMAIN + d] = Col[i*DOMAIN + d] - Hdist;
			}

			if (i<DOMAIN)
				Wnd[i*DOMAIN + d] = 0;
			else
				Wnd[i*DOMAIN + d] = Wnd[(i - 1)*DOMAIN + d] + Col[i*DOMAIN + d]
				- Col[(i - WndWidth)*DOMAIN + d];

			/*
			if( i>WndWidth )
			Wnd[i*DOMAIN+d]=Wnd[(i-1)*DOMAIN+d]+Col[i*DOMAIN+d]-Col[(i-WndWidth)*DOMAIN+d];
			else
			{
			if( i>d )
			Hdist=HammingDist(lfCode[j*IMGWIDTH+i],rtCode[j*IMGWIDTH+i-d]);
			else
			Hdist=0;

			Wnd[i*DOMAIN+d]=Wnd[i*DOMAIN+d]+Hdist;
			if( j>WndWidth )
			{
			if( i>d )
			Hdist=HammingDist(lfCode[(j-WndWidth)*IMGWIDTH+i],rtCode[(j-WndWidth)*IMGWIDTH+i-d]);
			else
			Hdist=0;
			Wnd[i*DOMAIN+d]=Wnd[i*DOMAIN+d]-Hdist;
			}
			}
			*/

			if (Wnd[i*DOMAIN + d] <= minSum)
			{
				minSum = Wnd[i*DOMAIN + d];
				disp = d * 15;
			}

		}
		if (j>WndWidth && i>DOMAIN)
			Disparity[j * IMGWIDTH + i] = disp;

	}

	delete[]lfCode;
	delete[]rtCode;
}

void CensusTransform(unsigned char *Image, CodeStruct Code[])
{
	int i, j, p, q;
	int TRANSWIND;
	int bit;
	unsigned char center, neighbor;

	TRANSWIND = 4;
	for (i = 0; i<IMGHEIGHT; i++)
	for (j = 0; j<IMGWIDTH; j++)
	{
		center = Image[i*IMGWIDTH + j];
		bit = 0;

		for (p = i - TRANSWIND; p <= i + TRANSWIND; p++)//9x9 wnd
		{
			for (q = j - TRANSWIND; q <= j + TRANSWIND; q++)
			{
				if (!((p == i) && (q == j)))
				{
					if (p<0 || p >= IMGHEIGHT || q<0 || q >= IMGWIDTH)
						neighbor = 0;
					else
						neighbor = Image[p*IMGWIDTH + q];

					if (neighbor > center)
						Code[i*IMGWIDTH + j].codebit[bit] = 1;
					else
						Code[i*IMGWIDTH + j].codebit[bit] = 0;
					bit++;
				}
			}
		}
	}
}

int HammingDist(CodeStruct Code1, CodeStruct Code2)
{
	int i;
	int dist;

	dist = 0;
	for (i = 0; i<CODEWIDTH; i++)
	{
		if (Code1.codebit[i] != Code2.codebit[i])
			dist++;
	}
	return dist;
}

void SSD_stereo_match(cv::Mat image1, cv::Mat image2, double T, double F)//此时利用飞行方向上的前进距离T可以求出对应点的距离信息;
{
	assert(image1.cols == image2.cols || image1.rows == image2.rows || image1.rows == 1);
	unsigned char *Img1 = image1.data;
	unsigned char *Img2 = image2.data;
	cv::Mat Disparity = Mat(1,image1.cols,CV_64FC1,Scalar(0));
	int sqDiff, disp;
	int minSSum = 100000;
	int col[640] = { 0 }, dist[640] = { 0 };
	for (int i = 0; i < image1.cols; i++)
	{
		if (i <= image1.cols / 2 + 0.5)//在前一帧图像的左半区，随着无人机临近，对应后一帧像素必然在其左侧;
		{
			for (int d = 0; d < DOMAIN - (WndWidth - 1) / 2; d++)
			{
				if (i>DOMAIN)
					sqDiff = (int)pow((Img1[i] - Img2[i - d]), 2) + (int)pow((Img1[i - 1] - Img2[i - d -1]), 2) + (int)pow((Img1[i - 2] - Img2[i - d - 2]), 2) + (int)pow((Img1[i - 3] - Img2[i - d - 3]), 2) + (int)pow((Img1[i - 4] - Img2[i - d - 4]), 2)
																  + (int)pow((Img1[i + 1] - Img2[i - d + 1]), 2) + (int)pow((Img1[i + 2] - Img2[i - d + 2]), 2) + (int)pow((Img1[i + 3] - Img2[i - d + 3]), 2) + (int)pow((Img1[i + 4] - Img2[i - d + 4]), 2);
				else
					sqDiff = 0;

				if (sqDiff < minSSum)
				{

					disp = d;
					minSSum = sqDiff;
				}
			}
			col[i] = disp;
			if (disp != 0 && i != (image1.cols / 2 - 0.5))
				dist[i] = abs(i - image1.cols / 2 - 0.5) / disp*T - F;
			else
				dist[i] = INFINITY;

		}
		else//与上相对，在第一帧图像右侧;
		{
			for (int d = 0; d < DOMAIN - (WndWidth - 1) / 2; d++)
			{
				if (i<640 - DOMAIN)
					sqDiff = (int)pow((Img1[i] - Img2[i + d]), 2) + (int)pow((Img1[i - 1] - Img2[i + d - 1]), 2) + (int)pow((Img1[i - 2] - Img2[i + d - 2]), 2) + (int)pow((Img1[i - 3] - Img2[i + d - 3]), 2) + (int)pow((Img1[i - 4] - Img2[i + d - 4]), 2)
																  + (int)pow((Img1[i + 1] - Img2[i + d + 1]), 2) + (int)pow((Img1[i + 2] - Img2[i + d + 2]), 2) + (int)pow((Img1[i + 3] - Img2[i + d + 3]), 2) + (int)pow((Img1[i + 4] - Img2[i + d + 4]), 2);
				else
					sqDiff = 0;

				if (sqDiff < minSSum)
				{

					disp = d;
					minSSum = sqDiff;
				}
			}
			col[i] = disp;
			if (disp != 0 && i != (image1.cols / 2 - 0.5))
				dist[i] = abs(i - image1.cols / 2 - 0.5) / disp*T - F;
			else
				dist[i] = INFINITY;
		}
	}
}

void Census_stereo_match(cv::Mat image1, cv::Mat image2)
{
	assert(image1.cols == image2.cols || image1.rows == image2.rows || image1.rows == 1);
	unsigned char *Img1 = image1.data;
	unsigned char *Img2 = image2.data;

}

void stereomatch(cv::Mat img1, cv::Mat img2)
{
	
	int i, j;
	unsigned char *LeftImg, *RightImg, *Disparity;
	double pixVal;
	IplImage *Result_Census, *Result_SSD, *Result_SAD, *Result_Rank;
	CvSize ImgSize;

	//IMGHEIGHT = img1.rows;
	//IMGWIDTH = img2.cols;

	//LeftImg = new unsigned char[IMGHEIGHT*IMGWIDTH];
	//RightImg = new unsigned char[IMGHEIGHT*IMGWIDTH];
	LeftImg = img1.data;
	RightImg = img2.data;
	Disparity = new unsigned char[IMGHEIGHT*IMGWIDTH];

	ImgSize.height = IMGHEIGHT;
	ImgSize.width = IMGWIDTH;

	Result_Census = cvCreateImage(ImgSize, IPL_DEPTH_8U, 1);
	/*Result_SAD = cvCreateImage(ImgSize, IPL_DEPTH_8U, 1);*/
	Result_SSD = cvCreateImage(ImgSize, IPL_DEPTH_8U, 1);
	/*Result_Rank = cvCreateImage(ImgSize, IPL_DEPTH_8U, 1);
*/
	for (j = 0; j<IMGHEIGHT; j++)
	{
		for (i = 0; i<IMGWIDTH; i++)
		{
			/*pixVal = cvGetReal2D(image1, j, i);
			LeftImg[j*IMGWIDTH + i] = (unsigned char)pixVal;

			pixVal = cvGetReal2D(image2, j, i);
			RightImg[j*IMGWIDTH + i] = (unsigned char)pixVal;*/

			pixVal = 0.0;
			Disparity[j*IMGWIDTH + i] = (unsigned char)pixVal;
		}
	}

	//cvSetZero(Result_Rank);
	cvSetZero(Result_Census);
	cvSetZero(Result_SSD);

	clock_t start;
	clock_t end;
	start = clock();

	Census(LeftImg, RightImg, Disparity);

	end = clock();

	for (j = 0; j<IMGHEIGHT; j++){
		for (i = 0; i<IMGWIDTH; i++)
		{
			cvSetReal2D(Result_Census, j, i, Disparity[j*IMGWIDTH + i]);
		}
	}


	printf("time: %.3f s\n", (double)(end - start) / CLOCKS_PER_SEC);
	cvNamedWindow("Census", 1);
	cvShowImage("Census", Result_Census);




	/*for (j = 0; j<IMGHEIGHT; j++)
	{
		for (i = 0; i<IMGWIDTH; i++)
		{
			pixVal = 0.0;
			Disparity[j*IMGWIDTH + i] = (unsigned char)pixVal;
		}
	}
	Rank(LeftImg, RightImg, Disparity);

	for (j = 0; j<IMGHEIGHT; j++){
		for (i = 0; i<IMGWIDTH; i++)
		{
			cvSetReal2D(Result_Rank, j, i, Disparity[j*IMGWIDTH + i]);
		}
	}
	cvNamedWindow("Rank", 1);
	cvShowImage("Rank", Result_Rank);*/



	for (j = 0; j<IMGHEIGHT; j++)
	{
		for (i = 0; i<IMGWIDTH; i++)
		{
			pixVal = 0.0;
			Disparity[j*IMGWIDTH + i] = (unsigned char)pixVal;
		}
	}
	SSD(LeftImg, RightImg, Disparity);

	for (j = 0; j<IMGHEIGHT; j++){
		for (i = 0; i<IMGWIDTH; i++)
		{
			cvSetReal2D(Result_SSD, j, i, Disparity[j*IMGWIDTH + i]);
		}
	}
	cvNamedWindow("SSD", 1);
	cvShowImage("SSD", Result_SSD);



	/*for (j = 0; j<IMGHEIGHT; j++)
	{
		for (i = 0; i<IMGWIDTH; i++)
		{
			pixVal = 0.0;
			Disparity[j*IMGWIDTH + i] = (unsigned char)pixVal;
		}
	}
	SAD(LeftImg, RightImg, Disparity);

	for (j = 0; j<IMGHEIGHT; j++){
		for (i = 0; i<IMGWIDTH; i++)
		{
			cvSetReal2D(Result_SAD, j, i, Disparity[j*IMGWIDTH + i]);
		}
	}
	cvNamedWindow("SAD", 1);
	cvShowImage("SAD", Result_SAD);*/



	cvWaitKey(0);
	//cvSaveImage("dispCensus.jpg", Result_Census);
	//cvSaveImage("dispSAD.jpg", Result_SAD);
	//cvSaveImage("dispSSD.jpg", Result_SSD);
	//("dispRank.jpg", Result_Rank);
	//cvDestroyWindow("SAD");
	//cvDestroyWindow("SSD");
	//cvDestroyWindow("Rank");
	//cvDestroyWindow("Census");

}



//void BM(IplImage * img1, IplImage * img2)
//{
//	CvStereoBMState* BMState = cvCreateStereoBMState();
//	assert(BMState);
//	BMState->preFilterSize = 9;
//	BMState->preFilterCap = 31;
//	BMState->SADWindowSize = 15;
//	BMState->minDisparity = 0;
//	BMState->numberOfDisparities = 64;
//	BMState->textureThreshold = 10;
//	BMState->uniquenessRatio = 15;
//	BMState->speckleWindowSize = 100;
//	BMState->speckleRange = 32;
//	BMState->disp12MaxDiff = 1;
//
//	CvMat* disp = cvCreateMat(img1->height, img1->width, CV_16S);
//	CvMat* vdisp = cvCreateMat(img1->height, img1->width, CV_8U);
//	int64 t = getTickCount();
//	cvFindStereoCorrespondenceBM(img1, img2, disp, BMState);
//	t = getTickCount() - t;
//	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
//	cvSave("disp.xml", disp);
//	cvNormalize(disp, vdisp, 0, 255, CV_MINMAX);
//	cvNamedWindow("BM_disparity", 0);
//	cvShowImage("BM_disparity", vdisp);
//	cvWaitKey(0);
//	//cvSaveImage("cones\\BM_disparity.png",vdisp);  
//	cvReleaseMat(&disp);
//	cvReleaseMat(&vdisp);
//	cvDestroyWindow("BM_disparity");
//}

//void SGBM(IplImage * img1, IplImage * img2)
//{
//
//	cv::StereoSGBM sgbm;
//	int SADWindowSize = 9;
//	sgbm.preFilterCap = 63;
//	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
//	int cn = img1->nChannels;
//	int numberOfDisparities = 64;
//	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
//	sgbm.minDisparity = 0;
//	sgbm.numberOfDisparities = numberOfDisparities;
//	sgbm.uniquenessRatio = 10;
//	sgbm.speckleWindowSize = 100;
//	sgbm.speckleRange = 32;
//	sgbm.disp12MaxDiff = 1;
//	Mat disp, disp8;
//	int64 t = getTickCount();
//	sgbm((Mat)img1, (Mat)img2, disp);
//	t = getTickCount() - t;
//	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
//	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
//
//	/*namedWindow("left", 1);
//	cvShowImage("left", img1);
//	namedWindow("right", 1);
//	cvShowImage("right", img2);*/
//	namedWindow("disparity", 1);
//	imshow("disparity", disp8);
//	waitKey();
//	imwrite("sgbm_disparity.png", disp8);
//	cvDestroyAllWindows();
//}

//void GC(IplImage * img1, IplImage * img2)
//{
//	CvStereoGCState* GCState = cvCreateStereoGCState(64, 3);
//	assert(GCState);
//	cout << "start matching using GC" << endl;
//	CvMat* gcdispleft = cvCreateMat(img1->height, img1->width, CV_16S);
//	CvMat* gcdispright = cvCreateMat(img2->height, img2->width, CV_16S);
//	CvMat* gcvdisp = cvCreateMat(img1->height, img1->width, CV_8U);
//	int64 t = getTickCount();
//	cvFindStereoCorrespondenceGC(img1, img2, gcdispleft, gcdispright, GCState);
//	t = getTickCount() - t;
//	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
//	//cvNormalize(gcdispleft,gcvdisp,0,255,CV_MINMAX);  
//	//cvSaveImage("GC_left_disparity.png",gcvdisp);  
//	cvNormalize(gcdispright, gcvdisp, 0, 255, CV_MINMAX);
//	cvSaveImage("GC_right_disparity.png", gcvdisp);
//
//
//	cvNamedWindow("GC_disparity", 0);
//	cvShowImage("GC_disparity", gcvdisp);
//	cvWaitKey(0);
//	cvReleaseMat(&gcdispleft);
//	cvReleaseMat(&gcdispright);
//	cvReleaseMat(&gcvdisp);
//}