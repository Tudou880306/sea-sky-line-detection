// sea-sky line detection.cpp : 定义控制台应用程序的入口点。
//
//
#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include <opencv\cv.h>
#include <opencv2\highgui\highgui.hpp>
#include <vector>
#include <ctime>


using namespace std;
using namespace cv;

//otsu 最大类间距算法，通过灰度直方图，找出阈值。
//类似k-means 聚类算法
int otsu(cv::Mat image)
{
	assert(!image.empty());

	int width = image.cols;
	int height = image.rows;
	int x = 0, y = 0;
	int pixelCount[256];
	float pixelPro[256];
	int i, j, pixelSum = width * height, threshold = 0;

	uchar* data = (uchar*)image.data;

	//初始化
	for (i = 0; i < 256; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//统计灰度级中每个像素在整幅图像中的个数
	for (i = y; i < height; i++)
	{
		for (j = x; j < width; j++)
		{
			pixelCount[data[i * image.step + j]]++;
		}
	}

	//计算每个像素在整幅图像中的比例
	for (i = 0; i < 256; i++)
	{
		pixelPro[i] = (float)(pixelCount[i]) / (float)(pixelSum);
	}

	//经典ostu算法,得到前景和背景的分割
	//遍历灰度级[0,255],计算出方差最大的灰度值,为最佳阈值
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < 256; i++)
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (j = 0; j < 256; j++)
		{
			if (j <= i) //背景部分
			{
				//以i为阈值分类，第一类总的概率
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else       //前景部分
			{
				//以i为阈值分类，第二类总的概率
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}

		u0 = u0tmp / w0;        //第一类的平均灰度
		u1 = u1tmp / w1;        //第二类的平均灰度
		u = u0tmp + u1tmp;        //整幅图像的平均灰度
								  //计算类间方差
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//找出最大类间方差以及对应的阈值
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = i;
		}
	}
	//返回最佳阈值;
	return threshold;
}


//对二值图像做边缘提取，没有采用canny
//对二值图像的纵向相邻像素做异或处理，提取像素值跳跃边界线
cv::Mat edge(cv::Mat image)
{
	cv::Mat dst = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

	uchar* data = (uchar*)image.data;
	uchar* data1 = (uchar*)dst.data;
	for (int i = 0; i<image.cols; i++)
	{
		for (int j = 0; j<image.rows - 1; j++)
		{
			int a = data[j * image.step + i];
			int b = data[(j + 1) * image.step + i];
			int  c = a^b;
			data1[j * image.step + i] = c;
		}
	}
	return dst;
}

//霍夫变换
/*功能：将输入图像按照给出参数要求提取线段，放在lines中。

	lines : 是一个vector<Vec4i>, Vec4i是一个包含4个int数据类型的结构体，[x1, y1, x2, y2], 可以表示一个线段。

	rho : 就是一个半径的分辨率。 以像素为单位的距离精度。 另一种形容方式是直线搜索时的进步尺寸的单位半径。

	theta : 角度分辨率。以弧度为单位的角度精度。另一种形容方式是直线搜索时的进步尺寸的单位角度

	threshold : 判断直线点数的阈值。累加平面的阈值参数，即识别某部分为图中的一条直线时它在累加平面中必须达到的值。 大于阈值 threshold 的线段才可以被检测通过并返回到结果中。

	minLineLength：线段长度阈值。有默认值0，表示最低线段的长度，比这个设定参数短的线段就不能被显现出来。

	minLineGap : 线段上最近两点之间的阈值 有默认值0，允许将同一行点与点之间连接起来的最大的距离
*/
static void on_HoughLines(cv::Mat src, cv::Mat dst)
{
	Mat midImage = src.clone();

	/*imshow("2", midImage);
	waitKey(15);*/
	//调用HoughLinesP函数  
	vector<Vec4i> mylines;
	HoughLinesP(midImage, mylines, 1, CV_PI / 180, 150 + 1, 50, 10);//局部otsu和全局otsu分别设置不同的参数
	//循环遍历绘制每一条线段  
	for (size_t i = 0; i < mylines.size(); i++)
	{
		Vec4i l = mylines[i];
		line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, 8);
	}
}
//灰度直方图
Mat getHistograph(const Mat grayImage);

cv::Mat SeaSkyDetector(cv::Mat input)
{
	cv::Rect rect(0,200,1920,150);
	cv::Mat src1 = input(rect);
	cv::Mat srcGray;
	cvtColor(src1, srcGray, CV_BGR2GRAY);

	//中值滤波，过滤噪声
	medianBlur(srcGray, srcGray, 3);


	//对于天空和海水各自的灰度一致性较强的情况，两者效果差距不大
	//对于光照不均匀的情况，例如海面较强反光等，采用局部otsu效果较好

#if 1 //全局otsu求阈值

	int threshold = otsu(srcGray);
	printf("threshold = %d\n", threshold);
	//对图像二值化
	cv::Mat dst = cv::Mat::zeros(srcGray.rows, srcGray.cols, CV_8UC1);
	cvThreshold(&(IplImage)srcGray, &(IplImage)dst, threshold, 255, CV_THRESH_BINARY);

	cv::Mat edgeimg = edge(dst);
	/*imshow(" 1", edgeimg);
	waitKey(5);*/
	on_HoughLines(edgeimg, src1);
	//imwrite("dst.jpg", src1);
	imshow(" src1", src1);
	waitKey(5);
	return src1;

#else //局部otsu求阈值
	cv::Mat res = cv::Mat::zeros(srcGray.rows, srcGray.cols, CV_8UC3);
	const int grid = 8;
	for (int i = 0; i < grid; i++)
	{

		cv::Mat t = srcGray(Rect(i* srcGray.cols / grid, 0, srcGray.cols / grid, srcGray.rows));
		cv::Mat t1 = src1(Rect(i* srcGray.cols / grid, 0, srcGray.cols / grid, srcGray.rows));
		int threshold = otsu(t);
		printf("threshold = %d\n", threshold);
		//对图像二值化
		cv::Mat dst = cv::Mat::zeros(t.rows, t.cols, CV_8UC1);
		cvThreshold(&(IplImage)t, &(IplImage)dst, threshold, 255, CV_THRESH_BINARY);

		cv::Mat edgeimg = edge(dst);
		cv::imshow(" 1", edgeimg);
		cv::waitKey(5);
		on_HoughLines(edgeimg, t1);
		cv::imshow(" t1", t1);
		cv::waitKey(5);
		Rect rect(i* srcGray.cols / grid, 0, srcGray.cols / grid, srcGray.rows);
		cv::Mat tt = res(rect);
		t1.copyTo(tt);
	}
	cv::imwrite("4.jpg", res);
	cv::imshow("res", res);

	cv::waitKey(10);
#endif
}

void main(int argc, char **argv)
{
	VideoCapture cap("test2.mp4");
	cv::Mat frame;
	cap >> frame;
	clock_t start, end;
	while (!frame.empty())
	{
		cap >> frame;
		start = clock();
		SeaSkyDetector(frame);
		end = clock();
		cout << end - start << endl;
	}

	getchar();
}


Mat getHistograph(const Mat grayImage)
{
	//定义求直方图的通道数目，从0开始索引
	int channels[] = { 0 };
	//定义直方图的在每一维上的大小，例如灰度图直方图的横坐标是图像的灰度值，就一维，bin的个数
	//如果直方图图像横坐标bin个数为x，纵坐标bin个数为y，则channels[]={1,2}其直方图应该为三维的，Z轴是每个bin上统计的数目
	const int histSize[] = { 256 };
	//每一维bin的变化范围
	float range[] = { 0,256 };

	//所有bin的变化范围，个数跟channels应该跟channels一致
	const float* ranges[] = { range };

	//定义直方图，这里求的是直方图数据
	Mat hist;
	//opencv中计算直方图的函数，hist大小为256*1，每行存储的统计的该行对应的灰度值的个数
	calcHist(&grayImage, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);//cv中是cvCalcHist

																					 //找出直方图统计的个数的最大值，用来作为直方图纵坐标的高
	double maxValue = 0;
	//找矩阵中最大最小值及对应索引的函数
	minMaxLoc(hist, 0, &maxValue, 0, 0);
	//最大值取整
	int rows = cvRound(maxValue);
	//定义直方图图像，直方图纵坐标的高作为行数，列数为256(灰度值的个数)
	//因为是直方图的图像，所以以黑白两色为区分，白色为直方图的图像
	Mat histImage = Mat::zeros(rows, 256, CV_8UC1);

	//直方图图像表示
	for (int i = 0; i < 256; i++)
	{
		//取每个bin的数目
		int temp = (int)(hist.at<float>(i, 0));
		//如果bin数目为0，则说明图像上没有该灰度值，则整列为黑色
		//如果图像上有该灰度值，则将该列对应个数的像素设为白色
		if (temp)
		{
			//由于图像坐标是以左上角为原点，所以要进行变换，使直方图图像以左下角为坐标原点
			histImage.col(i).rowRange(Range(rows - temp, rows)) = 255;
		}
	}
	//由于直方图图像列高可能很高，因此进行图像对列要进行对应的缩减，使直方图图像更直观
	Mat resizeImage;
	resize(histImage, resizeImage, Size(256, 256));
	return resizeImage;
}
