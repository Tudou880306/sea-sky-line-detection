// sea-sky line detection.cpp : �������̨Ӧ�ó������ڵ㡣
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

//otsu ��������㷨��ͨ���Ҷ�ֱ��ͼ���ҳ���ֵ��
//����k-means �����㷨
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

	//��ʼ��
	for (i = 0; i < 256; i++)
	{
		pixelCount[i] = 0;
		pixelPro[i] = 0;
	}

	//ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
	for (i = y; i < height; i++)
	{
		for (j = x; j < width; j++)
		{
			pixelCount[data[i * image.step + j]]++;
		}
	}

	//����ÿ������������ͼ���еı���
	for (i = 0; i < 256; i++)
	{
		pixelPro[i] = (float)(pixelCount[i]) / (float)(pixelSum);
	}

	//����ostu�㷨,�õ�ǰ���ͱ����ķָ�
	//�����Ҷȼ�[0,255],������������ĻҶ�ֵ,Ϊ�����ֵ
	float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
	for (i = 0; i < 256; i++)
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

		for (j = 0; j < 256; j++)
		{
			if (j <= i) //��������
			{
				//��iΪ��ֵ���࣬��һ���ܵĸ���
				w0 += pixelPro[j];
				u0tmp += j * pixelPro[j];
			}
			else       //ǰ������
			{
				//��iΪ��ֵ���࣬�ڶ����ܵĸ���
				w1 += pixelPro[j];
				u1tmp += j * pixelPro[j];
			}
		}

		u0 = u0tmp / w0;        //��һ���ƽ���Ҷ�
		u1 = u1tmp / w1;        //�ڶ����ƽ���Ҷ�
		u = u0tmp + u1tmp;        //����ͼ���ƽ���Ҷ�
								  //������䷽��
		deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
		//�ҳ������䷽���Լ���Ӧ����ֵ
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			threshold = i;
		}
	}
	//���������ֵ;
	return threshold;
}


//�Զ�ֵͼ������Ե��ȡ��û�в���canny
//�Զ�ֵͼ������������������������ȡ����ֵ��Ծ�߽���
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

//����任
/*���ܣ�������ͼ���ո�������Ҫ����ȡ�߶Σ�����lines�С�

	lines : ��һ��vector<Vec4i>, Vec4i��һ������4��int�������͵Ľṹ�壬[x1, y1, x2, y2], ���Ա�ʾһ���߶Ρ�

	rho : ����һ���뾶�ķֱ��ʡ� ������Ϊ��λ�ľ��뾫�ȡ� ��һ�����ݷ�ʽ��ֱ������ʱ�Ľ����ߴ�ĵ�λ�뾶��

	theta : �Ƕȷֱ��ʡ��Ի���Ϊ��λ�ĽǶȾ��ȡ���һ�����ݷ�ʽ��ֱ������ʱ�Ľ����ߴ�ĵ�λ�Ƕ�

	threshold : �ж�ֱ�ߵ�������ֵ���ۼ�ƽ�����ֵ��������ʶ��ĳ����Ϊͼ�е�һ��ֱ��ʱ�����ۼ�ƽ���б���ﵽ��ֵ�� ������ֵ threshold ���߶βſ��Ա����ͨ�������ص�����С�

	minLineLength���߶γ�����ֵ����Ĭ��ֵ0����ʾ����߶εĳ��ȣ�������趨�����̵��߶ξͲ��ܱ����ֳ�����

	minLineGap : �߶����������֮�����ֵ ��Ĭ��ֵ0������ͬһ�е����֮���������������ľ���
*/
static void on_HoughLines(cv::Mat src, cv::Mat dst)
{
	Mat midImage = src.clone();

	/*imshow("2", midImage);
	waitKey(15);*/
	//����HoughLinesP����  
	vector<Vec4i> mylines;
	HoughLinesP(midImage, mylines, 1, CV_PI / 180, 150 + 1, 50, 10);//�ֲ�otsu��ȫ��otsu�ֱ����ò�ͬ�Ĳ���
	//ѭ����������ÿһ���߶�  
	for (size_t i = 0; i < mylines.size(); i++)
	{
		Vec4i l = mylines[i];
		line(dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 0), 1, 8);
	}
}
//�Ҷ�ֱ��ͼ
Mat getHistograph(const Mat grayImage);

cv::Mat SeaSkyDetector(cv::Mat input)
{
	cv::Rect rect(0,200,1920,150);
	cv::Mat src1 = input(rect);
	cv::Mat srcGray;
	cvtColor(src1, srcGray, CV_BGR2GRAY);

	//��ֵ�˲�����������
	medianBlur(srcGray, srcGray, 3);


	//������պͺ�ˮ���ԵĻҶ�һ���Խ�ǿ�����������Ч����಻��
	//���ڹ��ղ����ȵ���������纣���ǿ����ȣ����þֲ�otsuЧ���Ϻ�

#if 1 //ȫ��otsu����ֵ

	int threshold = otsu(srcGray);
	printf("threshold = %d\n", threshold);
	//��ͼ���ֵ��
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

#else //�ֲ�otsu����ֵ
	cv::Mat res = cv::Mat::zeros(srcGray.rows, srcGray.cols, CV_8UC3);
	const int grid = 8;
	for (int i = 0; i < grid; i++)
	{

		cv::Mat t = srcGray(Rect(i* srcGray.cols / grid, 0, srcGray.cols / grid, srcGray.rows));
		cv::Mat t1 = src1(Rect(i* srcGray.cols / grid, 0, srcGray.cols / grid, srcGray.rows));
		int threshold = otsu(t);
		printf("threshold = %d\n", threshold);
		//��ͼ���ֵ��
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
	//������ֱ��ͼ��ͨ����Ŀ����0��ʼ����
	int channels[] = { 0 };
	//����ֱ��ͼ����ÿһά�ϵĴ�С������Ҷ�ͼֱ��ͼ�ĺ�������ͼ��ĻҶ�ֵ����һά��bin�ĸ���
	//���ֱ��ͼͼ�������bin����Ϊx��������bin����Ϊy����channels[]={1,2}��ֱ��ͼӦ��Ϊ��ά�ģ�Z����ÿ��bin��ͳ�Ƶ���Ŀ
	const int histSize[] = { 256 };
	//ÿһάbin�ı仯��Χ
	float range[] = { 0,256 };

	//����bin�ı仯��Χ��������channelsӦ�ø�channelsһ��
	const float* ranges[] = { range };

	//����ֱ��ͼ�����������ֱ��ͼ����
	Mat hist;
	//opencv�м���ֱ��ͼ�ĺ�����hist��СΪ256*1��ÿ�д洢��ͳ�Ƶĸ��ж�Ӧ�ĻҶ�ֵ�ĸ���
	calcHist(&grayImage, 1, channels, Mat(), hist, 1, histSize, ranges, true, false);//cv����cvCalcHist

																					 //�ҳ�ֱ��ͼͳ�Ƶĸ��������ֵ��������Ϊֱ��ͼ������ĸ�
	double maxValue = 0;
	//�Ҿ����������Сֵ����Ӧ�����ĺ���
	minMaxLoc(hist, 0, &maxValue, 0, 0);
	//���ֵȡ��
	int rows = cvRound(maxValue);
	//����ֱ��ͼͼ��ֱ��ͼ������ĸ���Ϊ����������Ϊ256(�Ҷ�ֵ�ĸ���)
	//��Ϊ��ֱ��ͼ��ͼ�������Ժڰ���ɫΪ���֣���ɫΪֱ��ͼ��ͼ��
	Mat histImage = Mat::zeros(rows, 256, CV_8UC1);

	//ֱ��ͼͼ���ʾ
	for (int i = 0; i < 256; i++)
	{
		//ȡÿ��bin����Ŀ
		int temp = (int)(hist.at<float>(i, 0));
		//���bin��ĿΪ0����˵��ͼ����û�иûҶ�ֵ��������Ϊ��ɫ
		//���ͼ�����иûҶ�ֵ���򽫸��ж�Ӧ������������Ϊ��ɫ
		if (temp)
		{
			//����ͼ�������������Ͻ�Ϊԭ�㣬����Ҫ���б任��ʹֱ��ͼͼ�������½�Ϊ����ԭ��
			histImage.col(i).rowRange(Range(rows - temp, rows)) = 255;
		}
	}
	//����ֱ��ͼͼ���и߿��ܸܺߣ���˽���ͼ�����Ҫ���ж�Ӧ��������ʹֱ��ͼͼ���ֱ��
	Mat resizeImage;
	resize(histImage, resizeImage, Size(256, 256));
	return resizeImage;
}
