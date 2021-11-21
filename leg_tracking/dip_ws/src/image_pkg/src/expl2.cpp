#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"


#define LINEAR_X 0
using namespace cv;

//////////////////////滤波//////////////////
// 空域高斯滤波器函数

void Gaussian(Mat &input, Mat &output, double sigma, int size)
{
    //创建高斯滤波器，size*size
    double arr[size][size];
	double sum = 0.0;
	// double sigma = 1.5;     //可以改变来调整效果

	for (int i = 0; i < size; ++i){
		for (int j = 0; j < size; ++j){ 
			 arr[i][j] = exp(-((i - 1)*(i - 1) + (j - 1)*(j - 1)) / (2 * pow(sigma,2)));      //2维高斯公式
             sum += arr[i][j];  
        }
    }

	for (int i = 0; i < size; ++i){
		for (int j = 0; j < size; ++j){
			arr[i][j] /= sum;       //得到相应的权值矩阵
        }
    }
        
    //处理输入的图像

    for(int i=0;i < input.rows;++i){
        for(int j=0;j<input.cols;++j){
            if(i>1  &&  i<input.rows-1  &&  j > 1 &&  j<input.cols)//边缘不处理
            {
                //对BGR通道分别处理

                //初始化
                    output.at<Vec3b>(i,j)[0] = 0;
                    output.at<Vec3b>(i,j)[1] = 0;
                    output.at<Vec3b>(i,j)[2] = 0;

                    for(int m = 0;m<size;++m){
                        for(int n = 0;n<size;++n){
                            
                            output.at<Vec3b>(i,j)[0] +=  arr[m][n] * input.at<Vec3b>(i+1-m,j+1-n)[0];
                            output.at<Vec3b>(i,j)[1] +=  arr[m][n] * input.at<Vec3b>(i+1-m,j+1-n)[1];
                            output.at<Vec3b>(i,j)[2] +=  arr[m][n] * input.at<Vec3b>(i+1-m,j+1-n)[2];
                        }
                    }
            }
        }
    }

}



//////////////////////形态学//////////////////

// 膨胀函数

void Dilate(Mat Src,  Mat Dst , uchar size){

uchar size_half = (size-1)/2;

for (int i = 0; i < Src.rows; ++i)
	{
		for (int j = 0; j < Src.cols; ++j)
		{	
			uchar maxV = 0;
		
			//遍历周围最大像素值,size*size
			for (int yi = i-size_half; yi <= i+size_half; yi++)
			{
				for (int xi = j-size_half; xi <= j+size_half; xi++)
				{					
					if (xi<0||xi>= Src.cols|| yi<0 || yi >= Src.rows)
					{
						continue;
					}					
					maxV = (std::max<uchar>)(maxV, Src.at<uchar>(yi, xi));			
				}
			}
			Dst.at<uchar>(i, j) = maxV;
        }
}
}

// 腐蚀函数

void Erode(Mat Src, Mat Dst,uchar size){

uchar size_half = (size-1)/2;

for (int i = 0; i < Src.rows; ++i)
	{
		for (int j = 0; j < Src.cols; ++j)
		{	
			uchar minV = 255;
		
			//遍历周围最小像素值,size*size 
			for (int yi = i-size_half; yi <= i+size_half; yi++)
			{
				for (int xi = j-size_half; xi <= j+size_half; xi++)
				{					
					if (xi<0||xi>= Src.cols|| yi<0 || yi >= Src.rows)
					{
						continue;
					}					
					minV = (std::min<uchar>)(minV, Src.at<uchar>(yi, xi));	//返回最小值
				}
			}
			Dst.at<uchar>(i, j) = minV;
        }
    }
}

int main(int argc, char **argv)
{
VideoCapture capture;
capture.open(0);//打开 zed 相机ROS_WARN("*****START");
ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
ros::NodeHandle n;

// ros::Rate loop_rate(10);//定义速度发布频率

ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器

if (!capture.isOpened())
{
    printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
    return 0;
}
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高


while (ros::ok())
{
    capture.read(frame);

    if(frame.empty())
        {
        break;
        }

     Mat frIn = frame;     //使用笔记本摄像头
    //Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));     //截取 zed 的左目图片

    Mat grey_image = frIn.clone();

    cvtColor(frIn,grey_image,COLOR_BGR2GRAY);   //转化为灰度图

    Mat Gauss_frame = frIn.clone();    //高斯滤波
    Mat Freq_frame= frIn.clone();     //频域滤波
    
    Mat Dilate_frame= grey_image.clone();   //膨胀
    Mat Erode_frame= grey_image.clone();    //腐蚀

    uchar Dilate_size = 7;
    uchar Erode_size = 7;

    // 空域滤波函数
     Gaussian(frIn,Gauss_frame,1.5,5);
     

    // // 频域滤波函数
    // freqfilt();

    // // 膨胀函数
    Dilate(grey_image,Dilate_frame,Dilate_size);
    

    // // 腐蚀函数
    Erode(grey_image,Erode_frame,Erode_size);

    // imshow("1",frame);
    // imshow("raw_frame",frIn);
    imshow("grey_frame",grey_image);
    // imshow("Gaussian",Gauss_frame);
    imshow("Dilate",Dilate_frame);
    imshow("Erode",Erode_frame);

    ros::spinOnce();
    waitKey(5);
}
return 0;
}