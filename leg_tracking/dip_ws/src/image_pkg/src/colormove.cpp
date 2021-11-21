//
// Created by zhuzhengming on 2021/10/31.
//
#include <boost/thread.hpp>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include "math.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;

void sendCmd();
void  control(Mat input);
void analyseColor();


ros::Publisher cmd_pub;
geometry_msgs::Twist twist;

uchar max_function(uchar a, uchar b,uchar c)
{
    uchar max;
    uchar  temp = a>b ? a : b;
    max = temp > c? temp:c;
    return  max;
}

uchar min_function(uchar a, uchar b,uchar c)
{
    uchar min;
    uchar  temp = a<b ? a : b;
    min = temp < c ? temp : c;
    return  min;
}

void rgb_to_hsv(Mat input,Mat &output) {
    //normalize
    uchar min, max;
    uchar r, g, b;
    uchar v; //0-255
    float s;//0-1
    int h;//0-360


    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
            r = input.at<Vec3b>(i, j)[0];
            g = input.at<Vec3b>(i, j)[1];
            b = input.at<Vec3b>(i, j)[2];

            min = min_function(r, g, b);
            max = max_function(r, g, b);

            //v
            v = max;

            //s
            if (max == 0) s = 0;
            else s = ((float) (max - min) / max);

            //h
            if (max == min) h = 0;
            if (max == r && g >= b) h = (int) (60 * ((float) (g - b) / (max - min)));
            if (max == r && g < b) h = (int) (60 * ((float) (g - b) / (max - min)) + 360);
            if (max == g) h = (int) (60 * ((float) (b - r) / (max - min)) + 120);
            if (max == b) h = (int) (60 * ((float) (r - g) / (max - min)) + 240);

            //reflect on rgb and show up
            output.at<Vec3b>(i, j)[0] = (uchar) (255 * h / 360);
            output.at<Vec3b>(i, j)[1] = (uchar) (s * 255);
            output.at<Vec3b>(i, j)[2] = v;

        }

    }
}



//void color_segmentation(Mat input,Mat &output){
////Create trackbars in "Control" window
//
//    namedWindow("Control_bar", CV_WINDOW_AUTOSIZE); //create a window called "Control"
//
//    createTrackbar("LowH", "Control_bar", &iLowH, 255 ); //Hue (0 - 255)
//    createTrackbar("HighH", "Control_bar", &iHighH, 255 );
//
//    createTrackbar("LowS", "Control_bar", &iLowS, 255 ); //Saturation (0 - 255)
//    createTrackbar("HighS", "Control_bar", &iHighS, 255 );
//
//    createTrackbar("LowV", "Control_bar", &iLowV, 255); //Value (0 - 255)
//    createTrackbar("HighV", "Control_bar", &iHighV, 255);
//
//    //segmentation
//
//    for (int i = 0; i < input.rows; i++) {
//        for (int j = 0; j < input.cols; j++) {
//
//            if (input.at<Vec3b>(i,j)[0] > iLowH && input.at<Vec3b>(i,j)[0] < iHighH
//                && input.at<Vec3b>(i,j)[1] > iLowS && input.at<Vec3b>(i,j)[1] < iHighS
//                && input.at<Vec3b>(i,j)[2] > iLowV && input.at<Vec3b>(i,j)[2] < iHighV)
//            {
//                output.at<uchar>(i,j) = 255;
//            }
//            else{
//                output.at<uchar>(i,j) = 0;
//            }
//
//        }
//    }
//    imshow("color segmentation",output);
//}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ColorMove");//初始化 ROS 节点
    ros::NodeHandle n;
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);//定义速度发布器

//开一个线程,一直读摄像头数据
    boost::thread analyseColor_thread(&analyseColor); //自己编写代码识别颜色
//开一个线程,一直发速度,有颜色就发对应前后左右移动小车,没颜色就不发
    boost::thread sendCMd_thread(&sendCmd);//&sendCmd 发送速度的方法

    analyseColor_thread.join();//启动识别颜色线程
    sendCMd_thread.join();//启动发送速度线程

    return 0;
}


void sendCmd() {

    ros::Rate loop_rate(10);//发布消息频率,1 秒 10 次
    while (ros::ok()) {
//        if(redN < 80) {
////redN 图像程序输出的值
//            twist.linear.x = 0;//线速度
//            twist.linear.y = 0;
//            twist.linear.z = 0;
//            twist.angular.x = 0;
//            twist.angular.y = 0;
//            twist.angular.z = 0.4;//角速度
            //ROS_WARN("*****cmd");//调试使用,打印输出(可以看程序执行到哪里)
            cmd_pub.publish(twist); //发布消息
        }
        loop_rate.sleep(); //根据频率进行睡眠

}


void analyseColor() {

//识别图像, 改变 redN(图像程序输出的值)的值
//    VideoCapture capture;
//    capture.open(0);//打开 zed 相机
//    if (!capture.isOpened()) {
//        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
//    }
//    waitKey(1000);
//
//    Mat frame;//当前帧图片
    while (ros::ok()) {
//        capture.read(frame);
//        if (frame.empty()) {
//            break;
//        }

        Mat raw_frame = imread("/home/zhuzhengming/dip_ws/src/image_pkg/src/test3.jpg",CV_LOAD_IMAGE_COLOR);
        Mat frame;
        resize(raw_frame,frame,Size(640,480));

        Mat smooth = frame.clone();
        Mat hsv_space = frame.clone();

        //smooth
        GaussianBlur(frame, smooth, Size(3, 3), 1);
        //imshow("smooth", smooth);

        //rgb_to_hsi
        rgb_to_hsv(smooth, hsv_space);
        //imshow("hsv",hsv_space);

        //color_segmentation
//        Mat segmented_image = smooth.clone();
//        cvtColor(smooth, segmented_image, CV_RGB2GRAY);
//        color_segmentation(hsv_space, segmented_image);

        control(hsv_space);


    }
}

void  control(Mat input){
    int hist[4] = {0};
    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {

            //bule
            if (input.at<Vec3b>(i,j)[0] > 156 && input.at<Vec3b>(i,j)[0] < 180
                && input.at<Vec3b>(i,j)[1] > 43 && input.at<Vec3b>(i,j)[1] < 255
                && input.at<Vec3b>(i,j)[2] > 46 && input.at<Vec3b>(i,j)[2] < 255)
            {
                hist[0]++;
            }


            //green
            if (input.at<Vec3b>(i,j)[0] > 65 && input.at<Vec3b>(i,j)[0] < 82
                && input.at<Vec3b>(i,j)[1] > 43 && input.at<Vec3b>(i,j)[1] < 255
                && input.at<Vec3b>(i,j)[2] > 46 && input.at<Vec3b>(i,j)[2] < 255)
            {
                hist[1]++;
            }

            //red
            if (input.at<Vec3b>(i,j)[0] > 0 && input.at<Vec3b>(i,j)[0] < 45
                && input.at<Vec3b>(i,j)[1] > 43 && input.at<Vec3b>(i,j)[1] < 255
                && input.at<Vec3b>(i,j)[2] > 46 && input.at<Vec3b>(i,j)[2] < 255)
            {
                hist[2]++;
            }

            //yellow
            if (input.at<Vec3b>(i,j)[0] > 112 && input.at<Vec3b>(i,j)[0] < 129
                && input.at<Vec3b>(i,j)[1] > 43 && input.at<Vec3b>(i,j)[1] < 255
                && input.at<Vec3b>(i,j)[2] > 46 && input.at<Vec3b>(i,j)[2] < 255)
            {
                hist[3]++;
            }

        }
        ROS_INFO("%d,%d,%d,%d",hist[0],hist[1],hist[2],hist[3]);
    }

    //find out which color is most in the picture
    if(hist[0]>hist[1] && hist[0]>hist[2] && hist[0]>hist[3] )//red
    {
            twist.linear.x = 1.0;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;//角速度
    }

    if(hist[1]>hist[0] && hist[1]>hist[2] && hist[1]>hist[3] )//green
    {
            twist.linear.x = -1.0;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;//角速度
    }

    if(hist[2]>hist[0] && hist[2]>hist[1] && hist[2]>hist[3] )//blue
    {
        twist.linear.x = 0;//线速度
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = -0.4;//角速度
    }

    if(hist[3]>hist[0] && hist[3]>hist[1] && hist[3]>hist[2] )//yellow
    {
        twist.linear.x = 0;//线速度
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0.4;//角速度
    }

    if(hist[0]==0&&hist[1]==0&&hist[2]==0&&hist[3]==0)
    {
        twist.linear.x = 0;//线速度
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;//角速度
    }

}