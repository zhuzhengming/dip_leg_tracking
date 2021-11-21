//
// Created by zhuzhengming on 2021/10/31.
//

#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
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

void rgb_to_hsv(Mat input,Mat &output){
    //normalize
    uchar min,max;
    uchar r,g,b;
    uchar v; //0-255
    float s;//0-1
    int h;//0-360


    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {
             r = input.at<Vec3b>(i,j)[0];
             g = input.at<Vec3b>(i,j)[1];
             b = input.at<Vec3b>(i,j)[2];

            min = min_function(r,g,b);
            max = max_function(r,g,b);

            //v
            v = max;

            //s
            if(max == 0)    s = 0;
            else    s = ((float )(max-min)/max);

            //h
            if (max == min)     h = 0;
            if(max == r && g >= b)      h = (int)(60*((float)(g-b)/(max - min)));
            if(max == r && g < b)       h = (int)(60*((float)(g-b)/(max - min))+360);
            if(max == g)     h = (int)(60*((float)(b-r)/(max - min))+120);
            if (max == b)       h = (int)(60*((float )(r-g)/(max - min))+240);

            //reflect on rgb and show up
            output.at<Vec3b>(i,j)[0] = (uchar)( 255 * h / 360);
            output.at<Vec3b>(i,j)[1] = (uchar)(s * 255);
            output.at<Vec3b>(i,j)[2] =  v;

        }

    }

}



//parameter
//bule
//int iLowH = 0;
//int iHighH = 45;
//
//int iLowS = 43;
//int iHighS = 255;
//
//int iLowV = 46;
//int iHighV = 255;

//green
//int iLowH = 60;
//int iHighH = 82;
//
//int iLowS = 43;
//int iHighS = 255;
//
//int iLowV = 46;
//int iHighV = 255;

//red(stable)
int iLowH = 156;
int iHighH = 180;

int iLowS = 43;
int iHighS = 255;

int iLowV = 46;
int iHighV = 255;

//yellow
//int iLowH = 112;
//int iHighH = 129;
//
//int iLowS = 43;
//int iHighS = 255;
//
//int iLowV = 46;
//int iHighV = 255;

//white
//int iLowH = 0;
//int iHighH = 360;
//
//int iLowS = 0;
//int iHighS = 30;
//
//int iLowV = 221;
//int iHighV = 255;



void color_segmentation(Mat input,Mat &output){
//Create trackbars in "Control" window

    namedWindow("Control_bar", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    createTrackbar("LowH", "Control_bar", &iLowH, 255 ); //Hue (0 - 255)
    createTrackbar("HighH", "Control_bar", &iHighH, 255 );

    createTrackbar("LowS", "Control_bar", &iLowS, 255 ); //Saturation (0 - 255)
    createTrackbar("HighS", "Control_bar", &iHighS, 255 );

    createTrackbar("LowV", "Control_bar", &iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control_bar", &iHighV, 255);

    //segmentation

    for (int i = 0; i < input.rows; i++) {
        for (int j = 0; j < input.cols; j++) {

            if (input.at<Vec3b>(i,j)[0] > iLowH && input.at<Vec3b>(i,j)[0] < iHighH
            && input.at<Vec3b>(i,j)[1] > iLowS && input.at<Vec3b>(i,j)[1] < iHighS
            && input.at<Vec3b>(i,j)[2] > iLowV && input.at<Vec3b>(i,j)[2] < iHighV)
            {
                output.at<uchar>(i,j) = 255;
            }
            else{
                output.at<uchar>(i,j) = 0;
            }

        }
    }
    imshow("color segmentation",output);
}

void draw_hist(Mat input){
   Mat extract_hsv_image = Mat::zeros(input.size(),CV_8UC3);
    rgb_to_hsv(input,extract_hsv_image);
    int hist[4] = {0};

    for (int i = 0; i < extract_hsv_image.rows; i++) {
        for (int j = 0; j < extract_hsv_image.cols; j++) {

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
    }

    Mat histogram = Mat::zeros(480,640,CV_8UC3);
    rectangle(histogram,Point(100,420),Point(200,420-(int)(0.01*hist[0])),Scalar(255,0,0),-1,8);
    rectangle(histogram,Point(200,420),Point(300,420-1*(int)(0.01*hist[1])),Scalar(48,139,87),-1,8);
    rectangle(histogram,Point(300,420),Point(400,420-1*(int)(0.01*hist[2])),Scalar(0,0,255),-1,8);
    rectangle(histogram,Point(400,420),Point(500,420-1*(int)(0.01*hist[3])),Scalar(0,255,255),-1,8);

    cout<<hist[0]<<","<<hist[1]<<","<<hist[2]<<","<<hist[3]<<endl;
    imshow("histogram",histogram);
}

void object_detection(Mat input,Mat src){

    //find contours
    vector<vector<Point>> contours;
    findContours(input,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

    //draw contours
    Mat contours_frame = Mat::zeros(input.size(),CV_8UC3);
    drawContours(contours_frame,contours,-1,Scalar(0,255,255),2,8);
    //imshow("contour", contours_frame);

    //describe objects

//    Mat described_image = Mat::zeros(input.size(),CV_8UC3);

    //min_circle
    vector<Point2f> circleCenters(contours.size());//center points cluster
    vector<float> circleRadius(contours.size());//radius points clusters
    //find and draw minimum circle
    for(int i = 0; i < contours.size();i++)
    {
            minEnclosingCircle(contours[i], circleCenters[i], circleRadius[i]);//find

            //remove small circle due to noise
        if(circleRadius[i]>40) {
            circle(src, circleCenters[i], circleRadius[i], Scalar(0, 0, 255), 2, 8);//draw
        }
        }

    //poly_describe
    vector<vector<Point>> contours_poly(contours.size());
    for (int i = 0; i<contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 2, true);
        drawContours(src, contours_poly, i, Scalar(0, 0, 0), 2, 8);
    }

     imshow("describe object",src);

Mat extract_image = Mat::zeros(src.size(),CV_8UC3);

//extract image in the contours
for(int i = 0; i < src.rows;i++){
        for (int j = 0; j < src.cols; j++) {
            for (int k = 0; k < contours_poly.size(); k++) {
                if(pointPolygonTest(contours_poly[k],Point(j,i), false) == 1
                || pointPolygonTest(contours_poly[k],Point(j,i), false) == 0)  //whether this point is in the contours or not
                {
                    extract_image.at<Vec3b>(i,j)[0] = src.at<Vec3b>(i,j)[0];
                    extract_image.at<Vec3b>(i,j)[1] = src.at<Vec3b>(i,j)[1];
                    extract_image.at<Vec3b>(i,j)[2] = src.at<Vec3b>(i,j)[2];
                }
            }
        }
    }

    imshow("extracted image",extract_image);

    draw_hist(extract_image);

}

int main(int argc, char **argv) {
    VideoCapture capture;
    capture.open(0);//打开 zed 相机
    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack");//初始化 ROS 节点
    ros::NodeHandle n;

    if (!capture.isOpened()) {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    Mat frame;//当前帧图片
    int nFrames = 0;//图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
    while (ros::ok()) {
        capture.read(frame);
        if (frame.empty()) {
            break;
        }
        //Mat raw_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        Mat raw_frame = imread("/home/zhuzhengming/dip_ws/src/image_pkg/src/test2.jpg",CV_LOAD_IMAGE_COLOR);
        Mat pic_frame;
        resize(raw_frame,pic_frame,Size(640,480));
        Mat smooth = pic_frame.clone();
        Mat hsv_space = pic_frame.clone();
        //image process
        GaussianBlur(pic_frame, smooth, Size(3, 3), 1);
        imshow("smooth", smooth);


        //rgb_to_hsi
        rgb_to_hsv(smooth,hsv_space);
        //imshow("hsv",hsv_space);

        //color_segmentation
        Mat segmented_image = smooth.clone();
        cvtColor(smooth,segmented_image, CV_RGB2GRAY);
        color_segmentation(hsv_space,segmented_image);

        //object_detection
        object_detection(segmented_image,smooth);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}

