//
// Created by zhuzhengming on 2021/11/21.
//

#include <stdlib.h>
#include <cv.h>
#include<string>
#include <highgui.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    VideoCapture capture;

    char key;
    char filename[200];
    int i = 1;
    capture.open(1);//打开 zed 相机ROS_WARN("*****START");

    ros::init(argc, argv, "camera_pub");//初始化 ROS 节点
    ros::NodeHandle n;

    if (!capture.isOpened()) {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
        return 0;
    }

    waitKey(1000);
    Mat frame;  //当前帧图片

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        capture.read(frame);
        if (frame.empty()) {
            break;
        }

        Mat half_frame = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));     //截取 zed 的左目图片

        imshow("sd",half_frame);

        string filename = "/home/zhuzhengming/picture/" +to_string(i)+".jpg";
        char key = waitKey(100);


        switch (key)
        {
            case'p':
                i++;
                imwrite(filename, half_frame);
                imshow("photo", half_frame);
                waitKey(500);
                destroyWindow("photo");
                break;
            default:
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
        waitKey(5);
    }
    return 0;
}