//
// Created by zhuzhengming on 2021/11/21.
//

#include <stdlib.h>
#include <cv.h>
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

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    VideoCapture capture;
    capture.open(0);//打开 zed 相机ROS_WARN("*****START");

    ros::init(argc, argv, "camera_pub");//初始化 ROS 节点
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera", 1);

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
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", half_frame).toImageMsg();

        imshow("sd",half_frame);

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        waitKey(5);
    }
    return 0;
}