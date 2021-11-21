//
// Created by zhuzhengming on 2021/11/21.
//

#include "ros/ros.h"
#include<geometry_msgs/Twist.h>
//#include <../../../devel/include/darknet_ros_msgs//BoundingBox.h>
//#include <../../../devel/include/darknet_ros_msgs//BoundingBoxes.h>
//
//void tracking(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){
//
//    float center_x = (float )((msg->bounding_boxes[0].xmin+msg->bounding_boxes[0].xmax)/2);
//    float center_y = (float )((msg->bounding_boxes[0].ymin+msg->bounding_boxes[0].ymax)/2);
//
//}

void control(void){

}

geometry_msgs::Twist cmd_vel;

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh("~");

//    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, tracking);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        control();
        pub.publish(cmd_vel);
        ros::spinOnce();
    }
    return 0;
}