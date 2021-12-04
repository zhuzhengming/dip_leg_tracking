//
// Created by zhuzhengming on 2021/11/21.
//

#include "ros/ros.h"
#include<geometry_msgs/Twist.h>
#include </home/zhuzhengming/workspace/devel/include/darknet_ros_msgs/BoundingBox.h>
#include </home/zhuzhengming/workspace/devel/include/darknet_ros_msgs/BoundingBoxes.h>

#define  frame_rows 480
#define frame_cols 640

typedef struct{
    float KP;                                                                                   //PID参数P
    float KI;                                                                                   //PID参数I
    float KD;                                                                                   //PID参数D
    float fdb;                                                                                  //PID反馈值
    float ref;                                                                                  //PID目标值
    float cur_error;                                                                    //当前误差
    float error[2];                                                                             //前两次误差
    float output;                                                                               //输出值 
    float outputMax;                                                                    //最大输出值的绝对值 
    
}PID_t;

void my_init(){
    smoother_cmd_vel.linear.x = 0.3; //keep straight moving
    smoother_cmd_vel.linear.y = 0;
    smoother_cmd_vel.linear.z = 0;
    smoother_cmd_vel.angular.x = 0;
    smoother_cmd_vel.angular.y = 0;
    smoother_cmd_vel.angular.z = 0;

    pid.ref = frame_cols/2;
    pid.outputMax = 0.6;

}


void pid_control(){

    pid.cur_error = pid.ref - pid.fdb;
        pid.output += pid.KP * (pid.cur_error - pid.error[1]) + pid.KI * pid.cur_error + pid.KD * 
(pid.cur_error - 2 * pid.error[1] + pid.error[0]);
        pid.error[0] = pid.error[1];
        pid.error[1] = pid.ref - pid.fdb;

        /*设定输出上限*/
        if(pid.output > pid.outputMax) pid.output = pid.outputMax;
        if(pid.output < -pid.outputMax) pid.output = -pid.outputMax;

    smoother_cmd_vel.angular.z = pid.output;
}

void tracking(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){

     obj_center_x = (float )((msg->bounding_boxes[0].xmin+msg->bounding_boxes[0].xmax)/2);
     obj_center_y = (float )((msg->bounding_boxes[0].ymin+msg->bounding_boxes[0].ymax)/2);
     pid.fdb = obj_center_x;

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh("~");
    my_init();

    nh.param<float>("pid_KP",pid.KP,0.005);
    nh.param<float>("pid_KI",pid.KI,0);
    nh.param<float>("pid_KD",pid.KD,0.00002);

    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, tracking);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);

    ros::Rate loop_rate(50);

    while (ros::ok()) {

        pid_control();
        pub.publish(smoother_cmd_vel);
        loop_rate.sleep();
        ros::spinOnce();
 }
    return 0;
}

