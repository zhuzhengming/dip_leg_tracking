//
// Created by ZhuZhengMing on 2021/10/25.
//

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


#define LINEAR_X 0
#define PI 3.1415926535
using namespace cv;


//直线参数
typedef struct hline_t
{
    int p;//极径
    int theta;//极角，角度
}hline;

//circle
typedef struct hcircle_t
{
    int theta;
    int x_center;
    int y_center;
}hcircle;


//////////////////////边缘检测//////////////////

//gauss filter
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

                //初始化
                output.at<uchar>(i,j) = 0;

                for(int m = 0;m<size;++m){
                    for(int n = 0;n<size;++n){

                        output.at<uchar>(i,j) +=  arr[m][n] * input.at<uchar>(i+1-m,j+1-n);

                    }
                }
            }
        }
    }

}


//calculate gradient on X and Y
void SobelGradDirection(const Mat imageSource,Mat &imageSobelX,Mat &imageSobelY,double *&pointDirection)
{

    pointDirection = new double[(imageSource.rows)*(imageSource.cols)];
    //init gradient angel
    for(int i=0;i<(imageSource.rows)*(imageSource.cols);i++)
    {
        pointDirection[i]=0;
    }

    imageSobelX=Mat::zeros(imageSource.size(),CV_32SC1);
    imageSobelY=Mat::zeros(imageSource.size(),CV_32SC1);

    uchar *P=imageSource.data;
    uchar *PX=imageSobelX.data;
    uchar *PY=imageSobelY.data;

    int step=imageSource.step;
    int stepXY=imageSobelX.step;
    int k=0;

    for(int i=1;i<(imageSource.rows-1);i++)
    {
        for(int j=1;j<(imageSource.cols-1);j++)
        {
            //通过指针遍历图像上每一个像素
            double gradY=P[(i-1)*step+j+1]+P[i*step+j+1]*2+P[(i+1)*step+j+1]-P[(i-1)*step+j-1]-P[i*step+j-1]*2-P[(i+1)*step+j-1];

            PY[i*stepXY+j*(stepXY/step)]=abs(gradY);
            double gradX=P[(i+1)*step+j-1]+P[(i+1)*step+j]*2+P[(i+1)*step+j+1]-P[(i-1)*step+j-1]-P[(i-1)*step+j]*2-P[(i-1)*step+j+1];

            PX[i*stepXY+j*(stepXY/step)]=abs(gradX);
            if(gradX==0)
            {
                gradX=0.00000000000000001;  //防止除数为0异常
            }

            pointDirection[k]=atan(gradY/gradX)*57.3;//弧度转换为度
            pointDirection[k]+=90;
            k++;
        }
    }

    convertScaleAbs(imageSobelX,imageSobelX);
    convertScaleAbs(imageSobelY,imageSobelY);
}

//imageGradX^2+imageGradY^2
void SobelAmplitude(const Mat imageGradX,const Mat imageGradY,Mat &SobelAmpXY)
{
    SobelAmpXY=Mat::zeros(imageGradX.size(),CV_32FC1);

    for(int i=0;i<SobelAmpXY.rows;i++)
    {
        for(int j=0;j<SobelAmpXY.cols;j++)
        {
            SobelAmpXY.at<float>(i,j)=sqrt(imageGradX.at<uchar>(i,j)*imageGradX.at<uchar>(i,j)+imageGradY.at<uchar>(i,j)*imageGradY.at<uchar>(i,j));
        }
    }
    convertScaleAbs(SobelAmpXY,SobelAmpXY);
}

//Sobel third step: restrain none max
void LocalMaxValue(const Mat imageInput,Mat &imageOutput,double *pointDirection)
{
    imageOutput=imageInput.clone();
    int k=0;
    for(int i=1;i<imageInput.rows-1;i++)
    {
        for(int j=1;j<imageInput.cols-1;j++)
        {
            int value00=imageInput.at<uchar>((i-1),j-1);
            int value01=imageInput.at<uchar>((i-1),j);
            int value02=imageInput.at<uchar>((i-1),j+1);
            int value10=imageInput.at<uchar>((i),j-1);
            int value11=imageInput.at<uchar>((i),j);
            int value12=imageInput.at<uchar>((i),j+1);
            int value20=imageInput.at<uchar>((i+1),j-1);
            int value21=imageInput.at<uchar>((i+1),j);
            int value22=imageInput.at<uchar>((i+1),j+1);

            if(pointDirection[k]>0&&pointDirection[k]<=45)
            {
                if(value11<=(value12+(value02-value12)*tan(pointDirection[i*(imageOutput.rows-1)+j]))||(value11<=(value10+(value20-value10)*tan(pointDirection[i*(imageOutput.rows-1)+j]))))
                {
                    imageOutput.at<uchar>(i,j)=0;
                }
            }
            if(pointDirection[k]>45&&pointDirection[k]<=90)

            {
                if(value11<=(value01+(value02-value01)/tan(pointDirection[i*(imageOutput.cols-1)+j]))||value11<=(value21+(value20-value21)/tan(pointDirection[i*(imageOutput.cols-1)+j])))
                {
                    imageOutput.at<uchar>(i,j)=0;

                }
            }
            if(pointDirection[k]>90&&pointDirection[k]<=135)
            {
                if(value11<=(value01+(value00-value01)/tan(180-pointDirection[i*(imageOutput.cols-1)+j]))||value11<=(value21+(value22-value21)/tan(180-pointDirection[i*(imageOutput.cols-1)+j])))
                {
                    imageOutput.at<uchar>(i,j)=0;
                }
            }
            if(pointDirection[k]>135&&pointDirection[k]<=180)
            {
                if(value11<=(value10+(value00-value10)*tan(180-pointDirection[i*(imageOutput.cols-1)+j]))||value11 <= (value12+(value22-value12)*tan(180-pointDirection[i*(imageOutput.cols-1)+j])))
                {
                    imageOutput.at<uchar>(i,j)=0;
                }
            }
            k++;
        }
    }
}

//double threshold
void DoubleThreshold(Mat &imageInput,double lowThreshold,double highThreshold)
{
    for(int i=0;i<imageInput.rows;i++)
    {
        for(int j=0;j<imageInput.cols;j++)
        {
            if(imageInput.at<uchar>(i,j)>highThreshold)
            {
                imageInput.at<uchar>(i,j)=255;
            }
            if(imageInput.at<uchar>(i,j)<lowThreshold)
            {
                imageInput.at<uchar>(i,j)=0;
            }
        }
    }


}

//link threshold

void DoubleThresholdLink(Mat &imageInput,double lowThreshold,double highThreshold)
{
    for(int i=1;i<imageInput.rows-1;i++)
    {
        for(int j=1;j<imageInput.cols-1;j++)
        {
            if(imageInput.at<uchar>(i,j)>lowThreshold&&imageInput.at<uchar>(i,j)<255)
            {
                if(imageInput.at<uchar>(i-1,j-1)==255||imageInput.at<uchar>(i-1,j)==255||imageInput.at<uchar>(i-1,j+1)==255||
                   imageInput.at<uchar>(i,j-1)==255||imageInput.at<uchar>(i,j)==255||imageInput.at<uchar>(i,j+1)==255||
                   imageInput.at<uchar>(i+1,j-1)==255||imageInput.at<uchar>(i+1,j)==255||imageInput.at<uchar>(i+1,j+1)==255)
                {
                    imageInput.at<uchar>(i,j)=255;
                    DoubleThresholdLink(imageInput,lowThreshold,highThreshold); //递归调用
                }
                else
                {
                    imageInput.at<uchar>(i,j)=0;
                }
            }
        }
    }
}


//边缘检测函数 :canny
void EdgeDetector(Mat input, Mat &output){

    //first step gauss filter
    Mat Gauss_image = input.clone();
    Gaussian(input,Gauss_image,1.5,5);
//    imshow("gauss",Gauss_image);
//    waitKey(5);

    //second step Sobel
    Mat imageSobelY = input.clone();
    Mat imageSobelX = input.clone();

    double *pointDirection = new double[(imageSobelX.cols)*(imageSobelY.rows)];    //gradient direction
    SobelGradDirection(Gauss_image,imageSobelX,imageSobelY,pointDirection);
//    imshow("imageSobelX",imageSobelX);
//    waitKey(5);

    Mat SobelGradAmpl;
    SobelAmplitude(imageSobelX,imageSobelY,SobelGradAmpl);

//    imshow("SobelGradAmpl",SobelGradAmpl);
//    waitKey(5);

    //third step :  restrain none max
    Mat imageLocalMax;
    LocalMaxValue(SobelGradAmpl,imageLocalMax,pointDirection);

//    imshow("imageLocalMax",imageLocalMax);
//    waitKey(5);

    //fourth step : double threshold and link

    DoubleThreshold(imageLocalMax,60,160);        //双阈值处理
    DoubleThresholdLink(imageLocalMax,60,160);   //双阈值中间阈值滤除及连接
//    imshow("imageLocalMax",imageLocalMax);
//    waitKey(5);

    output = imageLocalMax.clone();

}

//////////////////////霍夫线变换//////////////////


void Hough_Line(Mat input) {
//    //r=x*cos(theta)+y*sin(theta);
//    hline* lines;
//
//    int Size = (int) 2 * (sqrt(output.rows * output.rows + output.cols * output.cols)) + 100;
//    int offset = Size / 2;
//
//    int R;
//
//    //apply two dimension array
//    int **scoreboard = (int **) malloc(Size * sizeof(int *));
//
//    if (!scoreboard) {
//        std::cout << "error" << std::endl;
//    }
//
//    for (int i = 0; i < Size; i++) {
//
//        scoreboard[i] = (int *) malloc(181 * sizeof(int));
//
//        if (scoreboard[i] == NULL) {
//            std::cout << "error" << std::endl;
//        }
//        memset(scoreboard[i], 0, 181 * sizeof(int));//init with 0's
//    }
//
//    for (int x = 0; x < output.rows; x++) {
//        for (int y = 0; y < output.cols; y++) {
//            if (output.at<uchar>(x, y) == 255)  //black point
//            {
//                //one degree as a step
//                for (int angle = 0; angle < 181; angle++) {
//                    R = x * cos(angle * PI / 180.0) + y * sin(angle * PI / 180.0) + offset;
//
//                    scoreboard[R][angle]++;
//                }
//            }
//        }
//    }
//
//    for (int i = 0; i < Size; i++) {
//        for (int j = 0; j < 181; j++) {
//            std::cout<<scoreboard[i][j]<<std::endl;
//        }
//    }
//
//    //find line via threshold
//    int Max = 0;
//    int kp = 0;
//    int kt = 0;
//    int count = 0;
//    for (int i = 0; i < Size; i++)//p
//    {
//        for (int k = 0; k < 181; k++)//angle
//        {
//            if (scoreboard[i][k] > Max) {
//                Max = scoreboard[i][k];
//
//                kp = i - offset;
//                kt = k;
//            }
//
//            if (scoreboard[i][k] >= threshold) {
//                lines[count].p = i - Size / 2;
//                lines[count].theta = k;
//                count++;
//            }
//        }
//    }
//
//    //释放资源
//    for (int m = 0; m < Size; m++)
//    {
//        free(scoreboard[m]);
//    }
//    free(scoreboard);
//
//    drawDetectLines(src_image,lines,count);

//CV_method
    Mat  contuors,res ;
    Canny(input,contuors,100,300);
    cvtColor(contuors,res,CV_GRAY2BGR);
    std::vector<Vec4i> lines;
    HoughLinesP(contuors,lines,1,PI/180,80,50,10);

    for (size_t i= 0; i <lines.size() ; i++) {
        Vec4i l = lines[i];
        int start_point_x = l[0];
        int start_point_y = l[1];
        int end_point_x = l[2];
        int end_point_y = l[3];
        line(input,Point(start_point_x,start_point_y),Point(end_point_x,end_point_y),Scalar(0,255,0),2);
    }

    imshow("hough_line",input);

}



//////////////////////霍夫圆变换//////////////////
void Hough_Circle(Mat input){
    Mat contuors , gray_image;
//    Canny(input,contuors,100,200);
    cvtColor(input,gray_image,CV_RGB2GRAY);
//    EdgeDetector(gray_image,contuors);
//    imshow("canny_opencv",contuors);

//int R_max = max(input.rows,input.cols);
//int Size = (int)sqrt(input.rows*input.rows+input.cols*input.cols);
//int R;
//
//    for (int row = 0; row < contuors.rows; row++) {   //rows
//        for (int col = 0; col < contuors.cols; col++) {   //clos
//
//            if (contuors.at<uchar>(row, col) != 0) {
//
//                for (int r = 0; r < Size; r++){   // radius
//                    for (int angle = 0; angle < 181; angle++) { //angel
//                        R = pow((row - r* cos(angle)),2)+pow((col - r* sin(angle)),2);
//
//                    }
//            }
//        }
//        }
//    }
//}

    std::vector<Vec3f> cir;
    HoughCircles(gray_image,cir,CV_HOUGH_GRADIENT,1,50,200,60,40,200);
    for(size_t i = 0;i<cir.size();i++)
    {
        float x_center = cir[i][0];
        float y_center = cir[i][1];
        float radius = cir[i][2];
        circle(input,Point(x_center,y_center),radius,Scalar(255,0,0),2,8);
    }

    imshow("hough_circle",input);

}


int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(0);//打开 zed 相机
//    capture.open("rtsp://admin:admin@192.168.16.15:554//Streaming/Channels/1");

    ROS_WARN("*****START");
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
    Mat raw_frame;  //当前帧图片
    int nFrames = 0;    //图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);  //图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);    //图片高

    while (ros::ok())
    {
        capture.read(raw_frame);

        if(raw_frame.empty())
        {
            break;
        }

        //Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));     //截取 zed 的左目图片

        //open hk_vision camera



        Mat frIn = raw_frame;   //notebook camera

        Mat grey_image =frIn.clone();

        // 灰度图转换
        cvtColor(raw_frame,grey_image,COLOR_BGR2GRAY);
        imshow("grey",grey_image);

        Mat canny_image = grey_image.clone();
        Mat hough_Line_image = grey_image.clone();
        Mat hough_circle_image = grey_image.clone();

        // 边缘检测函数

//        EdgeDetector(grey_image,canny_image);
//        imshow("canny",canny_image);


        // 线检测
//        canny
//        Hough_Line(raw_frame);

        // 圆检测
//        Hough_Circle(raw_frame) ;

        geometry_msgs::Twist cmd_red;
// 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;
        pub.publish(cmd_red);
        ros::spinOnce();
        waitKey(5);
    }

return 0;
}