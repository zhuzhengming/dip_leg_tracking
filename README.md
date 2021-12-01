# Leg Tracking System

### preface: use limited computing resources and storage resources to achieve low-cost real-time target detection and tracking.



# specialty:

- #### Low-cost embedded development.

  - #### 传统的目标检测方法，受光线影响较大，基于当下流行的深度学习算法，测试低成本嵌入式开发的实际性能，更加贴近机器人开发实际工程，包含硬件：嵌入式单片机（底盘驱动），嵌入式计算机（jetson nano）。软件：linux系统，ROS系统等。

  - #### jetson nano: 

    - #### Low price.

    - #### Low power consumption.

    - #### 128 core Maxwell.(Maxwell 构架GPU核心)

    - #### 4GB memory

    - #### 4core ARM A57 CPU

    - #### 低成本，低功耗，体积小，但是拥有GPU，可以搭载一些轻量级神经网络来实现边缘AI计算研发。

  - #### 嵌入式开发的方便之处：

    - #### Remote PC communication over SSH in LAN（局域网内SSH协议远程PC通讯,远程调试机器人便利）

    - #### Automatic startup.（通过启动脚本，上电启动整个系统）

- #### Machine vision based on lightweight neural network.

  - ##### Considering limited computing resources and storage resources, I use a lightweight neural network model.

- #### Connect all parts with ROS.

  - ##### All parts connect each other via ROS.





- # Project Structure

![pic1](https://github.com/zhuzhengming/dip_leg_tracking/picture/pic1.png)

- ### Details of all parts:

  - ##### Environment Configuration

    - darknet
    - opencv
    - Automatic startup
    - remote communication configuration with PC : static IP, ssh

  - ##### lightweight deep learning models

    - **The current mainstream target detection algorithms are mainly divided into two categories:**

      - Two stage :  Generate Region Proposal (candidate area), and classify the candidate area on this basis.
      - One stage :Excluding the candidate region stage of the Two-stage algorithm, the position coordinate value and category probability of the object are directly obtained.Representative algorithms such as YOLO(You only look once)

    - YOLOv3-tiny :

      - ![pic2](https://github.com/zhuzhengming/dip_leg_tracking/picture/pic2.png)

      The backbone network has 7 3×3 convolutional layers, 6 pooling layers.

    - DIY leg training dataset.

      

  - Communication via ROS

    - 图片如下:

      ![pic3](https://github.com/zhuzhengming/dip_leg_tracking/picture/pic3.png)

  - pid tracking: 

    - pid controller.

  

- ### Result And Analyse

  - #### result: 

    - #### video as foloows:https://youtu.be/zSw16Put6mo

    - FPS is low, which is about 5-6 fps.  The system can only realize real-time tracking at a low speed. 

  - #### analyze

    - limited computing resources
    - lacking of virtual memory 

- ### Improvement

  - In order to provide more resources for the YOLOv3-Tiny detection algorithm to run, it is necessary to increase virtual memory swap and increase 4G virtual memory. At the same time, in order to further reduce the waste of resources, the ubuntu graphical user interface is closed. After finishing these preparations, the test was conducted again, and the number of frames was increased to 20fps. The overall picture was relatively smooth, and it was able to recognize objects in real time.

  - **close the ubuntu graphical user interface **

  - ##### increase virtual memory swap.

