#ifndef __CAMERA_UAV_H__
#define __CAMERA_UAV_H__

#include "single_uav.h"
#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp> 
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/highgui/highgui.hpp>  
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>  




class CameraUAV: public UAV
{
    public:

    const int DETECT_CIRCLE_LIMIT = 10;
    const float FLY_STEP_MAX = 0.2;
    const float FLY_STEP_MIN = -0.2;
    const float FLY_STEP = 0.005;

    bool detect_circle = false;
    int detect_circle_times = 0;

    cv::Point circle_center;
    cv::Point image_center;

    Eigen::Vector3d target_speed;

    
    CameraUAV(char* name, const ros::NodeHandle &nh_):UAV(name, nh_){
        ROS_INFO("===== UAV %s init =====", name);
        image_transport::ImageTransport it(this->nh);
        this->sub = it.subscribe("/realsense_plugin/camera/color/image_raw", 1, &CameraUAV::imageCallback, this); 
        
    }
    ~CameraUAV(){
        ;
    }


    // -------------------------------
    image_transport::Subscriber sub;

    void track_demo();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg); 

    char* get_state(int state);
};





#endif