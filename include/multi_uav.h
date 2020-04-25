#ifndef __MULTI_UAV_H__
#define __MULTI_UAV_H__

#include "single_uav.h"
#include <ros/ros.h>
#include <iostream>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

class MultiTask{
    public:
        UAV *center;
        UAV *convoy1;
        UAV *convoy2;
        UAV *convoy3;

        float CONVOY_DISTANCE = 5.0;
        float CONVOY_ANGLE_INCREMENT = 1.8;

        float convoy_angle = 0.0;

        Eigen::Vector3d target_point;


        MultiTask(const ros::NodeHandle &nh_);
        ~MultiTask();

        void setpoint(Eigen::Vector3d point);

        // 3个小飞机绕主机飞行
        void task1();


    //private:
        ros::NodeHandle task_nh;

        void setpoint_convoy1(Eigen::Vector3d point);
        void setpoint_convoy2(Eigen::Vector3d point);
        void setpoint_convoy3(Eigen::Vector3d point);

        void convoy_angle_update();
        float set_target_angle(float base_angle);


};

#endif