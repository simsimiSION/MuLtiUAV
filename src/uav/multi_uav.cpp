#include "single_uav.h"
#include "multi_uav.h"
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <typeinfo>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.1415927

using namespace std;


MultiTask::MultiTask(const ros::NodeHandle &nh_):task_nh(nh_)
{
    ROS_INFO("===== MultiTask init =====");

    this->center  = new UAV("/uav0", this->task_nh);
    this->convoy1 = new UAV("/uav1", this->task_nh);
    this->convoy2 = new UAV("/uav2", this->task_nh);
    this->convoy3 = new UAV("/uav3", this->task_nh);
}

MultiTask::~MultiTask()
{
    ;
}


void MultiTask::setpoint(Eigen::Vector3d point){
    this->target_point = point;

    this->center->send_setpoint(point, 0);
    this->setpoint_convoy1(point);
    this->setpoint_convoy2(point);
    this->setpoint_convoy3(point);

    this->convoy_angle_update();
}

void MultiTask::setpoint_convoy1(Eigen::Vector3d point){
    Eigen::Vector3d temp_point;

    temp_point[0] = point[0] + this->CONVOY_DISTANCE * cos(this->set_target_angle(0.0)/180*PI);
    temp_point[1] = point[1] + this->CONVOY_DISTANCE * sin(this->set_target_angle(0.0)/180*PI);
    temp_point[2] = point[2];

    this->convoy1->send_setpoint(temp_point, 0);
}

void MultiTask::setpoint_convoy2(Eigen::Vector3d point){
    Eigen::Vector3d temp_point;

    temp_point[0] = point[0] + this->CONVOY_DISTANCE * cos(this->set_target_angle(120.0)/180*PI);
    temp_point[1] = point[1] + this->CONVOY_DISTANCE * sin(this->set_target_angle(120.0)/180*PI);
    temp_point[2] = point[2];

    this->convoy2->send_setpoint(temp_point, 0);
}

void MultiTask::setpoint_convoy3(Eigen::Vector3d point){
    Eigen::Vector3d temp_point;

    temp_point[0] = point[0] + this->CONVOY_DISTANCE * cos(this->set_target_angle(-120.0)/180*PI);
    temp_point[1] = point[1] + this->CONVOY_DISTANCE * sin(this->set_target_angle(-120.0)/180*PI);
    temp_point[2] = point[2];

    this->convoy3->send_setpoint(temp_point, 0);
}

float MultiTask::set_target_angle(float base_angle){
    float angle = this->convoy_angle + base_angle;

    if (angle > 360.0){
        angle -= 360.0;
    }
    if (angle < 0.0){
        angle += 360.0;
    }
    return angle;
}

void MultiTask::convoy_angle_update(){
    this->convoy_angle += this->CONVOY_ANGLE_INCREMENT;
    if (this->convoy_angle > 360.0){
        this->convoy_angle = 0.0;
    }

}

void MultiTask::task1(){
    this->CONVOY_DISTANCE = 5.0;
    this->CONVOY_ANGLE_INCREMENT = 1.8;

    Eigen::Vector3d temp_point;
    temp_point[0] = 0.0;
    temp_point[1] = 0.0;
    temp_point[2] = 5.0;

    this->setpoint(temp_point);

}

int main(int argc, char **argv){
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "single_uav");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    MultiTask multi_task(nh);

    while(ros::ok())
    {   
        multi_task.task1();

 		ros::spinOnce();
        rate.sleep();
    }

    return 0;


}


