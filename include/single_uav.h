#ifndef __SINGLE_UAV_H__
#define __SINGLE_UAV_H__

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

class UAV{
    public:
        /******  parmaeter  ******/
        mavros_msgs::State current_state;

        Eigen::Vector3d pos_uav;
        Eigen::Vector3d target_pos_uav;
        Eigen::Vector3d temp_pos_uav;
        Eigen::Vector3d temp_target_pos_uav;

        mavros_msgs::SetMode mode_cmd;

        float MoveTimeCnt = 0;
        bool localponit_update = false;

        /******  function  ******/
        UAV(char* name, const ros::NodeHandle &nh_);
        ~UAV();

        // 初始化
        void initilize();

        // 发送设定点
        void send_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp);
        void send_setspeed(const Eigen::Vector3d& pos_sp);

        // 接受当前点
        Eigen::Vector3d rec_localpoint();

        // demo
        void circular_demo(float desire_r, float desire_z);


    //private:
        /******  parmaeter  ******/
        const char* ARM_SERVER_ADDRESS = "/mavros/cmd/arming";
        const char* SETPOINT_RAW_LOCAL_ADDRESS = "/mavros/setpoint_raw/local";
        const char* MODE_ADDRESS = "/mavros/set_mode";
        const char* POSITION_ADDRESS = "/mavros/local_position/pose";
        const char* MAVROS_STATE_ADDRESS = "/mavros/state";
        
        char* uav_name;
        
        ros::NodeHandle nh;

        // 解锁设置
        ros::ServiceClient arm_sc;

        // 发布当前位置信息
        ros::Publisher setpoint_raw_local_pub;
        
        // 获取当前的模式
        ros::ServiceClient mode_sc;
        
        // 获取当前位置信息
        ros::Subscriber position_sub;

        // 获取mavros state
        ros::Subscriber mavros_state_sub;

        /******  function  ******/
        // 定义各种回调函数
        void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        // something
        char* concat_str(char* prefix, const char* content);


};



#endif