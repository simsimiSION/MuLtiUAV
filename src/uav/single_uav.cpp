#include "single_uav.h"
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


using namespace std;
enum
{
    WAITING,		//等待offboard模式
	CHECKING,		//检查飞机状态
	PREPARE,		//起飞到第一个点
	REST,			//休息一下
	FLY,			//飞圆形路经
	FLYOVER,		//结束		
}FlyState = WAITING;//初始状态WAITING


UAV::UAV(char* name, const ros::NodeHandle &nh_):uav_name(name), nh(nh_)
{

    ROS_INFO("===== UAV %s init =====", name);
    this->arm_sc = this->nh.serviceClient<mavros_msgs::CommandBool>(this->concat_str(this->uav_name, ARM_SERVER_ADDRESS));
    this->setpoint_raw_local_pub = this->nh.advertise<mavros_msgs::PositionTarget>(this->concat_str(this->uav_name, SETPOINT_RAW_LOCAL_ADDRESS) , 20);
    this->mode_sc = this->nh.serviceClient<mavros_msgs::SetMode>(this->concat_str(this->uav_name, MODE_ADDRESS));
    this->position_sub = this->nh.subscribe(this->concat_str(this->uav_name, POSITION_ADDRESS), 100, &UAV::pos_cb, this);
    this->mavros_state_sub = this->nh.subscribe(this->concat_str(this->uav_name, MAVROS_STATE_ADDRESS), 10, &UAV::state_cb, this);

}

UAV::~UAV(){
    ;
}


void UAV::send_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp){
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];

    pos_setpoint.yaw = yaw_sp;

    this->setpoint_raw_local_pub.publish(pos_setpoint);
}

void UAV::send_setspeed(const Eigen::Vector3d& pos_sp){
    mavros_msgs::PositionTarget pos_setpoint;

    pos_setpoint.type_mask = 0b110111000111;  // 100 111 111 000  xyz + yaw

    pos_setpoint.coordinate_frame = 1;

    pos_setpoint.velocity.x = pos_sp[0];
    pos_setpoint.velocity.y = pos_sp[1];
    pos_setpoint.velocity.z = pos_sp[2];


    this->setpoint_raw_local_pub.publish(pos_setpoint);
}



Eigen::Vector3d UAV::rec_localpoint(){
    // this->localponit_update = false;
    // while(ros::ok() && this->localponit_update){;}
    return this->pos_uav;

}

void UAV::initilize(){
    ;
    // 还没有弄明白为什么会失败，
    // 是因为还没有开启吗？
    // 是需要给一个延迟吗？

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // if(this->arm_sc.call(arm_cmd)){
    //     ROS_INFO("UAV initilize: set armming!!!");
    // }else{
    //     ROS_INFO("UAV initilize: set armming False!!!");
    // }
    

    // mavros_msgs::SetMode state_cmd;
    // state_cmd.request.custom_mode = "OFFBOARD";

    // if(this->mode_sc.call(state_cmd)){
    //     ROS_INFO("UAV initilize: set OFFBOARD!!!");
    // }else{
    //     ROS_INFO("UAV initilize: set OFFBOARD False!!!");
    // }
    

}

void UAV::pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
    Eigen::Vector3d pos_uav_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    this->pos_uav = pos_uav_fcu_enu;

    this->localponit_update = true;
    //ROS_INFO("current pos: x:%f y:%f z:%f", this->pos_uav[0], this->pos_uav[1], this->pos_uav[2]);
}

void UAV::state_cb(const mavros_msgs::State::ConstPtr& msg){
    this->current_state = *msg;
}


char* UAV::concat_str(char* prefix, const char* content)
{
    char str[100];
    strcpy(str, prefix);
    strcat(str, content);
    char *s = str;
    return s;
}


void UAV::circular_demo(float desire_r, float desire_z){
    ROS_INFO("%s --- Fly state: %d, current state: %s", this->uav_name, FlyState, string(this->current_state.mode).data());

    switch (FlyState)
    {
    case WAITING:
        {
            if(this->current_state.mode != "OFFBOARD"){
                this->target_pos_uav[0] = this->pos_uav[0];
                this->target_pos_uav[1] = this->pos_uav[1];
                this->target_pos_uav[2] = this->pos_uav[2];

                this->temp_pos_uav[0] = this->pos_uav[0];
                this->temp_pos_uav[1] = this->pos_uav[1];
                this->temp_pos_uav[2] = this->pos_uav[2];

                this->send_setpoint(this->target_pos_uav, 0);
            }else{
                this->target_pos_uav[0] = this->temp_pos_uav[0];
                this->target_pos_uav[1] = this->temp_pos_uav[1];
                this->target_pos_uav[2] = this->temp_pos_uav[2];

                this->send_setpoint(this->target_pos_uav, 0);
                FlyState = CHECKING;
            }
        }
        break;

    case CHECKING:
        {
            if(this->pos_uav[0] == 0 && this->pos_uav[1] == 0){
                this->mode_cmd.request.custom_mode = "AUTO.LAND";
                this->mode_sc.call(mode_cmd);
                FlyState = WAITING;	
            }else{
                FlyState = PREPARE;
                this->MoveTimeCnt = 0;
            }
        }
        break;

    case PREPARE:
        {
            this->temp_target_pos_uav[0] = this->temp_pos_uav[0] - desire_r;
            this->temp_target_pos_uav[1] = this->temp_pos_uav[1];
            this->temp_target_pos_uav[2] = desire_z;
            this->MoveTimeCnt += 2;
            
            if(this->MoveTimeCnt > 500){
                FlyState = REST;
                this->MoveTimeCnt = 0;
            }

            this->target_pos_uav[0] = this->temp_pos_uav[0] + (this->temp_target_pos_uav[0]-this->temp_pos_uav[0])*(this->MoveTimeCnt/500);
            this->target_pos_uav[1] = this->temp_pos_uav[1] + (this->temp_target_pos_uav[1]-this->temp_pos_uav[1])*(this->MoveTimeCnt/500);
            this->target_pos_uav[2] = this->temp_pos_uav[2] + (this->temp_target_pos_uav[2]-this->temp_pos_uav[2])*(this->MoveTimeCnt/500);

            this->send_setpoint(this->target_pos_uav, 0);

            if(this->current_state.mode != "OFFBOARD" ){
                FlyState = WAITING;
            }
        }
        break;

    case REST:
        {
            this->target_pos_uav[0] = this->temp_pos_uav[0] - desire_r;
            this->target_pos_uav[1] = this->temp_pos_uav[1];
            this->target_pos_uav[2] = desire_z;

            this->send_setpoint(this->target_pos_uav, 0);
            this->MoveTimeCnt += 1;

            if(this->MoveTimeCnt > 100){
                FlyState = FLY;
                this->MoveTimeCnt = 0;
            }
            if(this->current_state.mode != "OFFBOARD" ){
                FlyState = WAITING;
            }
        }
        break;

    case FLY:
        {
            float phase = 3.1415926;						//起点在X负半轴
            float priod = 2000.0;
            float Omega = 2.0*3.14159 * this->MoveTimeCnt / priod;	//0~2pi
            

            this->MoveTimeCnt += 3;

            if(MoveTimeCnt >=priod)							//走一个圆形周期
            {
                FlyState = FLYOVER;
            }

            this->target_pos_uav[0] = this->temp_pos_uav[0] + desire_r * cos(Omega+phase);
            this->target_pos_uav[1] = this->temp_pos_uav[1] + desire_r * sin(Omega+phase);
            this->target_pos_uav[2] = desire_z;

            ROS_INFO("%s --- current pos: x:%f y:%f z:%f", this->uav_name, this->pos_uav[0], this->pos_uav[1], this->pos_uav[2]);

            this->send_setpoint(this->target_pos_uav, 0);
            if(this->current_state.mode != "OFFBOARD" ){
                FlyState = WAITING;
            }
        }
        break;

    case FLYOVER:
        {
            this->mode_cmd.request.custom_mode = "AUTO.LAND";
            this->mode_sc.call(mode_cmd);
            //FlyState = WAITING;

            ROS_INFO("%s --- current pos: x:%f y:%f z:%f", this->uav_name, this->pos_uav[0], this->pos_uav[1], this->pos_uav[2]);
        }
        break;
    
    default:
        break;
    }

}


