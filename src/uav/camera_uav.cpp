#include "single_uav.h"
#include "camera_uav.h"
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
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>  


using namespace std;
enum
{
    WAITING,		//等待offboard模式
	CHECKING,		//检查飞机状态
	PREPARE,		//起飞到第一个点
	REST,			//休息一下
    DETECT,         //找圆
	TRACK,			//飞圆形路经
	FLYOVER,		//结束		
}FlyState = WAITING;//初始状态WAITING


void CameraUAV::track_demo(){
    ROS_INFO("%s --- Fly state: %s, current state: %s", this->uav_name, this->get_state(FlyState), string(this->current_state.mode).data());

    float desire_r = 3.0;
    float desire_z = 8.0;

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
            // 沿用sinle uav的逻辑，首先到达设定点
            this->target_pos_uav[0] = this->temp_pos_uav[0] - desire_r;
            this->target_pos_uav[1] = this->temp_pos_uav[1];
            this->target_pos_uav[2] = desire_z;

            this->send_setpoint(this->target_pos_uav, 0);
            this->MoveTimeCnt += 1;

            if(this->MoveTimeCnt > 100){
                FlyState = DETECT;
                this->MoveTimeCnt = 0;
            }
            if(this->current_state.mode != "OFFBOARD" ){
                FlyState = WAITING;
            }
        }
        break;

    case DETECT:
        {
            // 此状态用于发现圆环
            float phase = 3.1415926;						//起点在X负半轴
            float priod = 2000.0;
            float Omega = 2.0*3.14159 * this->MoveTimeCnt / priod;	//0~2pi
            

            this->MoveTimeCnt += 3;

            if(MoveTimeCnt >=priod*1000)							//走圆形周期
            {
                FlyState = FLYOVER;
            }
            if(this->detect_circle){
                FlyState = TRACK;
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
    case TRACK:
        {
            this->target_speed[0] =  max(min( this->FLY_STEP*(this->image_center.y-this->circle_center.y), this->FLY_STEP_MAX ), this->FLY_STEP_MIN);
            this->target_speed[1] =  max(min( this->FLY_STEP*(this->image_center.x-this->circle_center.x), this->FLY_STEP_MAX ), this->FLY_STEP_MIN);
            this->target_speed[2] = 0;

            ROS_INFO("--- current speed: x:%f y:%f z:%f", this->target_speed[0], this->target_speed[1], this->target_speed[2]);

            this->send_setspeed(this->target_speed);
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


void CameraUAV::imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
  bool detect_once = false;

  // 获取图像
  cv_bridge::CvImagePtr cv_ptr;  
  try  {  
     cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  
  }  catch (cv_bridge::Exception& e)  {  
     ROS_ERROR("cv_bridge exception: %s", e.what());  
     return;  
  }  
 
  // 生成rgb图像，灰色图像
  cv::Mat img_rgb, img_gray;  
  img_rgb = cv_ptr->image;  
  cv::cvtColor(img_rgb, img_gray, CV_BGR2GRAY);

  this->image_center.x = img_gray.cols / 2;
  this->image_center.y = img_gray.rows / 2;

  //
  GaussianBlur( img_gray, img_gray, cv::Size(9, 9), 2, 2);

  vector<cv::Vec3f> circles;
  HoughCircles(img_gray, circles, CV_HOUGH_GRADIENT, 1, img_gray.rows/8, 200, 40, 0, 0);

  for(size_t i = 0; i < circles.size(); i++){
      
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);

      cv::circle(img_rgb, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
      cv::circle(img_rgb, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
      
      if (!detect_once){ 
          this->circle_center.x = cvRound(circles[i][0]);
          this->circle_center.y = cvRound(circles[i][1]);
          this->detect_circle_times += 1;

          if (this->detect_circle_times == this->DETECT_CIRCLE_LIMIT){
            this->detect_circle = true;   
          }

          detect_once = true;
      
      }
      //ROS_INFO("circle center -- x: %d, y: %d", cvRound(circles[i][0]), cvRound(circles[i][1]));

  }
  
  // 显示图像
  cv::imshow("view", img_rgb);  
} 

char* CameraUAV::get_state(int state){
    switch (state)
    {
    case 0:
        return "WAITING";
    case 1:
        return "CHECKING";
    case 2:
        return "PREPARE";
    case 3:
        return "REST";
    case 4:
        return "DETECT";
    case 5:
        return "TRACK";
    case 6:
        return "FLYOVER";
    default:
        return "NONE";
    }
}



// 让小车动的脚本： rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0.3,0,0]' '[0,0,0.0]'

int main(int argc, char **argv){
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "camera_uav");
    ros::NodeHandle nh("~");
    // 频率 [20Hz]
    ros::Rate rate(20.0);

    CameraUAV uav("/uav0",nh);

    cv::namedWindow("view", CV_WINDOW_NORMAL);   
    cv::startWindowThread();  
    

    while(ros::ok())
    {   
        uav.track_demo();

 		ros::spinOnce();
        rate.sleep();
    }
    cv::destroyWindow("view"); 
    return 0;


}