#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  

#include <typeinfo>
#include<iostream>
using namespace std;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
  ROS_INFO("show image ");
  cv_bridge::CvImagePtr cv_ptr;  
  try  {  
     cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  
  }  catch (cv_bridge::Exception& e)  {  
     ROS_ERROR("cv_bridge exception: %s", e.what());  
     return;  
  }  

  cv::Mat img_rgb;  
  img_rgb = cv_ptr->image;  

  cout << typeid(img_rgb).name() << endl;

  cv::imshow("view", img_rgb);  
}  

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "image_subscriber");  
  ros::NodeHandle nh;  
  cv::namedWindow("view", CV_WINDOW_NORMAL);   
  cv::startWindowThread();  
  image_transport::ImageTransport it(nh);  
  image_transport::Subscriber sub = it.subscribe("/realsense_plugin/camera/color/image_raw", 1, imageCallback);

  ros::spin(); 

  cv::destroyWindow("view");  
  return 0;
} 
