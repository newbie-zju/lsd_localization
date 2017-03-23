#ifndef multi_object_H
#define multi_object_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "geometry_msgs/Point.h"
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <unistd.h>

#define THRE   0.5               //Threshold for track
#define TIMES  5                 //times for ahead

using namespace std;
using namespace cv;

class multi_object_
{
   public:
       multi_object_();
       virtual ~multi_object_();

   private:
       ros::NodeHandle  nh_;
       ros::Subscriber  image_sub,height_sub;
       ros::Publisher   pointer_pub,edge_pub;

       cv_bridge::CvImagePtr cv_ptr;
       geometry_msgs::Point  roi_msg,edge_msg;

       Mat frame_raw;
       Mat binary;
       vector<Mat> mv;
       vector< vector<Point> > contours;
	   vector<Point2f> rois,rois_last,rois_temp;
	   Point2f  roi;
	   float height;
       string num;

       int thresh_red;
       int thresh_green;
       int m,n;

       void getSizeContours( vector<vector<Point> >& con);
       void image_callback(const sensor_msgs::Image::ConstPtr& msg);
       void height_callback(const px_comm::OpticalFlow::ConstPtr& msg);
       void iRobot_identify(Mat& frame_raw);
};

#endif
