#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <list>

#include "lsd_line/Line.h"
#include "lsd_line/LineGroup.h"
#include "lsd_line/Classifier.h"
#include "lsd_line/Tracker.h"
#include "lsd_line/TinyRobots.h"

#include <vo_flow/OpticalFlow.h>

//#include "gl/gl.h"
#define GLUT_DISABLE_ATEXIT_HACK
//#include "gl/glut.h"

extern "C"
{
#include "lsd_line/lsd.h"
};
#define Pi 3.1415926
#define BYTE unsigned char

#define HOUGH_RHO 1
//double类型的rho，以像素为单位的直线累加平面所需要的分辨率。另一种形容方式是直线搜索时的进步尺寸的单位半径
#define HOUGH_THETA CV_PI/180
//double类型的theta，以弧度为单位的直线累加平面所需要的分辨率。另一种形容方式是直线搜索时的进步尺寸的单位角度
#define pi 3.1415926


using namespace std;
using namespace cv;
string iImgTopic;
ntuple_list detected_lines;
IplImage *ipl_img;
IplImage *m_im;
LineGroup grid_basic;
Classifier cl;
TRACKER tr;
TinyRobots robots;
int frame_count;
bool lose_judge = false;
Mat translation_coordinate;
vector<Point3d> coordinate_tracker_vector;
vector<Point3d> coordinate_tracker_vector_calc_hop_lose_;
vector<Vec3d> flow_temp_;
Vec3d flow_temp;
Point3d flow_path_temp;
Point2d flow_path_temp1(0,0);
vector<Point3d> flow_path_temp_;
vector<Point3d> translation_vector_calc_hop_lose_;
Point3d coordinate_tracker_hopped1,coordinate_tracker_hopped2;
double quality, dt;
double err_x;
double err_y;
int f1,f2,f3;
int f4 =0;
ros::Time flow_time_now,flow_time_last;
Point3d flow_path;
nav_msgs::Path flow_pose_path;
nav_msgs::Path path_msg;
nav_msgs::Path path;
geometry_msgs::PoseStamped temp;
geometry_msgs::PoseStamped pose_msgs;
ros::Publisher pose_pub;
ros::Publisher path_lsd_pub;
ros::Publisher path_pub;
ros::Publisher flow_pose_path_pub;
ros::Subscriber path_sub;
ros::Subscriber image_sub;
ros::Subscriber flow_sub;

int cannyLowThreshold = 50;

int houghThreshold = 160;
//int类型的threshold，累加平面的阈值参数，即识别某部分为图中的一条直线时它在累加平面中必须达到的值。
//大于阈值的线段才可以被检测通过并存到结果中。
int houghMinLineLength = 65.0;
//double类型的minLineLength，有默认值0，表示最低线段的长度，比这个设定参数短的线段就不能被显现出来
int houghMaxLineGap = 65.0;
//double类型的maxLineGap，有默认值0，允许将同一行点与点之间连接起来的最大的距离

int scale_int = 8;
int sigma_scale_int = 8;
int quant_int = 2;
int ang_th_int = 90;
int eps_int = 0;
int density_th_int = 8;
int n_bins = 2048;
int max_grad_int = 625;

void flowCallback(const vo_flow::OpticalFlowConstPtr& flow)
{
    //cout << "opticalflow:: " << endl;
    flow_temp[0] = flow -> ground_distance + 0.3;
    flow_temp[2] = flow -> velocity_x;
    flow_temp[1] = flow -> velocity_y;
    quality = flow -> quality;


    flow_temp_.push_back(flow_temp);
    //flow_time_temp_.push_back(flow_time);

    f1 = flow_temp_.size();
    //f2 = flow_time_temp_.size();

    //vector<Point3d>::iterator itr = flow_temp_.begin();
    if(f1 <= 1 || (flow_temp_[f1-1][1] == 0 && flow_temp_[f1-1][2] == 0))
    {
        //ROS_INFO("WARNING: NO flow_data!");
        flow_time_now = flow ->header.stamp;
        flow_time_last = flow_time_now;

    }
    else
    {
        flow_time_now = flow->header.stamp;
        dt = (double(flow_time_now.sec + 1e-9 * flow_time_now.nsec)) - (double(flow_time_last.sec + 1e-9 * flow_time_last.nsec));
        flow_path_temp.x = flow_temp_[f1-1][1] * dt;
        flow_path_temp.y = flow_temp_[f1-1][2] * dt;
        flow_path_temp.z = flow_temp_[f1-1][0];
        flow_time_last = flow_time_now;
        //cout << "flow_path_temp: " << flow_path_temp.x << " " <<flow_path_temp.y << endl;

        //flow_path_temp_.push_back(flow_path_temp);


        flow_path.x += flow_path_temp.x;
        flow_path.y += flow_path_temp.y;
        flow_path.z = flow_path_temp.z;

        //flow_path_.push_back(flow_path);
        cout << flow_path.x << " " << flow_path.y<<" " << flow_path.z << endl;
        geometry_msgs::PoseStamped temp2;
        temp2.pose.position.x = flow_path.x;
        temp2.pose.position.y = flow_path.y;
        temp2.pose.position.z = flow_path.z;

        //temp2.pose.orientation.x  = flow-> pose.orientation.x;
        //temp2.pose.orientation.y  = flow-> pose.orientation.y;
        //temp2.pose.orientation.z  = flow-> pose.orientation.z;
        //temp2.pose.orientation.w  = flow-> pose.orientation.w;
        temp2.header.stamp = flow-> header.stamp;
        flow_pose_path.poses.push_back(temp2);
        flow_pose_path.header.frame_id = "map";
        flow_pose_path.header.stamp = flow-> header.stamp;
        flow_pose_path_pub.publish(flow_pose_path);
        //cout<< temp2.pose.position.x <<" " <<temp2.pose.position.y<<" "<< temp2.pose.position.z << endl;
    }


}

/****
void pathCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

	geometry_msgs::PoseStamped temp;
	temp.pose.position.x = msg-> pose.position.x;
	temp.pose.position.y = msg-> pose.position.y;
	temp.pose.position.z = msg-> pose.position.z;

	cout <<temp.pose.position.x << " " <<temp.pose.position.y << " " <<temp.pose.position.z << endl;
	temp.pose.orientation.x  = msg-> pose.orientation.x;
	temp.pose.orientation.y  = msg-> pose.orientation.y;
	temp.pose.orientation.z  = msg-> pose.orientation.z;
	temp.pose.orientation.w  = msg-> pose.orientation.w;
	temp.header.stamp = msg-> header.stamp;
	//temp.header.frame_id = "temp";
	path.poses.push_back(temp);
	path.header.frame_id = "map";
	path.header.stamp = msg-> header.stamp;
	path_pub.publish(path);

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

/*****
    //----------------* houghlinesp *---------------------//
	cv::Mat dst, edge, gray;
	dst.create(cv_ptr->image.size(), cv_ptr->image.type());
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, edge, 5);
    //cv::Canny(edge, edge, cannyLowThreshold*2.5, cannyLowThreshold, 3);
     vector<cv::Vec4i> lines;
    cv::HoughLinesP(edge, lines, HOUGH_RHO, HOUGH_THETA, houghThreshold, houghMinLineLength, houghMaxLineGap);
	IplImage temp_img = IplImage(edge);
	ipl_img = &temp_img;
	m_im = ipl_img;

    if(ipl_img!=0)
	{
        frame_count++;

		if(frame_count == 1289)
		{
			int a =0;

			printf("%d",a);
		}


        IplImage* res_im = (IplImage*)cvClone(m_im);

        robots.FindCurrentCoordinates(res_im);

        grid_basic.ShowInfo(true);
        grid_basic.GetRawLines(lines);
        grid_basic.Fusion_average();

        //grid_basic.DrawRawLines2Image(res_im);
        //grid_basic.Draw2Image(res_im);
        cl.ShowInfo(true);
        cl.classify(grid_basic.line_fusion);
        cl.Draw2Image(res_im);

        Mat_<double>translation_coordinate(3,1);
        tr.ProcessFrame(cl.GetHorizontalLines(),cl.GetVerticalLines());
        geometry_msgs::Point coordinate;
        coordinate.x = tr.coordinate_tracker.x;
        coordinate.y = tr.coordinate_tracker.y;
        coordinate.z = tr.coordinate_tracker.z;
        coordinate_pub.publish(coordinate);

        geometry_msgs::PoseStamped coordinate_hopped;
        coordinate_hopped.header.frame_id = "map";
        coordinate_hopped.header.stamp = msg-> header.stamp;
        coordinate_hopped.pose.position.x = tr.coordinate_tracker_hopped.x;
        coordinate_hopped.pose.position.y = tr.coordinate_tracker_hopped.y;
        coordinate_hopped.pose.position.z = tr.coordinate_tracker_hopped.z;
        path_img.poses.push_back(coordinate_hopped);
        path_img.header.frame_id = "map";
        path_img.header.stamp = msg-> header.stamp;
        path_img_pub.publish(path_img);
        //coordinate_pub_hopped.publish(coordinate_hopped);


        tr.FindRealRobotPos(robots.GetImgPos());
        tr.Draw2Image(res_im);



        CvFont font2;
        cvInitFont(&font2,CV_FONT_VECTOR0,0.8,0.8,0,2,8);
        char temp[100];
        sprintf(temp,"%d",frame_count);
        cvPutText(res_im,temp,cvPoint(20,20),&font2,CV_RGB(200,100,50));

        cvShowImage("view",res_im);

        cvWaitKey(30);
        cvReleaseImage(&res_im);
****







	cv::Mat dst;
	cv::Mat dst_;
	dst.create(cv_ptr -> image.size(),cv_ptr -> image.type());
	//CV_BLUR_NO_SCALE,CV_BLUR,CV_GAUSSIAN,CV_MEDIAN,CV_BILATERAL
	//cvSmooth(cv_ptr->image,dst_,CV_GAUSSIAN,3,0,0.0,0.0);
	cv::GaussianBlur(cv_ptr->image, dst_, Size(9,9),0,0);
	cv_ptr->image.copyTo(dst,dst_);
	//方框滤波
    //cv::boxFilter(dst,dst,-1,Size(5,5));
    //高斯滤波
	//cv::GaussianBlur(dst,dst,Size(7,7),0,0);
	//cv_ptr->image.copyTo(dst,cv_ptr->image);
	IplImage temp_img = IplImage(dst);
	ipl_img = &temp_img;
	m_im = ipl_img;

	if(ipl_img!=0)
	{

   		frame_count++;

		if(frame_count == 289)
		{
			int a =0;

			printf("%d",a);
		}

		IplImage* im_gray = cvCreateImage(cvSize(m_im->width,m_im->height),IPL_DEPTH_8U,1);
		cvCvtColor(m_im,im_gray,CV_RGB2GRAY);
		image_double image = new_image_double(im_gray->width, im_gray->height);
		BYTE* im_src = (BYTE*)im_gray->imageData;
		int xsize = image->xsize;//宽度
		int ysize = image->ysize;//高度
		int y,x;
		LineGroup grid_basic;


		for (y = 0;y < ysize;y++)
		{
			for (x = 0;x < xsize;x++)  //x是横坐标，y是纵坐标
			{
				image->data[y*xsize + x] = im_src[y*im_gray->widthStep + x];//im_gray是灰度图像，没有颜色通道e
			}
		}
		cvReleaseImage(&im_gray);

		detected_lines =LineSegmentDetection(image, double(scale_int)/10, double(sigma_scale_int)/10, double(quant_int)/10, double(ang_th_int)/10, double(eps_int)/10,  double(density_th_int)/10, double (n_bins), double(max_grad_int)/10, NULL);
		free_image_double(image);

		int dim = detected_lines->dim;
		IplImage* res_im = (IplImage*)cvClone(m_im); //要在彩色图上画直线，对原图做拷贝
		//IplImage* res_im = cvCreateImage(cvSize(m_im->width,m_im->height),IPL_DEPTH_8U,3);

		robots.FindCurrentCoordinates(res_im);

		grid_basic.ShowInfo(true);
		grid_basic.GetRawLines(dim,detected_lines);
		grid_basic.Fusion_average();

		//grid_basic.DrawRawLines2Image(res_im);
		//grid_basic.Draw2Image(res_im);
		cl.ShowInfo(true);
		cl.classify(grid_basic.line_fusion);
		cl.Draw2Image(res_im);

		Mat_<double>translation_coordinate(3,1);
		tr.ProcessFrame(cl.GetHorizontalLines(),cl.GetVerticalLines());
		//coordinate_tracker_vector.push_back(coordinate_tracker);
        geometry_msgs::PoseStamped pose_msgs;
        pose_msgs.header.stamp = msg -> header.stamp;
        pose_msgs.header.frame_id = "map";

		path_msg.header.stamp = msg-> header.stamp;
		path_msg.header.frame_id = "map";
		//cout << "coordinate_tracker2: "<< tr.coordinate_tracker2<< endl;

        translation_vector_calc_hop_lose_.push_back(tr.coordinate_tracker_hopped);
        //cout << "translation_vector_calc_hop_lose_: " << translation_vector_calc_hop_lose_ << endl;
        f3 = translation_vector_calc_hop_lose_.size();
        vector<Point3d>::iterator itr = translation_vector_calc_hop_lose_.begin();

        if (f3 <= 1)
        {
            pose_msgs.pose.position.x = tr.coordinate_tracker_hopped.x;
            pose_msgs.pose.position.y = tr.coordinate_tracker_hopped.y;
            pose_msgs.pose.position.z = tr.coordinate_tracker_hopped.z;
            pose_pub.publish(pose_msgs);

            path_msg.poses.push_back(pose_msgs);
            path_lsd_pub.publish(path_msg);
        }
        else
        {
            err_x = translation_vector_calc_hop_lose_.at(f3-1).x-translation_vector_calc_hop_lose_.at(f3-2).x;
            err_y = translation_vector_calc_hop_lose_.at(f3-1).y-translation_vector_calc_hop_lose_.at(f3-2).y;
            coordinate_tracker_hopped1.x = tr.coordinate_tracker_hopped.x + flow_path_temp1.x;
            coordinate_tracker_hopped1.y = tr.coordinate_tracker_hopped.y + flow_path_temp1.y;
            if (err_x == 0 && err_y == 0)
            {
                //tr.coordinate_tracker_hopped.x +=flow_path_temp.x;
                //tr.coordinate_tracker_hopped.y +=flow_path_temp.y;
                coordinate_tracker_hopped1.x = coordinate_tracker_hopped1.x + flow_path_temp.x;
                coordinate_tracker_hopped1.y =coordinate_tracker_hopped1.y + flow_path_temp.y;
                flow_path_temp1.x += flow_path_temp.x;
                flow_path_temp1.y += flow_path_temp.y;
                //cout << "flow_path_temp1: " << flow_path_temp1 << endl;
                coordinate_tracker_hopped2.x = flow_path_temp1.x;
                coordinate_tracker_hopped2.y = flow_path_temp1.y;
                lose_judge = true;
                pose_msgs.pose.position.x = coordinate_tracker_hopped1.x;
                pose_msgs.pose.position.y = coordinate_tracker_hopped1.y;
                pose_msgs.pose.position.z = tr.coordinate_tracker_hopped.z;
                pose_pub.publish(pose_msgs);

                path_msg.poses.push_back(pose_msgs);
                path_lsd_pub.publish(path_msg);
            }
            else
            {
                pose_msgs.pose.position.x = coordinate_tracker_hopped1.x;
                pose_msgs.pose.position.y = coordinate_tracker_hopped1.y;
                pose_msgs.pose.position.z = tr.coordinate_tracker_hopped.z;
                pose_pub.publish(pose_msgs);

                path_msg.poses.push_back(pose_msgs);
                path_lsd_pub.publish(path_msg);
                //flow_path_temp1.x = 0.0;
                //flow_path_temp1.y = 0.0;
            }
            //pose_msgs.pose.position
            //cout<< pose_msgs.pose.position.x <<" " <<pose_msgs.pose.position.y <<" "<< pose_msgs.pose.position.z << endl;
            translation_vector_calc_hop_lose_.erase(itr);
        }

       /***
        geometry_msgs::Point coordinate_hopped;
        coordinate_hopped.x = tr.coordinate_tracker_hopped.x;
        coordinate_hopped.y = tr.coordinate_tracker_hopped.y;
        coordinate_hopped.z = tr.coordinate_tracker_hopped.z;
        coordinate_pub_hopped.publish(coordinate_hopped);
       ***

/*
		geometry_msgs::PoseStamped coordinate_hopped;
		coordinate_hopped.header.frame_id = "map";
		coordinate_hopped.header.stamp = msg-> header.stamp;
		coordinate_hopped.pose.position.x = tr.coordinate_tracker.y;
		coordinate_hopped.pose.position.y = tr.coordinate_tracker.x;
		coordinate_hopped.pose.position.z = 1.9;
		path_img.poses.push_back(coordinate_hopped);
		path_img.header.frame_id = "map";
		path_img.header.stamp = msg-> header.stamp;
		path_img_pub.publish(path_img);
*		//coordinate_pub_hopped.publish(coordinate_hopped);


		tr.FindRealRobotPos(robots.GetImgPos());
		tr.Draw2Image(res_im);



		CvFont font2;
		cvInitFont(&font2,CV_FONT_VECTOR0,0.8,0.8,0,2,8);
		char temp[100];
		sprintf(temp,"%d",frame_count);
		cvPutText(res_im,temp,cvPoint(20,20),&font2,CV_RGB(200,100,50));

		cvShowImage("view",res_im);

		cvWaitKey(10);
		cvReleaseImage(&res_im);
		cvReleaseImage(&im_gray);


	}

}

******/

int main(int argc, char *argv[])
{
	freopen("/home/exbot/catkin_ws_eclipse/src/lsd_line/test.txt", "w", stdout);
	ros::init(argc,argv,"lsd_line");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	//ros::Subscriber sub = nh.subscribe("true_pose", 1, pathCallback);
	ros::Subscriber flow_sub =nh.subscribe("/serial_flow_msg",1, flowCallback);

	flow_pose_path_pub = nh.advertise<nav_msgs::Path>("flow_pose_path",10);
	//pose_pub = nh.advertise<geometry_msgs::PoseStamped>("path_pose",10);
	//path_pub = nh.advertise<nav_msgs::Path>("path",10);
	//path_lsd_pub = nh.advertise<nav_msgs::Path>("path_img",10);
	//coordinate_pub = nh.advertise<geometry_msgs::Point>("coordinate",10);
	//coordinate_pub_hopped = nh.advertise<geometry_msgs::Point>("coordinate_hopped",10);
	nh_param.param("iImgTopic",iImgTopic,string("/usb_cam/image_rect"));
    frame_count=0;

	cv::namedWindow("view");
	cv::namedWindow("view1");
	cv::startWindowThread();
	//创建一个线程用来自动更新窗口,以及处理窗口触发事件。返回值为0,表示没有创建出线程
	cv::createTrackbar("scale:", "view1", &scale_int, 10);
	cv::createTrackbar("sigma_scale:", "view1", &sigma_scale_int, 10);
	cv::createTrackbar("quant:", "view1", &quant_int, 50);
	cv::createTrackbar("ang_th:", "view1", &ang_th_int, 500);
	cv::createTrackbar("eps:", "view1", &eps_int, 100);
	cv::createTrackbar("density_th:", "view1", &density_th_int, 10);
	cv::createTrackbar("n_bins:", "view1", &n_bins, 4096);
	cv::createTrackbar("max_grad: ", "view1", &max_grad_int, 2550);

	//image_transport::ImageTransport it(nh);
	//image_transport::Subscriber image_sub = it.subscribe(iImgTopic,1,imageCallback);
	ros::spin();
	//ros::Subscriber image_sub = nh.subscribe(camera_topic.c_str(),1,)
	cv::destroyWindow("view");
	cv::destroyWindow("view1");


	return 0;
}
