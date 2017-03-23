#ifndef __TRACKER_H__
#define __TRACKER_H__

#include <vector>
#include "lsd_line/GridLine.h"
#include "lsd_line/square.h"
#include "lsd_line/d_unit.h"
#include "lsd_line/robot_unit.h"
#include "lsd_line/Line.h"
#include "lsd_line/LineGroup.h"
#include "lsd_line/Classifier.h"
#include "lsd_line/TinyRobots.h"
#include "lsd_line/Line.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <std_msgs/Bool.h>

//#include "lsd_line/sparse_flow.h"



#define GLUT_DISABLE_ATEXIT_HACK
#define HORIZONTAL 0
#define VERTICAL 1

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

using namespace cv;
using namespace std;

class TRACKER
{
public:


    ntuple_list detected_lines;
    IplImage *ipl_img;
    IplImage *m_im;
    LineGroup grid_basic;
    Classifier cl;
    TinyRobots robots;
    int frame_count;

    int scale_int;
    int sigma_scale_int;
    int quant_int;
    int ang_th_int;
    int eps_int;
    int density_th_int;
    int n_bins;
    int max_grad_int;

	int l;
	double error_x;
	double error_y;
	double d_upper_threshold;
	double d_lower_threshold;
	double min_horizontal_d;
	double min_vertical_d;
	bool initialized;
	vector<int> frame_offset_h;
	vector<int> frame_offset_v;
	vector<square> squares;
	Mat_<double> rotation_vector;
    	Mat_<double> translation_vector;
	Mat_<double> translation_vector_calc;
	//Mat_<double> translation_coordinate;
	vector<Mat_<double> > rotation_vectors;
	vector<Mat_<double> > translation_vectors;
	vector<Mat_<double> > translation_coordinate_;
	vector <Point3d>  translation_vector_calc_;
	vector <Point3d>  translation_vector_calc_hop_;
	vector<vector <Point2d> > img_point_sets;
	vector<vector <Point3d> > obj_point_sets;
	vector<vector <Point2d> > img_point_square_sets;
	vector<vector <Point3d> > obj_point_square_sets;

	vector <Point2d> imagePoints0;
	vector <Point2d> imagePoints_origin;
	vector <Point2d> ProjectPosition;
	vector <Point3d> ProjectPosition_origin;

	vector<robot_unit> robot_sets;
	Point3d coordinate_tracker;
	Point3d coordinate_tracker2;
	Point3d coordinate_tracker_hopped;
	//ros::Publisher path_lsd_pub;
	geometry_msgs::Point coordinate;





public:
	TRACKER(void);
	~TRACKER(void);
	void pathCallback(const geometry_msgs::PoseStampedConstPtr& msg);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	vector<GridLine> vertical_last;
	vector<GridLine> horizontal_last;
	vector<GridLine> vertical;
	vector<GridLine> horizontal;

	int frame_offset_horizontal;
	int frame_offset_vertical;

public:
	bool Initialize(vector<Line> horizontal, vector<Line> vertical);
	void LoadGrids(vector<Line> horizontal_raw, vector<Line> vertical_raw);
	void Draw2Image(IplImage* res_im);
	bool IsInitialized();
	CvPoint2D64f FindLineCross(Line line1,Line line2);
	//Mat_<double> ProcessFrame(vector<Line> horizontal_raw, vector<Line> vertical_raw);
	void ProcessFrame(vector<Line> horizontal_raw, vector<Line> vertical_raw);
	bool FindOffSet();
	int FindOffsetMode(vector<int> offset);
	void FindRealRobotPos(vector<robot_unit> robots);
    //void pathCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    //void imageCallback(const sensor_msgs::Image::ConstPtr msg);
};
#endif
