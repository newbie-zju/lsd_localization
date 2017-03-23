#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

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

//#include "gl/gl.h"
#define GLUT_DISABLE_ATEXIT_HACK  
//#include "gl/glut.h"

extern "C"
{
#include "lsd_line/lsd.h"
};
#define Pi 3.1415926
#define BYTE unsigned char


using namespace std;
using namespace cv;
string iImgTopic;
ntuple_list detected_lines;
IplImage *ipl_img;
IplImage *m_im;
Classifier cl;
Tracker tr;
TinyRobots robots;
int frame_count;
ros::Publisher coordinate_pub;
ros::Publisher coordinate_pub_hopped;
Mat translation_coordinate;


int scale_int = 8;       
int sigma_scale_int = 8;                            
int quant_int = 20;                                                 
int ang_th_int = 225;         
int eps_int = 0;        
int density_th_int = 7; 
int n_bins = 1024;                                      
int max_grad_int = 2550;
/*
 IplImage * doPryDown(IplImage * in,int filter=CV_GAUSSIAN_5x5)
 {
      //assert(in->width%2==0 && in->height%2==0);
      IplImage * out=cvCreateImage(cvSize(in->width/2,in->height/2),in->depth,in->nChannels);
      cvPyrDown(in,out,CV_GAUSSIAN_5x5);//filter=7 目前只支持CV_GAUSSIAN_5x5
      return (out);
 }
*/

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
	cv::Mat edge, gray;
	cv::Mat dst;
	dst.create(cv_ptr -> image.size(),cv_ptr -> image.type());
	cv_ptr->image.copyTo(dst,cv_ptr->image);
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
		//detected_lines = lsd(image);//detected_lines中存储提取直线的首位坐标及宽度，具体意义见说明文档
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
		geometry_msgs::Point coordinate;
		coordinate.x = tr.coordinate_tracker.x;
		coordinate.y = tr.coordinate_tracker.y;
		coordinate.z = tr.coordinate_tracker.z;
		coordinate_pub.publish(coordinate);
		
		geometry_msgs::Point coordinate_hopped;
		coordinate_hopped.x = tr.coordinate_tracker_hopped.x;
		coordinate_hopped.y = tr.coordinate_tracker_hopped.y;
		coordinate_hopped.z = tr.coordinate_tracker_hopped.z;
		coordinate_pub_hopped.publish(coordinate_hopped);		


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

 

int main(int argc, char *argv[])
{
	freopen("/home/exbot/catkin_ws_eclipse/src/lsd_line/out_world.txt", "w", stdout) ;
	ros::init(argc,argv,"lsd_line");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	coordinate_pub = nh.advertise<geometry_msgs::Point>("coordinate",10);
	coordinate_pub_hopped = nh.advertise<geometry_msgs::Point>("coordinate_hopped",10);
	nh_param.param("iImgTopic",iImgTopic,string("camera/image_rect_color"));
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
	cv::createTrackbar("n_bins:", "view1", &n_bins, 2048);
	cv::createTrackbar("max_grad: ", "view1", &max_grad_int, 2550);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe(iImgTopic,1,imageCallback);
	ros::spin();	
	//ros::Subscriber image_sub = nh.subscribe(camera_topic.c_str(),1,)
	cv::destroyWindow("view");
	cv::destroyWindow("view1");

	return 0;
}
