#include "multi_object_.h"
#include "sstream"


multi_object_::multi_object_()
{
  ROS_INFO("Starting multi_object!");

  string camera_topic;
  //if (!nh_.getParam ("from_camera/camera_topic_name", camera_topic)) camera_topic = "/camera/image_rect_color";
  nh_.param("camera_topic_name",camera_topic,std::string("/camera/image_rect_color"));

  image_sub = nh_.subscribe(camera_topic.c_str(), 1, &multi_object_::image_callback,this);
  height_sub = nh_.subscribe("/px4flow/opt_flow", 10, &multi_object_::height_callback,this);
  pointer_pub = nh_.advertise<geometry_msgs::Point>("position_to_land",10);
  edge_pub = nh_.advertise<geometry_msgs::Point>("position_of_green_edge",10);

  ros::Rate loop_rate(10);

  namedWindow("image_rect_color_binary",CV_WINDOW_AUTOSIZE);   //WINDOW_NORMAL  CV_WINDOW_AUTOSIZE
  moveWindow("image_rect_color_binary",700,0);

  thresh_red = 70;
  thresh_green = 250;
  int FOCUS=200;
  m=70;//70
  n=150;
  int thre=3;

  while(ros::ok())
  {
    ros::spinOnce();
    if(!frame_raw.data)
    {
      loop_rate.sleep();
      ROS_INFO("Frame is empty!!!!");
      continue;
    }
    else
    {
       loop_rate.sleep();

       iRobot_identify(frame_raw);

       for(int i=0;i<contours.size();i++)
       {
           Point2f cen,cen1,cen2;
           float radius;

           cout<<"contours: "<<i<<" "<<contours[i].size()<<endl;
           minEnclosingCircle(Mat(contours[i]),cen,radius);

           Vec4f lin;
           fitLine(Mat(contours[i]),lin,CV_DIST_L2,0,0.01,0.01);
           cout << cen.x <<"  "<< cen.y <<"  "<<lin[2]<<"  "<<lin[3]<< endl;
           //cen.x=-40*lin[0]+lin[2];
           //cen.y=-40*lin[1]+lin[3];
           //cen1.x=40*lin[0]+lin[2];
           //cen1.y=40*lin[1]+lin[3];
           //line(binary,cen1,cen,Scalar(70),3);
           //cen.x=lin[2];
           //cen.y=lin[3];

           cen1.x=-40*lin[1]+lin[2];
           cen1.y=40*lin[0]+lin[3];
           cen2.x=40*lin[1]+lin[2];
           cen2.y=-40*lin[0]+lin[3];

           //LineIterator it1(binary,cen,cen1,8,false),it2(binary,cen,cen2,8,false);

           int result1=0,result2=0;

           for (int i1 = 0; i1 < 40; i1++)
           {
               if(int(binary.at<uchar>(int(i1*lin[0]+lin[3]),int(-1*i1*lin[1]+lin[2]))) > 0)result1++;
               //std::cout << int(-1*i1*lin[1]+lin[2])<<"  "<<int(i1*lin[0]+lin[3]) <<"  "<<int (binary.at<uchar>( int(i1*lin[0]+lin[3]),int(-1*i1*lin[1]+lin[2]) )) << std::endl;
               else break;
           }

           for (int i2 = 0; i2 < 40; i2++)
           {
               if(int(binary.at<uchar>(int(-1*i2*lin[0]+lin[3]),int(i2*lin[1]+lin[2]))) > 0)result2++;
               else break;
           }

           cout<<"result1:"<<result1<<"  "<<"result2:"<<result2<<endl;

           if((result1-result2)>thre)
           {
               line(binary,cen,cen1,Scalar(70),3);
               circle(binary,cen1,6,Scalar(70),-1);
           }

           if((result2-result1)>thre)
           {
               line(binary,cen,cen2,Scalar(70),3);
               circle(binary,cen2,6,Scalar(70),-1);
           }
       }
    }

	     imshow("image_rect_color_binary",binary);

         createTrackbar( " min:", "image_rect_color_binary", &m, 255, NULL );
         createTrackbar( " max:", "image_rect_color_binary", &n, 255, NULL );
         createTrackbar( " thre:", "image_rect_color_binary", &thre, 50, NULL );

         waitKey(1);
  }
}

multi_object_::~multi_object_()
{
  ROS_INFO("Destroying multi_object!");
  destroyAllWindows();
}

void multi_object_::iRobot_identify(Mat& frame_raw)
{
      cv::Mat_<cv::Vec3b>::iterator it = frame_raw.begin<cv::Vec3b>();   //begin iterator
      cv::Mat_<cv::Vec3b>::iterator itend = frame_raw.end<cv::Vec3b>();  //end iterator

      for(;it!=itend;++it)
      {
         if( ((*it)[1]>(0.5*((*it)[0]+(*it)[2])+thresh_green)) && ((*it)[1]>((*it)[0]+thresh_green-10)) && ((*it)[1]>((*it)[2]+thresh_green-10)) )
           {                                                            //BGR GGG  fornight 90 60 60  120 100 100
             (*it)[0]=255;//250
           }
          else if( (*it)[2]>(0.5*((*it)[0]+(*it)[1])+thresh_red) && ((*it)[2]>((*it)[0]+thresh_red-10)) && ((*it)[2]>((*it)[1]+thresh_red-10)) )
           {                                                            //BGR  RRR forsun 70 60 60 90 80 80
             (*it)[0]=255;                                              //sunday everychannel value is big
           }
         else
          {
             (*it)[0]=0;
          }
       }
      split(frame_raw,mv);
      //mv[0].copyTo(binary);
      binary.create(mv[0].size(), CV_8UC1 );
      binary = mv[0].clone();

      //Mat elements(5,5,CV_8U,Scalar(1));
      morphologyEx(binary,binary,MORPH_CLOSE,Mat());
      findContours(mv[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);   //find contours this will change mv[0]
      //findContours(mv[0], contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);   //find contours this will change mv[0]
      getSizeContours(contours);      //contours size filter
}

void multi_object_::height_callback(const px_comm::OpticalFlow::ConstPtr& msg)
{
      height = msg -> ground_distance;
}

void multi_object_::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  cv_ptr->image.copyTo(frame_raw);
}

void multi_object_::getSizeContours(vector< vector<Point> >& con)
{
    int cmin = m;   // contours minimum size
    int cmax = n;   // contours max size
    vector< vector<Point> >::iterator itc = con.begin();   //iterator begin
    while(itc != con.end())
    {
        if((itc->size()) < cmin || (itc->size()) > cmax)
        {
           itc = con.erase(itc);
        }
        else ++ itc;
    }
}
