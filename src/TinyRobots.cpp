#include "lsd_line/TinyRobots.h"


TinyRobots::TinyRobots(void)
{
}


TinyRobots::~TinyRobots(void)
{
}

void TinyRobots::FindCurrentCoordinates(IplImage *res_im)
{
	 
	 IplImage* dst = cvCreateImage(cvGetSize(res_im),IPL_DEPTH_8U,1);

	 for(int i=0;i<dst->height;i++)
	 {
		  for(int j=0;j<dst->width;j++)
		  {
			  int blue = CV_IMAGE_ELEM(res_im, uchar, i, 3*j);
			  int red = CV_IMAGE_ELEM(res_im, uchar, i, 3*j+2);
			  if( (1.0*red / blue) <3.5 )
			  {
		          CV_IMAGE_ELEM(dst, uchar, i, j) =  0;
			  }
			  else
			  {
			      CV_IMAGE_ELEM(dst, uchar, i, j) = 255;
			  }
		  }
	 }
	
	 
	 cvErode(dst,dst,0,4);
	 cvDilate(dst,dst,0,6);

	 CvMemStorage* storage = cvCreateMemStorage(0);    
     CvSeq* contour = 0;    
	 CvFont font;
	 cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1.8,8);


	 cvFindContours( dst, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE  );
	 
	 
	 int count=0;

	 vector<Point2d> coordinates;

	 current.clear();
	 
	 for(int i=0;i<last.size();i++)
	 {
		 last.at(i).lost();   
	 }

	 for( ; contour != 0; contour = contour->h_next )    
     {    
          count++;
          double tmparea=fabs(cvContourArea(contour));
		  CvRect area_Rect = cvBoundingRect(contour,0);  
		  int ret = FindRobot(Point2d(area_Rect.x+area_Rect.width/2,(int)area_Rect.y+area_Rect.height/2));
		  if (ret == -1)
		  {
			  current.push_back(robot_unit(Point2i((int)area_Rect.x+area_Rect.width/2,(int)area_Rect.y+area_Rect.height/2),last));
		  }
		  else
		  {
			  for(int i=0; i<last.size();i++)
			  {
				  if(ret ==last.at(i).id)
				  {
					  last.at(i).Update(Point2i((int)area_Rect.x+area_Rect.width/2,(int)area_Rect.y+area_Rect.height/2));
					  current.push_back(last.at(i));
				  }
			  }

		  }
     }   
	 for(int i=0;i<last.size();i++)
	 {
		 if(!last.at(i).active)
		 {
			 last.at(i).life--;
			 if(last.at(i).life>=0)
			 {
				 current.push_back(last.at(i));
			 }
		 }
	 
	 }

     
	 //for(int i=0; i<current.size();i++)
	 //{
		// if(current.at(i).active)
		// {
	 //     char temp[100];	
		//  cvCircle(res_im,CvPoint(current.at(i).img_pos.x,current.at(i).img_pos.y),1,CvScalar(CV_RGB(245,175,0)),CV_AA,1);
		//  sprintf(temp,"X: %.2d ; Y: %.2d; ID: %d",current.at(i).img_pos.x ,current.at(i).img_pos.y,current.at(i).id);
	 //     cvPutText(res_im,temp,CvPoint(current.at(i).img_pos.x +10,current.at(i).img_pos.y+10),&font,CvScalar(CV_RGB(255,0,0)));	 
		// }
	 //}
     

	 cvReleaseImage(&dst);
	 cvReleaseMemStorage(&storage);
	 //cvReleaseMemStorage(&contour->storage);

	 last = current;


}

int TinyRobots::FindRobot(Point2d pos)
{
	double min_distance = 500;
	int id = -1;
	int count = 0;
	for(int i=0; i<last.size();i++)
	{
		double temp_d = last.at(i).FromImgPoint(pos);
		if (temp_d < min_distance)
		{
			min_distance  = last.at(i).FromImgPoint(pos);
			count = i;
		}
	}
	if((min_distance >40))
	{
	    return -1;
	}
	else
	{
		last.at(count).Found();
		return last.at(count).id;
	}
}

vector<robot_unit>  TinyRobots::GetImgPos()
{
	return current;
}
