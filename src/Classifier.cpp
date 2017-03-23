#include "lsd_line/Classifier.h"

#define Pi 3.1415926

Classifier::Classifier(void)
{
	show_info = false;
	horizontal_last_average_theta = 0;
	vertical_last_average_theta = Pi/2;
	vertical_average_theta = 0;
	horizontal_average_theta = 0;

	horizontal = vector<Line>();
	vertical = vector<Line>();
}


Classifier::~Classifier(void)
{
}

void Classifier::classify(vector<Line> line_fusion)
{
	vector<vector<Line> > groups;
	
	if(line_fusion.size()==0) 
	{
		if(horizontal.size()>0) horizontal.clear();
		if(vertical.size()>0) vertical.clear();
		return;
   	}
	for(int i=0;i<line_fusion.size();i++)
	{
		Line target = line_fusion[i];
		bool sorted = false;
		if(i==0)
		{
		    vector<Line> temp;
			temp.push_back(target);
			groups.push_back(temp);
		}
		else
		{
		for(int j=0; j< groups.size();j++)
		{
				vector<Line> group;
				group = groups[j];
				for(int k=0; k < group.size(); k ++)
				{
				
					Line sorted_line = group[k];
					if (abs(sorted_line.Get_theta() - target.Get_theta()) <= 0.3 || abs(abs(sorted_line.Get_theta() - target.Get_theta()) - Pi) < 0.3)
					{
							groups[j].push_back(target);
							sorted = true;
							break;
					}
				}
				if (sorted)
				break;
			}
			if (!sorted)
			{
				vector<Line> temp;
				temp.push_back(target);
				groups.push_back(temp);
			}
	  }
	}
	int max_group=0;
	int max2_group=0;
	int max_index=0;
	int max2_index=0;
	for(int i=0;i<groups.size();i++)
	{
		if(groups[i].size()>max_group)
		{
			max_group = groups[i].size();
			max_index = i;
		}
		else if(groups[i].size()>max2_group)
		{
			max2_group = groups[i].size();
			max2_index = i;
		}
	}

	if(groups.size()<2) 
	{
		vertical = vector<Line>();
		horizontal = vector<Line>();
		return;
	}
	vertical = groups[max_index];
	horizontal = groups[max2_index];


	double theta_average=0;

	for(int i=0; i<vertical.size();i++)
	{
		Line single_line = vertical[i];
		if (single_line.Get_theta() < -0.8)
		{
			theta_average += (single_line.Get_theta() + Pi);
		}
		else if(single_line.Get_theta() >= 2.8)
		{
			theta_average += (single_line.Get_theta() - Pi);
		}
		else
		{
			theta_average += single_line.Get_theta();
		}
	}
	vertical_average_theta = theta_average / vertical.size();

	 theta_average=0;

	 for(int i=0; i<horizontal.size();i++)
	{
		Line single_line = horizontal[i];
		if (single_line.Get_theta() < -0.8)
		{
			theta_average += (single_line.Get_theta() + Pi);
		}
		else if(single_line.Get_theta() >= 2.8)
		{
			theta_average += (single_line.Get_theta() - Pi);
		}
		else
		{
			theta_average += single_line.Get_theta();
		}
	}
	horizontal_average_theta = theta_average / horizontal.size();

	double delta_horizontal_theta =  MIN(abs(horizontal_last_average_theta - horizontal_average_theta),abs(horizontal_last_average_theta - horizontal_average_theta-Pi));
	double delta_cross_theta1 = MIN(abs(horizontal_last_average_theta - vertical_average_theta),abs(horizontal_last_average_theta - vertical_average_theta-Pi));
	double delta_cross_theta2 = MIN(abs(vertical_last_average_theta - horizontal_average_theta),abs(vertical_last_average_theta - horizontal_average_theta-Pi));
	double delta_vertical_theta =  MIN(abs(vertical_last_average_theta - vertical_average_theta),abs(vertical_last_average_theta - vertical_average_theta-Pi));

	if((delta_horizontal_theta > delta_cross_theta1) && (delta_vertical_theta > delta_cross_theta2))
	{
		double temp;
		vector<Line> temp_vector;
		temp_vector = horizontal;
		temp = horizontal_average_theta;
		horizontal = vertical;
		horizontal_average_theta = vertical_average_theta;
		vertical = temp_vector;
		vertical_average_theta = temp;
	}
	horizontal_last_average_theta = horizontal_average_theta;
	vertical_last_average_theta = vertical_average_theta;

	//arrange
	if(horizontal.size()>1)
	{
		for(int i=0; i<horizontal.size();i++)
		{
	   		for(int i=1; i<horizontal.size();i++)
			{
				if(horizontal[i].d<horizontal[i-1].d)
				{
					Line temp = horizontal[i];
					horizontal[i] = horizontal[i-1];
					horizontal[i-1] = temp;
				
				}
			}   
		}
	}

	if(vertical.size()>1)
	{
		for(int i=0; i<vertical.size();i++)
		{
	   		for(int i=1; i<vertical.size();i++)
			{
				if(vertical[i].d<vertical[i-1].d)
				{
					Line temp = vertical[i];
					vertical[i] = vertical[i-1];
					vertical[i-1] = temp;
				
				}
			}   
		}
	}


}

void Classifier::Draw2Image(IplImage* res_im)
{
	
	for (int i = 0; i < vertical.size(); i++)
	{
			vector<CvPoint> temp_points;
			temp_points = vertical[i].Get_boundary(cvPoint(0, 0), cvPoint(res_im->width, res_im->height));
			//sprintf(temp,"G:%d",i);
			if(temp_points.size()>=2)
			{			  
			  cvLine(res_im,temp_points[0],temp_points[1],CV_RGB(100,50,50*i),2,CV_AA);
			  if(show_info)
			  {
				  char temp[100];
				  CvFont font;
	              //cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1,8);
				  //sprintf(temp,"theta: %.2f ;d: %.2f; G:%d",vertical[i].theta,vertical[i].d,i);
				  sprintf(temp,"V:%d",i);
				  //cvPutText(res_im,temp,cvPoint((temp_points[0].x +temp_points[1].x)/2,(temp_points[0].y +temp_points[1].y)/2),&font,CV_RGB(0,0,0));
				  cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
				  //cvPutText(res_im,temp,cvPoint((temp_points[0].x +temp_points[1].x)/2,(temp_points[0].y +temp_points[1].y)/2+20*i),&font,CV_RGB(100,50,50*i));
			  }
			}
	}
	for (int i = 0; i < horizontal.size(); i++)
	{
			vector<CvPoint> temp_points;
			temp_points = horizontal[i].Get_boundary(cvPoint(0, 0), cvPoint(res_im->width, res_im->height));
			//sprintf(temp,"G:%d",i);
			if(temp_points.size()>=2)
			{			  
			  cvLine(res_im,temp_points[0],temp_points[1],CV_RGB(50*i,50,100),2,CV_AA);
			  if(show_info)
			  {
				  char temp[100];
				  CvFont font;
	              //cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1,8);
				  //sprintf(temp,"theta: %.2f ;d: %.2f; G:%d",horizontal[i].theta,horizontal[i].d,i);
				  sprintf(temp,"H:%d",i);
				  //cvPutText(res_im,temp,cvPoint((temp_points[0].x +temp_points[1].x)/2,(temp_points[0].y +temp_points[1].y)/2),&font,CV_RGB(0,0,0));
				  cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
				  //cvPutText(res_im,temp,cvPoint((temp_points[0].x +temp_points[1].x)/2+20*i,(temp_points[0].y +temp_points[1].y)/2),&font,CV_RGB(50*i,50,100));
		
			  }
			}
	}

	if(show_info)
	{
		char temp[100];
		char temp2[100];
		CvFont font;
		cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
		sprintf(temp,"Horizontal average theta: %.2f;",horizontal_average_theta);
		cvPutText(res_im,temp,cvPoint(20,40),&font,CV_RGB(0,0,0));
		sprintf(temp2,"Vertical average theta: %.2f;",vertical_average_theta);
		cvPutText(res_im,temp2,cvPoint(20,60),&font,CV_RGB(0,0,0));
	}
}

void Classifier::ShowInfo(bool show)
{
	show_info = show;
}

vector<Line> Classifier::GetHorizontalLines()
{
	return horizontal;
}

vector<Line> Classifier::GetVerticalLines()
{
	return vertical;
}
