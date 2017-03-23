#include "lsd_line/LineGroup.h"


using namespace std;


LineGroup::LineGroup(void)
{
	//default parameters
	length_threshold = 35/2;
	group_size_threshold = 4;
	show_info = false;
}


LineGroup::~LineGroup(void)
{
}
//void LineGroup::GetRawLines_(vector<cv::Vec4i> detected_lines)
void LineGroup::GetRawLines(int dim, ntuple_list detected_lines)
{
    /***
    for (int i=0; i< detected_lines.size(); i++)
    {
        cv::Vec4d detected_lines_line = detected_lines[i];
        CvPoint start_pt = cvPoint((int)detected_lines_line[0],(int)detected_lines_line[1]);
        CvPoint end_pt = cvPoint((int)detected_lines_line[2],(int)detected_lines_line[3]);
        Line target = Line(start_pt, end_pt);
       bool sorted = false;

	   if(target.length <= length_threshold) continue;

 	   if (line_groups.size() == 0)
       {
		     line_groups.push_back(vector<Line>());
			 line_groups[0].push_back(target);
       }
       else
       {
		   for(int j=0; j< line_groups.size();j++)
           {
			   vector<Line> group = line_groups[j];
			   for(int k=0; k < group.size(); k ++)
               {
				   Line sorted_line = group[k];
                   if (abs(sorted_line.Get_theta() - target.Get_theta()) <= 0.12 || abs(abs(sorted_line.Get_theta() - target.Get_theta()) - Pi) < 0.12)
                   {
                      if (abs(sorted_line.Get_distance() - target.Get_distance()) <= 35/2)
                      {
						 line_groups[j].push_back(target);
                         sorted = true;
                         break;
                       }
                    }
                }
                if (sorted)
                break;
            }
            if (!sorted)
            {
				vector<Line> temp = vector<Line>();
				temp.push_back(target);
				line_groups.push_back(temp);
            }
         }
        }

    ****/
	for(int i=0; i< detected_lines->size; i++)
    {
	   CvPoint start_pt = cvPoint((int)detected_lines->values[i*dim+0],(int)detected_lines->values[i*dim+1]);
	   CvPoint end_pt = cvPoint((int)detected_lines->values[i*dim+2],(int)detected_lines->values[i*dim+3]);

       Line target = Line(start_pt, end_pt);
       bool sorted = false;

	   if(target.length <= length_threshold) continue;

 	   if (line_groups.size() == 0)
       {
		     line_groups.push_back(vector<Line>());
			 line_groups[0].push_back(target);
       }
       else
       {
		   for(int j=0; j< line_groups.size();j++)
           {
			   vector<Line> group = line_groups[j];
			   for(int k=0; k < group.size(); k ++)
               {
				   Line sorted_line = group[k];
                   if (abs(sorted_line.Get_theta() - target.Get_theta()) <= 0.12 || abs(abs(sorted_line.Get_theta() - target.Get_theta()) - Pi) < 0.12)
                   {
                      if (abs(sorted_line.Get_distance() - target.Get_distance()) <= 35/2)
                      {
						 line_groups[j].push_back(target);
                         sorted = true;
                         break;
                       }
                    }
                }
                if (sorted)
                break;
            }
            if (!sorted)
            {
				vector<Line> temp = vector<Line>();
				temp.push_back(target);
				line_groups.push_back(temp);
            }
         }
        }

}

void LineGroup::GetRawLines(vector<Vec4i> raw_points)
{
	for(int i=0; i< raw_points.size(); i++)
    {

	   Line target = Line(cvPoint(raw_points.at(i)[0],raw_points.at(i)[1]),cvPoint(raw_points.at(i)[2],raw_points.at(i)[3]));
       bool sorted = false;

	   if(target.length <= length_threshold) continue;

 	   if (line_groups.size() == 0)
       {
		     line_groups.push_back(vector<Line>());
			 line_groups[0].push_back(target);
       }
       else
       {
		   for(int j=0; j< line_groups.size();j++)
           {
			   vector<Line> group = line_groups[j];
			   for(int k=0; k < group.size(); k ++)
               {
				   Line sorted_line = group[k];
                   if (abs(sorted_line.Get_theta() - target.Get_theta()) <= 0.12 || abs(abs(sorted_line.Get_theta() - target.Get_theta()) - Pi) < 0.12)
                   {
                      if (abs(sorted_line.Get_distance() - target.Get_distance()) <= 35/2)
                      {
						 line_groups[j].push_back(target);
                         sorted = true;
                         break;
                       }
                    }
                }
                if (sorted)
                break;
            }
            if (!sorted)
            {
				vector<Line> temp = vector<Line>();
				temp.push_back(target);
				line_groups.push_back(temp);
            }
         }
        }
}


void LineGroup::Fusion_average()
{
	for(int i=0; i<line_groups.size();i++)
	{
		vector<Line> group;
		group = line_groups[i];

		double theta_average, d_average;
		double length_sum;
		theta_average = 0;
		d_average = 0;
		length_sum = 0;
		CvPoint neighbor;
		if(group.size()<=group_size_threshold)
		{
		   if(i==line_groups.size()-1)
			   break;
		   else
			   continue;
		}
		for(int j=1; j<group.size();j++)
		{
			Line single_line = group[j];
			if (single_line.Get_theta() < -0.8)
			{
				theta_average += (single_line.Get_theta() + Pi)*single_line.length;
			}
			else if(single_line.Get_theta() >= 2.8)
			{
				theta_average += (single_line.Get_theta() - Pi)*single_line.length;
			}
			else
			{
			    theta_average += single_line.Get_theta()*single_line.length;
			}

			d_average += single_line.Get_distance()*single_line.length;
			length_sum += single_line.length;
		}
		neighbor = group[0].p1();
		theta_average = theta_average / length_sum;
		d_average = d_average / length_sum;
		line_fusion.push_back(Line(theta_average, d_average, neighbor));
	}

 for(int i=0; i<line_groups.size();i++)
 {
	vector<Line> group;
	group = line_groups[i];
	double x_average=0;
	double y_average=0;

	CvPoint neighbor;
	if(group.size()<=group_size_threshold)
	{
		if(i==line_groups.size()-1)
			break;
		else
			continue;
	}
	for(int j=0; j<group.size();j++)
	{
		x_average+= group[j].p1().x;
		x_average+= group[j].p2().x;
		y_average+= group[j].p1().y;
		y_average+= group[j].p2().y;
	}
	x_average = x_average/(group.size()*2);
	y_average = y_average/(group.size()*2);
 }

}

void LineGroup::Draw2Image(IplImage* res_im)
{
	//int color_count = 0;


	for (int i = 0; i < line_fusion.size(); i++)
	{
			vector<CvPoint> temp_points;
			temp_points = line_fusion[i].Get_boundary(cvPoint(0, 0), cvPoint(res_im->width, res_im->height));
			//sprintf(temp,"G:%d",i);
			if(temp_points.size()>=2)
			{
			  cvLine(res_im,temp_points[0],temp_points[1],CV_RGB(0,0,0),2,CV_AA);
			  if(show_info)
			  {
				  char temp[100];
				  CvFont font;
	              cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1,8);
				  sprintf(temp,"theta: %.2f ;d: %.2f; G:%d",line_fusion[i].theta,line_fusion[i].d,i);
				  cvPutText(res_im,temp,cvPoint((temp_points[0].x +temp_points[1].x)/2,(temp_points[0].y +temp_points[1].y)/2),&font,CV_RGB(0,0,0));
			  }
			}
	}
}

void LineGroup::DrawRawLines2Image(IplImage* res_im)
{
	vector<CvScalar> colors;

	colors.push_back(CV_RGB(0,0,0));
	colors.push_back(CV_RGB(0,200,0));
	colors.push_back(CV_RGB(0,200,200));
	colors.push_back(CV_RGB(100,200,200));
	colors.push_back(CV_RGB(200,200,0));
	colors.push_back(CV_RGB(200,0,200));
    colors.push_back(CV_RGB(0,100,0));
	colors.push_back(CV_RGB(0,100,100));
	colors.push_back(CV_RGB(100,0,200));
	colors.push_back(CV_RGB(100,100,0));
	colors.push_back(CV_RGB(100,0,100));

	int color_count = 0;
	CvFont font;
	cvInitFont(&font,CV_FONT_VECTOR0,0.3,0.3,0,1,8);

	for (int i = 0; i < line_groups.size(); i++)
	{
		color_count++;
		if(color_count > 10)
			color_count =0;

		if(line_groups[i].size()>=group_size_threshold)
			for (int j = 0; j <line_groups[i].size(); j++)
			{

				//sprintf(temp,"G:%d",i);
				cvLine(res_im,line_groups[i][j].p1(),line_groups[i][j].p2(),colors[color_count],1,CV_AA);
				if(show_info)
				{
					char temp[40];
					sprintf(temp,"theta: %.2f ;d: %.2f; G:%d",line_groups[i][j].theta,line_groups[i][j].d,i);
					cvPutText(res_im,temp,cvPoint((line_groups[i][j].p1().x+line_groups[i][j].p2().x)/2,(line_groups[i][j].p1().y+line_groups[i][j].p2().y)/2),&font,colors[color_count]);
				}
			}
		else
		{
			for (int j = 0; j <line_groups[i].size(); j++)
			{
				cvLine(res_im,line_groups[i][j].p1(),line_groups[i][j].p2(),CvScalar(CV_RGB(0,0,0)),1,CV_AA);
			}
		}

	}

}

void LineGroup::ShowInfo(bool show)
{
	show_info = show;
}

vector<Line> LineGroup::GetLineFusion()
{
	return line_fusion;
}
