#include "lsd_line/robot_unit.h"
#include "lsd_line/Tracker.h"
#include "lsd_line/Matrix.h"
using namespace std;

CvPoint2D64f FindLineCross(Line line1,Line line2)
{
	CvPoint2D64f point;

	vector<double> Arow1,Arow2,Crow1,Crow2;

	Arow1.push_back(line1.A);
	Arow1.push_back(line1.B);
	Arow2.push_back(line2.A);
	Arow2.push_back(line2.B);
	Crow1.push_back(-line1.C);
	Crow2.push_back(-line2.C);

	Matrix A,B,C;
	A.push_back(Arow1);
	A.push_back(Arow2);
	C.push_back(Crow1);
	C.push_back(Crow2);

	B = inverse(A)*C; 

	point.x = B[0][0];
	point.y = B[1][0];

	return point;
}

Tracker::Tracker(void)
{
	initialized = false;
	frame_offset_vertical = 0;
	frame_offset_horizontal = 0;


	translation_vector = Mat_<double> (3,1); 
    rotation_vector = Mat_<double>(3,1); 
	translation_vector = 0; 
    rotation_vector = 0; 

}

Tracker::~Tracker(void)
{

}


bool Tracker::IsInitialized()
{
	return initialized;
}

bool Tracker::Initialize(vector<Line> horizontal_raw, vector<Line> vertical_raw)
{


	LoadGrids(horizontal_raw,vertical_raw);

	if(FindOffSet())
	{
		horizontal_last = horizontal;
		vertical_last = vertical;
		initialized = true;
        return true;
	}
	else
	{
	    return false;
	}
}

void Tracker::LoadGrids(vector<Line> horizontal_raw, vector<Line> vertical_raw)
{
	    horizontal.clear();
		vertical.clear();
		for(int i=0; i<horizontal_raw.size(); i++)
		{
			horizontal.push_back(GridLine(horizontal_raw.at(i)));
		}
		for(int i=0; i<horizontal.size(); i++)
		{
			if(i-1>=0)
			{
				horizontal.at(i).toPre(horizontal.at(i-1),HORIZONTAL);
			}
			else 
			{
				horizontal.at(i).toPreNull();
			}

			if(i+1<horizontal.size())
			{
				horizontal.at(i).toPost(horizontal.at(i+1),HORIZONTAL);
			}
			else
			{
				horizontal.at(i).toPostNull();
			}
		}

		for(int i=0; i<vertical_raw.size(); i++)
		{
			vertical.push_back(GridLine(vertical_raw.at(i)));
		}
		for(int i=0; i<vertical.size(); i++)
		{
			if(i-1>0)
			{
				vertical.at(i).toPre(vertical.at(i-1),VERTICAL);
			}
			else 
			{
				vertical.at(i).toPreNull();
			}

			if(i+1<vertical.size())
			{
				vertical.at(i).toPost(vertical.at(i+1),VERTICAL);
			}
			else
			{
				vertical.at(i).toPostNull();
			}
		}


}

void Tracker::ProcessFrame(vector<Line> horizontal_raw, vector<Line> vertical_raw)
{
	if(!initialized)
	{
		Initialize( horizontal_raw, vertical_raw);
		return;
	}

	LoadGrids(horizontal_raw, vertical_raw);
    
	if(!FindOffSet())
	{
	   frame_offset_h.clear();
	   frame_offset_v.clear();
	   squares.clear();
	   printf("Reinitialization required");
	   initialized = false;
	   return;
	}

	vector<vector<GridLine> > neibour;
	for(int i=0;i<horizontal.size();i++)
	{
		GridLine target = horizontal.at(i);
	    vector<GridLine> current_arrange;
		current_arrange = horizontal_last;
		for(int j=0; j<current_arrange.size();j++)
		{
			for(int k=0; k<(current_arrange.size()-1);k++)
			{
				double d_1 = current_arrange.at(k).toGridLine(target,HORIZONTAL);
				double d_2 = current_arrange.at(k+1).toGridLine(target,HORIZONTAL);
				if( d_1 > d_2)
				{
					swap(current_arrange.at(k),current_arrange.at(k+1));
				}
			}
		}   
		neibour.push_back(current_arrange);
	}

	frame_offset_h.clear();
	
	for(int i=0; i<neibour.size();i++)
	{
		if(abs(horizontal.at(i).toGridLine(neibour.at(i).at(0),HORIZONTAL))<60)
			frame_offset_h.push_back(horizontal.at(i).world_lable - neibour.at(i).at(0).world_lable);
	}
	frame_offset_horizontal = FindOffsetMode(frame_offset_h);

	neibour.clear();
	for(int i=0;i<vertical.size();i++)
	{
		GridLine target = vertical.at(i);
	    vector<GridLine> current_arrange;
		current_arrange = vertical_last;
		for(int j=0; j<current_arrange.size();j++)
		{
			for(int k=0; k<(current_arrange.size()-1);k++)
			{
				double d_1 = current_arrange.at(k).toGridLine(target,VERTICAL);
				double d_2 = current_arrange.at(k+1).toGridLine(target,VERTICAL);
				if( d_1 > d_2)
				{
					swap(current_arrange.at(k),current_arrange.at(k+1));
				}
			}
		}   
		neibour.push_back(current_arrange);
	}

	frame_offset_v.clear();
	
	for(int i=0; i<neibour.size();i++)
	{
		if(abs(vertical.at(i).toGridLine(neibour.at(i).at(0),VERTICAL))<60)
			frame_offset_v.push_back(vertical.at(i).world_lable - neibour.at(i).at(0).world_lable);
	}

	frame_offset_vertical = FindOffsetMode(frame_offset_v);

	for(int i=0; i<horizontal.size();i++)
	{
		horizontal.at(i).world_lable -= frame_offset_horizontal;
	}
	for(int i=0; i<vertical.size();i++)
	{
		vertical.at(i).world_lable -= frame_offset_vertical;
	}


	for(int i=0; i<horizontal.size();i++)
	{
		horizontal.at(i).stability = 0.5;
	}
	vector<d_unit> shift_h;
	for(int i=0; i<horizontal.size();i++)
	{
		for(int j=0;j<horizontal_last.size();j++)
		{
			if(horizontal.at(i).world_lable == horizontal_last.at(j).world_lable)
			{
				horizontal.at(i).stability+=0.4;
				shift_h.push_back(d_unit(i,horizontal.at(i).toGridLine(horizontal_last.at(j),HORIZONTAL)));
			}
		}
	}
	double average_shift=0;
	for(int i=0;i<shift_h.size();i++)
	{
		average_shift += abs(shift_h.at(i).d);
	}
	average_shift = average_shift/shift_h.size();
	for(int i=0;i<shift_h.size();i++)
	{
		double shift_rate = abs((shift_h.at(i).d - average_shift) / (10*average_shift)); 
		horizontal.at(shift_h.at(i).num).stability = (horizontal.at(shift_h.at(i).num).stability - shift_rate)*horizontal.at(shift_h.at(i).num).confidence_level;
	}


	for(int i=0; i<vertical.size();i++)
	{
		vertical.at(i).stability = 0.5;
	}
	vector<d_unit> shift_v;
	for(int i=0; i<vertical.size();i++)
	{
		for(int j=0;j<vertical_last.size();j++)
		{
			if(vertical.at(i).world_lable == vertical_last.at(j).world_lable)
			{
				vertical.at(i).stability+=0.4;
				shift_v.push_back(d_unit(i,vertical.at(i).toGridLine(vertical_last.at(j),HORIZONTAL)));
			}
		}
	}
	
	average_shift=0;
	for(int i=0;i<shift_v.size();i++)
	{
		average_shift += abs(shift_v.at(i).d);
	}
	average_shift = average_shift/shift_v.size();
	for(int i=0;i<shift_v.size();i++)
	{
		double shift_rate = abs((shift_v.at(i).d - average_shift) / (10*average_shift)); 
		vertical.at(shift_v.at(i).num).stability = (vertical.at(shift_v.at(i).num).stability - shift_rate)*vertical.at(shift_v.at(i).num).confidence_level;
	}

	squares.clear();
	int square_count=0;

	for(int i=0;i<(horizontal.size()-1);i++)
	{
		for(int j=(i+1); j<horizontal.size();j++)
		{
			int grid_distance_h;
			grid_distance_h = horizontal.at(i).world_lable - horizontal.at(j).world_lable;
			
			for(int m =0; m<(vertical.size()-1);m++)
			{
				for(int n=(m+1); n<vertical.size();n++)
				{ 
					int grid_distance_v;
					grid_distance_v = vertical.at(m).world_lable - vertical.at(n).world_lable;
					if(grid_distance_h == grid_distance_v)
					{
						double min_reliability;
						square_count++;
						CvPoint2D64f p1,p2,p3,p4;
						p1 = FindLineCross(horizontal.at(i),vertical.at(m));
						p2 = FindLineCross(horizontal.at(i),vertical.at(n));
						p3 = FindLineCross(horizontal.at(j),vertical.at(n));
						p4 = FindLineCross(horizontal.at(j),vertical.at(m));
						min_reliability = min(min(horizontal.at(i).stability,horizontal.at(j).stability),min(vertical.at(m).stability,vertical.at(n).stability));
						squares.push_back(square(p1,p2,p3,p4,square_count,min_reliability,vertical.at(m).world_lable,horizontal.at(i).world_lable,abs(horizontal.at(i).world_lable - horizontal.at(j).world_lable)));
					}
				}
			}
			
		}

	}

	//arrange squares by reliability
	for(int i=0;i<squares.size();i++)
	{
		for(int j=1;j<squares.size();j++)
		{
			if(squares.at(j).reliability > squares.at(j-1).reliability)
			{
				swap(squares.at(j),squares.at(j-1));
			}
		}
	}

	//Find pnp
	if(squares.size()>0)
	{

		//Mat rvec;
		//Mat tvec;

		//vector<Point3d> obj_m;
		//vector<Point2d> img_m;
		//vector<double> dist_co;
		//vector<double> cam_para;

		//obj_m.push_back(Point3d(squares.at(0).x_pos,squares.at(0).y_pos,0));
		//obj_m.push_back(Point3d(squares.at(0).x_pos+squares.at(0).scale,squares.at(0).y_pos,0));
		//obj_m.push_back(Point3d(squares.at(0).x_pos+squares.at(0).scale,squares.at(0).y_pos+squares.at(0).scale,0));
		//obj_m.push_back(Point3d(squares.at(0).x_pos,squares.at(0).y_pos+squares.at(0).scale,0));

		//img_m.push_back(Point2d(squares.at(0).p1.x,squares.at(0).p1.y));
		//img_m.push_back(Point2d(squares.at(0).p2.x,squares.at(0).p2.y));
  //      img_m.push_back(Point2d(squares.at(0).p3.x,squares.at(0).p3.y));
  //      img_m.push_back(Point2d(squares.at(0).p4.x,squares.at(0).p4.y));  
  //      
		//cam_para.push_back(700.9);
		//cam_para.push_back(0);
		//cam_para.push_back(316.5);
		//cam_para.push_back(0);
		//cam_para.push_back(0);
		//cam_para.push_back(726);
		//cam_para.push_back(241);
		//cam_para.push_back(0);
		//cam_para.push_back(0);
		//cam_para.push_back(0);
		//cam_para.push_back(1);
		//cam_para.push_back(0);

		//dist_co.push_back(0);
		//dist_co.push_back(0);
		//dist_co.push_back(0);
		//dist_co.push_back(0);



		Mat_<double> camera_matrix (3, 3); 
		Mat_<double> dist_coef (5, 1); 


        camera_matrix = 0; 
        camera_matrix(0, 0) = 215; 
        camera_matrix(1, 1) = 215; 
        camera_matrix(2, 2) = 1; 
        camera_matrix(0, 2) = 320; 
        camera_matrix(1, 2) = 240; 

		dist_coef = 0; 
		//pointorigin.push_back(Point3d(0,0,0));
		//pointorigin.push_back(Point3d(0,1,0));
		//pointorigin.push_back(Point3d(1,1,0));
		//pointorigin.push_back(Point3d(1,0,0));
        
		rotation_vectors.clear();
		translation_vectors.clear();
		img_point_sets.clear();
		obj_point_sets.clear();
		img_point_square_sets.clear();
		obj_point_square_sets.clear();
		ProjectPosition.clear();
		ProjectPosition_origin.clear();

		//for(int i=0; i < squares.size();i++)
		//{
  //         vector<Point3d> temp_obj_points;
		//   vector<Point2d> temp_img_points;
		//   vector<Point3d> temp_square_points;
		//   vector<Point2d> temp_img_square_points;
		//   vector<Point3d> point_obj_temp;
		//   vector<Point2d> point_img_temp;
		// 
  //         Mat_<double> rot_m (3, 3); 
  //         Mat_<double> rot_vec (3, 1); 
  //         Mat_<double> trans_vec (3, 1); 
		//   Mat Rx;
  //		   
		//   temp_square_points.push_back(Point3d(squares.at(i).x_pos - squares.at(i).scale,squares.at(i).y_pos - squares.at(i).scale,0));
		//   temp_square_points.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos - squares.at(i).scale,0));
		//   temp_square_points.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos,0));
		//   temp_square_points.push_back(Point3d(squares.at(i).x_pos - squares.at(i).scale,squares.at(i).y_pos,0));


		//   temp_obj_points.push_back(Point3d(squares.at(i).x_pos - squares.at(i).scale,squares.at(i).y_pos - squares.at(i).scale,0));
		//   temp_obj_points.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos - squares.at(i).scale,0));
		//   temp_obj_points.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos,0));
		//   temp_obj_points.push_back(Point3d(squares.at(i).x_pos - squares.at(i).scale,squares.at(i).y_pos,0));
		//
  //         temp_img_points.push_back(Point2d(squares.at(i).p1.x,squares.at(i).p1.y));
		//   temp_img_points.push_back(Point2d(squares.at(i).p2.x,squares.at(i).p2.y));
		//   temp_img_points.push_back(Point2d(squares.at(i).p3.x,squares.at(i).p3.y));
		//   temp_img_points.push_back(Point2d(squares.at(i).p4.x,squares.at(i).p4.y));

  //         solvePnP( temp_obj_points, temp_img_points, camera_matrix, dist_coef, rot_vec, trans_vec );
		//   
		//   projectPoints( temp_square_points, rot_vec, trans_vec, camera_matrix, dist_coef, temp_img_square_points ); 

		//   Rodrigues(rot_vec, Rx); // R is 3x3
 
		//   trans_vec = -Rx * trans_vec; // translation of inverse

		//   Mat T(4, 4, Rx.type()); // T is 4x4
		//   T( Range(0,3), Range(0,3) ) = Rx * 1; // copies R into T
		//   T( Range(0,3), Range(3,4) ) = trans_vec * 1; // copies tvec into T
		//   double *p = T.ptr<double>(3);
		//   p[0] = p[1] = p[2] = 0; p[3] = 1;
  //         
		//   point_obj_temp.push_back(Point3d(-T.at<double>(0,3),-T.at<double>(1,3),0));
		//   projectPoints( point_obj_temp, rot_vec, trans_vec, camera_matrix, dist_coef, point_img_temp ); 
		//   

		//   img_point_sets.push_back(temp_img_points);
		//   obj_point_sets.push_back(temp_obj_points);
		//   rotation_vectors.push_back(Rx);
		//   translation_vectors.push_back(trans_vec);

		//   img_point_square_sets.push_back(temp_img_square_points);
		//   obj_point_square_sets.push_back(temp_square_points);

		//}



		for(int i=0;i<squares.size();i++)
		{
			vector <Point3d> points3d; 
			vector <Point2d> imagePoints; 
		
			vector <Point3d> point0;
			vector <Point3d> pointorigin;
			Mat_<double> rotation_matrix (3, 3); 
			Mat_<double> rotation_vector_calc (3, 1); 
			Mat_<double> translation_vector_calc (3, 1); 
			Mat_<double> rot (3, 3); 

			pointorigin.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos,0));
			pointorigin.push_back(Point3d(squares.at(i).x_pos + squares.at(i).scale,squares.at(i).y_pos,0));
			pointorigin.push_back(Point3d(squares.at(i).x_pos + squares.at(i).scale,squares.at(i).y_pos + squares.at(i).scale,0));
			pointorigin.push_back(Point3d(squares.at(i).x_pos ,squares.at(i).y_pos + squares.at(i).scale,0));

			points3d.push_back(Point3d(squares.at(i).x_pos,squares.at(i).y_pos,0));
			points3d.push_back(Point3d(squares.at(i).x_pos + squares.at(i).scale,squares.at(i).y_pos,0));
			points3d.push_back(Point3d(squares.at(i).x_pos + squares.at(i).scale,squares.at(i).y_pos + squares.at(i).scale,0));
			points3d.push_back(Point3d(squares.at(i).x_pos ,squares.at(i).y_pos + squares.at(i).scale,0));

			imagePoints.push_back(Point2d(squares.at(i).p1.x,squares.at(i).p1.y));
			imagePoints.push_back(Point2d(squares.at(i).p2.x,squares.at(i).p2.y));
			imagePoints.push_back(Point2d(squares.at(i).p3.x,squares.at(i).p3.y));
			imagePoints.push_back(Point2d(squares.at(i).p4.x,squares.at(i).p4.y));

			solvePnP( points3d, imagePoints, camera_matrix, dist_coef, rotation_vector_calc, translation_vector_calc );

			projectPoints( pointorigin, rotation_vector_calc, translation_vector_calc, camera_matrix, dist_coef, imagePoints_origin ); 
			

            Mat R;
			Mat temp_T;
			temp_T = translation_vector_calc;
			

		    Rodrigues(rotation_vector_calc, R); // R is 3x3
            temp_T = -R*temp_T;

			//R = R.t();  // rotation of inverse
		    translation_vector_calc = -R * translation_vector_calc; // translation of inverse

		    Mat T(4, 4, R.type()); // T is 4x4
		    T( Range(0,3), Range(0,3) ) = R * 1; // copies R into T
		    T( Range(0,3), Range(3,4) ) = translation_vector_calc * 1; // copies tvec into T
			// fill the last row of T (NOTE: depending on your types, use float or double)
			double *p = T.ptr<double>(3);
			p[0] = p[1] = p[2] = 0; p[3] = 1;

			point0.push_back(Point3d(-T.at<double>(0,3),-T.at<double>(1,3),0));

			ProjectPosition.push_back(Point2d(-T.at<double>(0,3),-T.at<double>(1,3)));
			ProjectPosition_origin.push_back(Point3d(-T.at<double>(0,3),-T.at<double>(1,3),T.at<double>(2,3)));
		    projectPoints( point0, rotation_vector_calc, translation_vector_calc, camera_matrix, dist_coef, imagePoints0 ); 

			img_point_square_sets.push_back(imagePoints_origin);
			img_point_sets.push_back(imagePoints0);
	

		    Mat Ri = R.inv();

			
			//for(int i=0;i<translation_vector_calc.rows;i++)
			//{
			//	for(int j=0;j<translation_vector_calc.cols;j++)
			//	{
			//		cout<<translation_vector_calc.at<double>(i,j);
			//		cout<<' ';
			//	}
			//	cout<<'\n';
			//}

			cout<<T.at<double>(0,3)<<'\n';
			cout<<T.at<double>(1,3)<<'\n';
			cout<<T.at<double>(2,3)<<'\n'<<'\n';
		
			coordinate_tracker = Point3d(ProjectPosition_origin.at(0).x,ProjectPosition_origin.at(0).y,ProjectPosition_origin.at(0).z);
		cout << "coordinate_tracker: " << coordinate_tracker << endl;
		translation_vector_calc_.push_back(coordinate_tracker);
				
		l = translation_vector_calc_.size();
		cout << "the size : " << translation_vector_calc_.size() << endl;
		cout << "the ProjectPosition_origin: " << translation_vector_calc_ <<endl;
		//cout << "the ProjectPosition_dange : " << translation_vector_calc_.at(l).x <<endl;	
				
		vector<Point3d>::iterator itr = translation_vector_calc_.begin();
		
		if(l <= 1)
		{
			cout << "WARNING: tranlsation vectors are empty!" << endl;
			//coordinate_tracker_hopped = coordinate_tracker;
		}
		
		else
		{
			
			error_x = (translation_vector_calc_.at(l-1).x-translation_vector_calc_.at(l-2).x);
			error_y=  (translation_vector_calc_.at(l-1).y-translation_vector_calc_.at(l-2).y);

			if (error_x > 0.8 ) translation_vector_calc_.at(l-1).x = translation_vector_calc_.at(l-1).x-1.0;
			if (error_x > 1.8 ) translation_vector_calc_.at(l-1).x = translation_vector_calc_.at(l-1).x-2.0;
			if (error_x < -0.8) translation_vector_calc_.at(l-1).x = translation_vector_calc_.at(l-1).x+1.0;
			if (error_x < -1.8) translation_vector_calc_.at(l-1).x = translation_vector_calc_.at(l-1).x+2.0;
			if (error_y > 0.8)  translation_vector_calc_.at(l-1).y = translation_vector_calc_.at(l-1).y-1.0;
			if (error_y > 1.8)  translation_vector_calc_.at(l-1).y = translation_vector_calc_.at(l-1).y-2.0;
			if (error_y < -0.8) translation_vector_calc_.at(l-1).y = translation_vector_calc_.at(l-1).y+1.0;
			if (error_y < -1.8) translation_vector_calc_.at(l-1).y = translation_vector_calc_.at(l-1).y+2.0;
			cout << "translation_vector_calc be hopped : " << translation_vector_calc_[l-1] << endl;

			coordinate_tracker_hopped = Point3d(translation_vector_calc_.at(l-1).x,translation_vector_calc_.at(l-1).y,translation_vector_calc_.at(l-1).z);
			translation_vector_calc_.erase(itr);	
		}
		


		}

		//points3d.push_back(Point3d(-1,-1,0));
		//points3d.push_back(Point3d(0,-1,0));
		//points3d.push_back(Point3d(0,0,0));
		//points3d.push_back(Point3d(-1,0,0));

       
        
		//translation_vector(2, 0) = 106; 
		
	    //projectPoints( points3d, rotation_vector, translation_vector, camera_matrix, dist_coef, imagePoints ); 

		//solvePnP( points3d, imagePoints, camera_matrix, dist_coef, rotation_vector, translation_vector, true ); 
		
		//point0.push_back(Point3d(translation_vector_calc(0,0),translation_vector_calc(1,0),0));

		




	
	}
	else
	{
		squares.clear();  
		imagePoints0.clear(); 
		imagePoints_origin.clear(); 
		ProjectPosition.clear(); 
		ProjectPosition_origin.clear();
    }
	//End find pnp
	


	horizontal_last = horizontal;
	vertical_last = vertical;

}



void Tracker::Draw2Image(IplImage* res_im)
{
	if((horizontal.size()<2) && (vertical.size()<2)) return;


	/*Draw current relative position*/
	//for(int i=0; i<horizontal.size();i++)
	//{
	//	char temp[100];
	//	CvFont font;
	//	sprintf(temp,"H:%d",horizontal.at(i).world_lable);
	//	cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
	//	cvPutText(res_im,temp,cvPoint(50,100+i*20),&font,CV_RGB(140,50,30));
	//}
	//for(int i=0; i<vertical.size();i++)
	//{
	//	char temp[100];
	//	CvFont font;
	//	sprintf(temp,"V:%d",vertical.at(i).world_lable);
	//	cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
	//	cvPutText(res_im,temp,cvPoint(100,100+i*20),&font,CV_RGB(140,50,30));
	//}

	for(int i=0; i<1;i++)
	{
		char temp[100];
		CvFont font;
		sprintf(temp,"Initialized: %d",initialized);
		cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
		cvPutText(res_im,temp,cvPoint(50,100),&font,CV_RGB(10,50,30));
	}

	//for(int i=0; i<frame_offset_h.size();i++)
	//{
	//	char temp0[100];
	//	CvFont font;
	//	sprintf(temp0,"OffsetH: %d",frame_offset_h.at(i));
	//	cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
	//	cvPutText(res_im,temp0,cvPoint(10,200+20*i),&font,CV_RGB(30,30,30));
	//}

	//for(int i=0;i<frame_offset_v.size();i++)
	//{
	//	char temp1[100];
	//	CvFont font;
	//	sprintf(temp1,"OffsetV: %d",frame_offset_v.at(i));
	//	cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
	//	cvPutText(res_im,temp1,cvPoint(130,200+20*i),&font,CV_RGB(30,30,30));
	//}


	for (int i = 0; i < vertical.size(); i++)
	{
		char temp[100];
		CvFont font;
		sprintf(temp,"V:%d",vertical.at(i).world_lable);
		cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
		cvPutText(res_im,temp,cvPoint((vertical.at(i).p1().x + vertical.at(i).p2().x)/2,(vertical.at(i).p1().y + vertical.at(i).p2().y)/2+20),&font,CV_RGB(0,0,0));
	}
	for (int i = 0; i < horizontal.size(); i++)
	{
		char temp[100];
		CvFont font;
		sprintf(temp,"H:%d",horizontal.at(i).world_lable);
		cvInitFont(&font,CV_FONT_VECTOR0,0.6,0.6,0,2,8);
		cvPutText(res_im,temp,cvPoint((horizontal.at(i).p1().x + horizontal.at(i).p2().x)/2+20, (horizontal.at(i).p1().y + horizontal.at(i).p2().y)/2),&font,CV_RGB(0,0,0));	
	}

	vector<CvScalar> colors;
	int count_color=0;
	colors.push_back(CV_RGB(255,0,0));
	colors.push_back(CV_RGB(200,150,0));
	colors.push_back(CV_RGB(200,200,0));
	colors.push_back(CV_RGB(150,220,0));
	colors.push_back(CV_RGB(120,250,0));
	colors.push_back(CV_RGB(50,255,0));
	colors.push_back(CV_RGB(20,255,0));
	colors.push_back(CV_RGB(0,255,20));
	colors.push_back(CV_RGB(0,255,80));
	colors.push_back(CV_RGB(0,220,120));
	colors.push_back(CV_RGB(0,200,150));
	colors.push_back(CV_RGB(0,150,200));
	colors.push_back(CV_RGB(0,120,220));
	colors.push_back(CV_RGB(0,100,250));
	colors.push_back(CV_RGB(0,80,250));
	colors.push_back(CV_RGB(80,0,250));
	colors.push_back(CV_RGB(100,0,200));
	colors.push_back(CV_RGB(150,0,200));
	colors.push_back(CV_RGB(150,0,150));
	colors.push_back(CV_RGB(120,80,120));
	colors.push_back(CV_RGB(100,100,100));
	colors.push_back(CV_RGB(90,90,90));
	colors.push_back(CV_RGB(80,80,80));
	colors.push_back(CV_RGB(70,70,70));
	colors.push_back(CV_RGB(60,60,60));
	colors.push_back(CV_RGB(50,50,50));


	for(int i=0; i< squares.size(); i++)
	{
		CvPoint p1,p2,p3,p4;
		p1 = cvPoint((int)(squares.at(i).p1.x),(int)(squares.at(i).p1.y));
		p2 = cvPoint((int)(squares.at(i).p2.x),(int)(squares.at(i).p2.y));
		p3 = cvPoint((int)(squares.at(i).p3.x),(int)(squares.at(i).p3.y));
		p4 = cvPoint((int)(squares.at(i).p4.x),(int)(squares.at(i).p4.y));

        double width;
		if(i==0) width =2;
		else if(i<4) width =1.5;
		else width =0.5;

		cvLine(res_im,p1,p2,colors.at(count_color),width,CV_AA);
		cvLine(res_im,p2,p3,colors.at(count_color),width,CV_AA);
		cvLine(res_im,p3,p4,colors.at(count_color),width,CV_AA);
		cvLine(res_im,p4,p1,colors.at(count_color),width,CV_AA);
	
		count_color++;
		if(count_color >= 24)
		{
			count_color = 0;
		}
	}
	 


	if(ProjectPosition.size()>0)
	{
	 CvFont font;
	 cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1.8,8);
	 char temp[100];
	 //sprintf(temp,"X: %.2f ; Y: %.2f;",coordinate_tracker_hopped.x ,coordinate_tracker_hopped.y);
	 //sprintf(temp,"X: %.2f ; Y: %.2f;",translation_vector_calc_.at(1).x ,translation_vector_calc_.at(1).y);
	 sprintf(temp,"X: %.2f ; Y: %.2f;",ProjectPosition.at(0).x ,ProjectPosition.at(0).y);
	 cvPutText(res_im,temp,cvPoint((int)imagePoints0.at(0).x+10,(int)imagePoints0.at(0).y+10),&font,CvScalar(CV_RGB(255,0,0)));
	
	}
	 //cvCircle(res_im,cvPoint(320,240),2,CvScalar(CV_RGB(255,0,0)),CV_AA,1);

	//for(int i=0; i<imagePoints_origin.size();i++)
	//{
	// cvCircle(res_im,cvPoint((int)imagePoints_origin.at(i).x,(int)imagePoints_origin.at(i).y),0.5,CvScalar(CV_RGB(0,0,255)),CV_AA,1);
	//}
	
	if(robot_sets.size()>0)
	{
		CvFont font;
		cvInitFont(&font,CV_FONT_VECTOR0,0.5,0.5,0,1.8,8);
		char temp[100];

		for(int i=0; i<robot_sets.size();i++)
		 {
			 if(robot_sets.at(i).active)
			 {
			  char temp[100];	
			  cvCircle(res_im,cvPoint(robot_sets.at(i).img_pos.x,robot_sets.at(i).img_pos.y),1,CvScalar(CV_RGB(245,175,0)),CV_AA,1);
			  sprintf(temp,"X: %.2f ; Y: %.2f; ID: %d",robot_sets.at(i).real_pos.x ,robot_sets.at(i).real_pos.y,robot_sets.at(i).id);
			  cvPutText(res_im,temp,cvPoint(robot_sets.at(i).img_pos.x +10,robot_sets.at(i).img_pos.y+10),&font,CvScalar(CV_RGB(220,0,0)));	 
			 }
		 }
	}


	for(int i=0;i<img_point_square_sets.size();i++)
	{
		for(int j=0; j<img_point_square_sets.at(i).size();j++)
		{
			cvCircle(res_im,cvPoint((int)img_point_square_sets.at(i).at(j).x,(int)img_point_square_sets.at(i).at(j).y),1,CvScalar(CV_RGB(0,0,255-20*i)),CV_AA,1);
		}
	}

	for(int i=0;i<img_point_sets.size();i++)
	{
		for(int j=0; j<img_point_sets.at(i).size();j++)
		{
			cvCircle(res_im,cvPoint((int)img_point_sets.at(i).at(j).x,(int)img_point_sets.at(i).at(j).y),0.2,CvScalar(CV_RGB(0,255-20*i,0)),CV_AA,1);		
		}
	
	}

	if(imagePoints0.size()>0)
	{
	 cvCircle(res_im,cvPoint((int)imagePoints0.at(0).x,(int)imagePoints0.at(0).y),2,CvScalar(CV_RGB(255,0,0)),CV_AA,1);
	}

	//for(int i=0;i<horizontal_last.size();i++)
	//{
	//	cvLine(res_im,horizontal_last[i].p1(),horizontal_last[i].p2(),CV_RGB(70,70,70),2,CV_AA);
	//}
	//for(int i=0;i<vertical_last.size();i++)
	//{
	//	cvLine(res_im,vertical_last[i].p1(),vertical_last[i].p2(),CV_RGB(70,70,70),2,CV_AA);
	//}


}

bool Tracker::FindOffSet()
{
	double min_horizontal_d=500;
	double min_vertical_d=500;
	double average_d=0;
	int average_d_count=0;

	for(int i=0; i<horizontal.size(); i++)
	{
		if(horizontal.at(i).d_to_post>0)
		{
			if((horizontal.at(i).d_to_post<min_horizontal_d) && (horizontal.at(i).d_to_post > 80))
				min_horizontal_d = horizontal.at(i).d_to_post;
		}
	}
	for(int i=0; i<vertical.size(); i++)
	{
		if(vertical.at(i).d_to_post>0)
		{
			if((vertical.at(i).d_to_post<min_vertical_d) && (vertical.at(i).d_to_post > 80))
				min_vertical_d = vertical.at(i).d_to_post;
		}
	}

	for(int i=0; i<horizontal.size(); i++)
	{
		if(horizontal.at(i).d_to_post>0)
		{
			//if(horizontal[i].d_to_post<min_horizontal_d)
			//	min_horizontal_d = horizontal[i].d_to_post;
			if((horizontal.at(i).d_to_post / min_horizontal_d) < 1.8)
			{
				average_d+=horizontal.at(i).d_to_post;
			   average_d_count++;
			}
		}
	}
	for(int i=0; i<vertical.size(); i++)
	{
		if(vertical.at(i).d_to_post>0)
		{
			//if(vertical[i].d_to_post<min_vertical_d)
			//	min_vertical_d = vertical[i].d_to_post;
			if((vertical.at(i).d_to_post / min_horizontal_d ) < 1.8)
			{
				average_d+=vertical.at(i).d_to_post;
			  average_d_count++;
			}
		}
	}

		if(horizontal.size()<2 || vertical.size()<2)
	{
	    return false;
		printf("Lack of detected grids;");
	}

	average_d = average_d / average_d_count;
	//d_upper_threshold = max(min_horizontal_d,min_vertical_d)+30;
	//d_lower_threshold = min(min_horizontal_d,min_vertical_d)-5;


	d_upper_threshold = 1.8*average_d;
	d_lower_threshold = 60;

	int middle = (int)horizontal.size()/2+0.5;
	if(horizontal.size() >= 2)
	{
		horizontal.at(middle).world_lable = 0;
		horizontal.at(middle).offset = 0; 
		horizontal.at(middle).confidence_level = 1;
	}
	for(int i=1;(i+middle)<horizontal.size();i++)
	{
		if((horizontal.at(middle+i).d_to_pre > d_lower_threshold)&&horizontal.at(middle+i).d_to_pre < d_upper_threshold)
		{
			horizontal.at(middle+i).offset = 1;
			horizontal.at(middle+i).world_lable = horizontal.at(middle+i-1).world_lable +  horizontal.at(middle+i).offset;
			horizontal.at(middle+i).confidence_level = 1-0.1*i;
		}
		else 
		{
			double mul = horizontal.at(middle+i).d_to_pre / average_d;
			if((abs((int)(mul+0.5) - mul)>0.4) && (abs((int)(mul+0.5) - mul)<0.6))
			{
               //Likely false detection
				int count =0;
				for(vector<GridLine>::iterator iter = horizontal.begin() ; iter!=horizontal.end();iter++)
				{
					if(count == middle+i)
					{
						horizontal.erase(iter);
						break;
					}
				    count++;
				}
			}
			else
			{
				horizontal.at(middle+i).offset = (int) (mul+0.5);
				horizontal.at(middle+i).world_lable = horizontal.at(middle+i-1).world_lable +  horizontal.at(middle+i).offset;
				horizontal.at(middle+i).confidence_level = 1-0.1*i - abs((int)(mul+0.5) - mul);

			}
		}
	}
	for(int i=1;(middle-i)>=0;i++)
	{
		if((horizontal.at(middle-i).d_to_post > d_lower_threshold)&&horizontal.at(middle-i).d_to_post < d_upper_threshold)
		{
			horizontal.at(middle-i).offset = -1;
			horizontal.at(middle-i).world_lable = horizontal.at(middle-i+1).world_lable + horizontal.at(middle-i).offset;
			horizontal.at(middle-i).confidence_level = 1-0.1*i;
		}
		else 
		{
			double mul = horizontal[middle-i].d_to_post / average_d;
			if((abs((int)(mul+0.5) - mul)>0.5) && (abs((int)(mul+0.5) - mul)<0.6))
			{
               //Likely false detection
				int count =0;
				for(vector<GridLine>::iterator iter = horizontal.begin() ; iter!=horizontal.end();iter++)
				{
					if(count == middle-i)
					{
						horizontal.erase(iter);
						break;
					}
				    count++;
				}
			}
			else
			{
				horizontal.at(middle-i).offset =  - (int) (mul+0.5);
				horizontal.at(middle-i).world_lable = horizontal.at(middle-i+1).world_lable +  horizontal.at(middle-i).offset;
				horizontal.at(middle-i).confidence_level = 1-0.1*i - abs((int)(mul+0.5) - mul);

			}
		}
	}
	

	middle = (int)vertical.size()/2+0.5;
	if(vertical.size() >= 2)
	{
		vertical.at(middle).world_lable = 0;
		vertical.at(middle).offset = 0; 
		vertical.at(middle).confidence_level = 1;
	}
	for(int i=1;(i+middle)<vertical.size();i++)
	{
		if((vertical.at(middle+i).d_to_pre > d_lower_threshold)&&vertical.at(middle+i).d_to_pre < d_upper_threshold)
		{
			vertical.at(middle+i).offset = 1;
			vertical.at(middle+i).world_lable = vertical.at(middle+i-1).world_lable +  vertical.at(middle+i).offset;
			vertical.at(middle+i).confidence_level = 1-0.1*i;
		}
		else 
		{
			double mul = vertical.at(middle+i).d_to_pre / average_d;
			if((abs((int)(mul+0.5) - mul)>0.5) && (abs((int)(mul+0.5) - mul)<0.6))
			{
               //Likely false detection
				int count =0;
				for(vector<GridLine>::iterator iter = vertical.begin() ; iter!=vertical.end();iter++)
				{
					if(count == middle+1)
					{
						vertical.erase(iter);
						break;
					}
				    count++;
				}
			}
			else
			{
				vertical.at(middle+i).offset = (int) (mul+0.5);
				vertical.at(middle+i).world_lable = vertical.at(middle+i-1).world_lable +  vertical.at(middle+i).offset;
				vertical.at(middle+i).confidence_level = 1-0.1*i - abs((int)(mul+0.5) - mul);

			}
		}


	}
	for(int i=1;(middle-i)>=0;i++)
	{
		if((vertical.at(middle-i).d_to_post > d_lower_threshold)&&(vertical.at(middle-i).d_to_post < d_upper_threshold))
		{
			vertical.at(middle-i).offset = 1;
			vertical.at(middle-i).world_lable = vertical.at(middle-i+1).world_lable - vertical.at(middle-i).offset;
			vertical.at(middle-i).confidence_level = 1-0.1*i;
		}
		else 
		{
			double mul = vertical.at(middle-i).d_to_post / average_d;
			if((abs((int)(mul+0.5) - mul)>0.5) && ((abs((int)(mul+0.5) - mul)<0.6)))
			{
               //Likely false detection
				int count =0;
				for(vector<GridLine>::iterator iter = vertical.begin() ; iter!=vertical.end();iter++)
				{
					if(count == middle-i)
					{
						vertical.erase(iter);
						break;
					}
				    count++;
				}
			}
			else
			{
				vertical.at(middle-i).offset =  - (int) (mul+0.5);
				vertical.at(middle-i).world_lable = vertical.at(middle-i+1).world_lable +  vertical.at(middle-i).offset;
				vertical.at(middle-i).confidence_level = 1-0.1*i - abs((int)(mul+0.5) - mul);

			}
		}

	}

	for(int i=0; i<(horizontal.size()-1);i++)
	{
		if ((horizontal.at(i).world_lable>=horizontal.at(i+1).world_lable) || (horizontal.at(i).world_lable < -99))
		{
            return false;
			printf("Initialization error;");
		}
	}
	if(horizontal.at(horizontal.size()-1).world_lable<-99) return false;

	for(int i=0; i<(vertical.size()-1);i++)
	{
		if ((vertical.at(i).world_lable>=vertical.at(i+1).world_lable) || (vertical.at(i).world_lable < -99))
		{
            return false;
			printf("Initialization error;");
		}
	}

	if(vertical.at(vertical.size()-1).world_lable<-99) return false;

	return true;
}

int Tracker::FindOffsetMode(vector<int> offset)
{
	vector<int> temp;
	if (offset.size()<1) return false;
	for(int i=0; i<offset.size();i++) temp.push_back(0);
	for(int i=0; i<temp.size();i++)
	{
		for(int j=0; j<temp.size();j++)
		{
			if(offset.at(i) == offset.at(j))
			{
				temp.at(i)++;
			}
		}
	}
	int max=-1000;
	int count =-1;
	for(int i=0; i<temp.size();i++)
	{
		if(temp.at(i)>max)
		{
			max = temp.at(i);
			count = i;
		}
	}
	return offset.at(count);
}

void Tracker::FindRealRobotPos(vector<robot_unit> robots)
{
 	robot_sets = robots;
	if(horizontal.size()<2 || vertical.size()<2) return;
	for(int i=0; i<robot_sets.size();i++)
	{
		double x=0,y=0;
		double d_min = 1000;
		int count = -1;
		for(int j=0; j<horizontal.size();j++)
		{
			double d = horizontal.at(j).toPoint(robot_sets.at(i).img_pos);
			if(d < d_min)
			{
			    d_min = d;
				count = j;
			}
		}

		if(horizontal.at(count).d_to_pre >0)
		{
				if(horizontal.at(count-1).toPoint(robot_sets.at(i).img_pos)>horizontal.at(count).d_to_pre)
				    y = horizontal.at(count).world_lable + (d_min / horizontal.at(count).d_to_pre) * (horizontal.at(count).world_lable - horizontal.at(count-1).world_lable);
				else
					y = horizontal.at(count).world_lable - (d_min / horizontal.at(count).d_to_pre) * (horizontal.at(count).world_lable - horizontal.at(count-1).world_lable);
			
		}
		else if(horizontal.at(count).d_to_post >0)
		{ 
				if(horizontal.at(count+1).toPoint(robot_sets.at(i).img_pos)>horizontal.at(count).d_to_post)
			        y = horizontal.at(count).world_lable - (d_min / horizontal.at(count).d_to_post) * ( horizontal.at(count+1).world_lable - horizontal.at(count).world_lable);
				else
					y = horizontal.at(count).world_lable + (d_min / horizontal.at(count).d_to_post) * ( horizontal.at(count+1).world_lable - horizontal.at(count).world_lable);
		}

		d_min = 1000;
		count = -1;
		for(int j=0; j<vertical.size();j++)
		{
			double d = vertical.at(j).toPoint(robot_sets.at(i).img_pos);
			if(d < d_min)
			{
			    d_min = d;
				count = j;
			}
		}
		
		if(vertical.at(count).d_to_pre >0)
		{
				if(vertical.at(count-1).toPoint(robot_sets.at(i).img_pos)>vertical.at(count).d_to_pre)
				    x = vertical.at(count).world_lable + (d_min / vertical.at(count).d_to_pre) * (vertical.at(count).world_lable - vertical.at(count-1).world_lable);
				else 
					x = vertical.at(count).world_lable - (d_min / vertical.at(count).d_to_pre) * (vertical.at(count).world_lable - vertical.at(count-1).world_lable);
		}
		else if(vertical.at(count).d_to_post >0)
		{
				if(vertical.at(count+1).toPoint(robot_sets.at(i).img_pos)>vertical.at(count).d_to_post)
			        x = vertical.at(count).world_lable - (d_min / vertical.at(count).d_to_post) * (vertical.at(count+1).world_lable - vertical.at(count).world_lable );
				else
					x = vertical.at(count).world_lable + (d_min / vertical.at(count).d_to_post) * (vertical.at(count+1).world_lable - vertical.at(count).world_lable );
			
		}

		robot_sets.at(i).SetRealXY(Point2d(x,y));
	}

}
