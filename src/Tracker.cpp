#include "lsd_line/robot_unit.h"
#include "lsd_line/Tracker.h"
#include "lsd_line/Matrix.h"

TRACKER::TRACKER(void)
{
	initialized = false;
	frame_offset_vertical = 0;
	frame_offset_horizontal = 0;


	translation_vector = Mat_<double> (3,1);
    rotation_vector = Mat_<double>(3,1);
	translation_vector = 0;
    rotation_vector = 0;

}

/**
TRACKER::TRACKER():nh("~")
{
    ROS_INFO("Starting lsd_line node...");
    string iImg_topic,path_topic;
    nh.param("iImg_topic",iImg_topic,string("/usb_cam/image_rect"));
    nh.param("true_pose",path_topic,string("true_pose"));

    image_sub = nh.subscribe(iImg_topic.c_str(),1,&TRACKER::imageCallback,this);
    path_sub = nh.subscribe(path_topic.c_str(), 1, &TRACKER::pathCallback,this);

    path_pub = nh.advertise<nav_msgs::Path>("path",10);
	path_lsd_pub = nh.advertise<nav_msgs::Path>("path_img",10);
    frame_count=0;

    initialized = false;
	frame_offset_vertical = 0;
	frame_offset_horizontal = 0;


	translation_vector = Mat_<double> (3,1);
    rotation_vector = Mat_<double>(3,1);
	translation_vector = 0;
    rotation_vector = 0;

    scale_int = 8;
    sigma_scale_int = 8;
    quant_int = 2;
    ang_th_int = 90;
    eps_int = 0;
    density_th_int = 8;
    n_bins = 2048;
    max_grad_int = 625;


}
**/
TRACKER::~TRACKER()
{

}

CvPoint2D64f TRACKER::FindLineCross(Line line1,Line line2)
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




bool TRACKER::IsInitialized()
{
	return initialized;
}

bool TRACKER::Initialize(vector<Line> horizontal_raw, vector<Line> vertical_raw)
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

void TRACKER::LoadGrids(vector<Line> horizontal_raw, vector<Line> vertical_raw)
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

void TRACKER::ProcessFrame(vector<Line> horizontal_raw, vector<Line> vertical_raw)
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
	   //printf("Reinitialization required");
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
        camera_matrix(0, 0) = 409.385023;
        camera_matrix(1, 1) = 318.07321;
        camera_matrix(2, 2) = 1;
        camera_matrix(0, 2) = 410.856631;
        camera_matrix(1, 2) = 239.435546;

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
		translation_vector_calc_.clear();

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
		    translation_vector_calc = R.t() * translation_vector_calc;
		    //translation_vector_calc = -R * translation_vector_calc; // translation of inverse

		    Mat T(4, 4, R.type()); // T is 4x4
		    T( Range(0,3), Range(0,3) ) = R.t() * 1; // copies R into T
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

			//cout<<T.at<double>(0,3)<<'\n';
			//cout<<T.at<double>(1,3)<<'\n';
			//cout<<T.at<double>(2,3)<<'\n'<<'\n';

        coordinate_tracker = Point3d(T.at<double>(0,3),-T.at<double>(1,3),-T.at<double>(2,3));
		//cout << "coordinate_tracker1: " << coordinate_tracker << endl;
		//cout << "the ProjectPosition_origin: " <<  coordinate_tracker.x << " " <<coordinate_tracker.y << " "<< coordinate_tracker.z << endl;
		translation_vector_calc_.push_back(coordinate_tracker);
		}

		//cout << "translation_vector_calc_: " << translation_vector_calc_ << endl;

		for (int i = 0; i<translation_vector_calc_.size()-1; i++)
		{
            coordinate_tracker.x += translation_vector_calc_.at(i).x;
            coordinate_tracker.y += translation_vector_calc_.at(i).y;
            coordinate_tracker.z += translation_vector_calc_.at(i).z;
		}

		coordinate_tracker2.x = coordinate_tracker.x/translation_vector_calc_.size()+0.2;
		coordinate_tracker2.y = coordinate_tracker.y/translation_vector_calc_.size()-0.3;
		coordinate_tracker2.z = coordinate_tracker.z/translation_vector_calc_.size()+0.3;

		translation_vector_calc_hop_.push_back(coordinate_tracker2);

        //cout << "translation_vector_calc_hop_: " << translation_vector_calc_hop_ << endl;

		l = translation_vector_calc_hop_.size();
		//cout << "the size : " << translation_vector_calc_.size() << endl;
		//cout << "the ProjectPosition_origin: " << translation_vector_calc_ <<endl;
		//cout << "the ProjectPosition_dange : " << translation_vector_calc_.at(l).x <<endl;

		vector<Point3d>::iterator itr = translation_vector_calc_hop_.begin();

		if(l <= 1)
		{
			//cout << "WARNING: tranlsation vectors are empty!" << endl;
			coordinate_tracker_hopped = coordinate_tracker2;
		}

		else
		{

			error_x = (translation_vector_calc_hop_.at(l-1).x-translation_vector_calc_hop_.at(l-2).x);
			error_y=  (translation_vector_calc_hop_.at(l-1).y-translation_vector_calc_hop_.at(l-2).y);

			if (error_x > 0.6 && error_x < 1.6) translation_vector_calc_hop_.at(l-1).x -= 1.0;
			if (error_x >= 1.6 ) translation_vector_calc_hop_.at(l-1).x = translation_vector_calc_hop_.at(l-1).x-2.0;
			if (error_x < -0.6 && error_x > -1.6) translation_vector_calc_hop_.at(l-1).x = translation_vector_calc_hop_.at(l-1).x+1.0;
			if (error_x <= -1.6) translation_vector_calc_hop_.at(l-1).x = translation_vector_calc_hop_.at(l-1).x+2.0;
			if (error_y > 0.6 && error_y < 1.6)  translation_vector_calc_hop_.at(l-1).y = translation_vector_calc_hop_.at(l-1).y-1.0;
			if (error_y >= 1.6)  translation_vector_calc_hop_.at(l-1).y = translation_vector_calc_hop_.at(l-1).y-2.0;
			if (error_y < -0.6 && error_y > -1.6) translation_vector_calc_hop_.at(l-1).y = translation_vector_calc_hop_.at(l-1).y+1.0;
			if (error_y < -1.6) translation_vector_calc_hop_.at(l-1).y = translation_vector_calc_hop_.at(l-1).y+2.0;
			//cout << "translation_vector_calc be hopped : " << translation_vector_calc_[l-1] << endl;
           // cout <<translation_vector_calc_[l-1].x <<" " << translation_vector_calc_[l-1].y << " "<<translation_vector_calc_[l-1].z << endl;
			//cout << " " << endl;
			coordinate_tracker_hopped = Point3d(translation_vector_calc_hop_.at(l-1).x,translation_vector_calc_hop_.at(l-1).y,translation_vector_calc_hop_.at(l-1).z);
			translation_vector_calc_hop_.erase(itr);
			//cout << "coordinated_tracker_hopped:" << coordinate_tracker_hopped << endl;
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



void TRACKER::Draw2Image(IplImage* res_im)
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

bool TRACKER::FindOffSet()
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


int TRACKER::FindOffsetMode(vector<int> offset)
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


void TRACKER::FindRealRobotPos(vector<robot_unit> robots)
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

/***
void TRACKER::pathCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{


	temp.pose.position.x = msg-> pose.position.x;
	temp.pose.position.y = msg-> pose.position.y;
	temp.pose.position.z = msg-> pose.position.z;
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

void TRACKER::imageCallback(const sensor_msgs::Image::ConstPtr msg)
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
/***
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
		ProcessFrame(cl.GetHorizontalLines(),cl.GetVerticalLines());
        geometry_msgs::PoseStamped pose_msgs;
		path_msg.header.stamp = msg-> header.stamp;
		path_msg.header.frame_id = "map";
		pose_msgs.pose.position.x = coordinate_tracker.x;
		pose_msgs.pose.position.x = coordinate_tracker.y;
		pose_msgs.pose.position.x = coordinate_tracker.z;
		path_msg.poses.push_back(pose_msgs);
		path_lsd_pub.publish(path_msg);

	/***
        geometry_msgs::Point coordinate_hopped;
        coordinate_hopped.x = tr.coordinate_tracker_hopped.x;
        coordinate_hopped.y = tr.coordinate_tracker_hopped.y;
        coordinate_hopped.z = tr.coordinate_tracker_hopped.z;
        coordinate_pub_hopped.publish(coordinate_hopped);
	****/

	/**
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

	//coordinate_pub_hopped.publish(coordinate_hopped);


		//tr.FindRealRobotPos(robots.GetImgPos());
		Draw2Image(res_im);



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

***/





