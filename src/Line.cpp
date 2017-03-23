
#include "lsd_line/Line.h"
#include <opencv2/opencv.hpp>
#include <list>
#include <vector>
#include "math.h"

#define Pi 3.1415926



using namespace std;

Line::Line()
{
}

Line::Line(CvPoint p1, CvPoint p2) 
{
    point1 = p1;
    point2 = p2;
	  
	theta = atan2 ((p1.y - p2.y) , (p1.x - p2.x));
               
        if(p1.x == p2.x)
        {
            // y = kx + b
            A = 1;
            B = 0;
            C = p1.x;
        }
        else
        {
            // y = kx + C
            A = 1.0*(p1.y - p2.y) / (p1.x - p2.x);
            B = -1;
            C = p1.y - 1.0* p1.x*(p1.y - p2.y) / (p1.x - p2.x);
        }

    d = abs(C) /sqrt(A * A + B * B);
	if(theta<0) theta += Pi;
	length = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

Line::Line(double angle, double distance, CvPoint near)
{
    theta = angle;
    d = distance;
    if (tan(theta) == 1)
    {
        A = 1;
        B = 0;
        if (abs(near.x - d) < abs(near.x + d))
        {
            C = -d;
        }
        else
        {
            C = d;
        }
    }
    else
    {
        A = tan(theta);
        B = -1;
        C = d * sqrt(A * A + B * B);
        if (abs(A * near.x + B * near.y - C) < abs(A * near.x + B * near.y + C))
        {
            C = -C;
        }
    }
}


Line::~Line()
{
        
}

double Line::Get_theta() {
   return theta;
}

double Line::Get_distance() {
   return d;
}

CvPoint Line::p1(){
   return point1;
}

CvPoint Line::p2() {
   return point2;
}

vector<CvPoint> Line::Get_boundary(){
    // y = kx + b
    // Ax +By + C = 0
    // x = 0
    // y = 0
    // x = a;
    // y = a;
	vector<CvPoint> points;
    points = vector<CvPoint>(2);

    if (B != 0)
    {
        points[0].x = 0;
		points[0].y = (int)(-C / B);
        points[1].x = 800;
        points[1].y = (int)((-C - 800 * A) / B);
    }
    else
    {
        points[0].x = (int)(-C / A);
        points[0].y = 0;
        points[1].x = (int)(-C / A);
        points[1].y = 800;
    }

    /*
    if (B == 0)
    {
        points[0].X = (int)(-C / A);
        points[1].X = (int)(-C / A);
        points[0].Y = p1.Y;
        points[1].Y = p2.Y;
    }
    else
    {
        //a.X = p1.X;
        //a.Y = (int)((-C - A * p1.X) / B);
        //b.X = p1.X;
        //b.Y = (int)((-C - A * p2.X) / B);
                
            
            
    }
    */
    return points;
}

vector<CvPoint> Line::Get_boundary(CvPoint p1, CvPoint p2)
{
	// y = kx + b
	// Ax +By + C = 0
	// x = 0
	// y = 0
	// x = a;
	// y = a;
	// Ax + By + C = 0
	vector<CvPoint> points;
	CvPoint min, max;
	min = cvPoint(MIN(p1.x, p2.x), MIN(p1.y, p2.y));
	max = cvPoint(MAX(p1.x, p2.x), MAX(p1.y, p2.y));
            
	points = vector<CvPoint>(2);

	if (A == 0)
	{
		points[0].x = min.x;
		points[0].y = (int)(-C / B);
		points[1].x = max.x;
		points[1].y = (int)(-C / B);
	}
	else if (B == 0)
	{
		points[0].x = (int)(-C / A);
		points[0].y = min.y;
		points[1].x = (int)(-C / A);
		points[1].y = max.y;
	}
	else
	{
		vector<CvPoint> lpoints;
		vector<CvPoint> cross;
		cross = vector<CvPoint>(4);
		cross[0].x = min.x;
		cross[0].y = (int)((-C - cross[0].x * A) / B);
		cross[1].x = max.x;
		cross[1].y = (int)((-C - cross[1].x * A) / B);
		cross[2].y = min.y;
		cross[2].x = (int)((-C - cross[2].y * B) / A);
		cross[3].y = max.y;
		cross[3].x = (int)((-C - cross[3].y * B) / A);
		if (cross[0].y >= min.y && cross[0].y <= max.y)
		{
			lpoints.push_back(cross[0]);
		}
		if (cross[1].y >= min.y && cross[1].y <= max.y)
		{
			lpoints.push_back(cross[1]);
		}
		if (cross[2].x >= min.x && cross[2].x <= max.x)
		{
			lpoints.push_back(cross[2]);
		}
		if (cross[3].x >= min.x && cross[3].x <= max.x)
		{
			lpoints.push_back(cross[3]);
		}
		if(lpoints.size()<2)
		{
		   return vector<CvPoint>(0);
		}

		points[0] = lpoints[0];
		for(int i=0; i< lpoints.size();i++)
		{  
			CvPoint p = lpoints[i];
			if(!(p.x == points[0].x && p.y == points[0].y))
			points[1] = p;
		}

	}
	point1 = points[0];
	point2 = points[1];

	/*
	if (B == 0)
	{
		points[0].X = (int)(-C / A);
		points[1].X = (int)(-C / A);
		points[0].Y = p1.Y;
		points[1].Y = p2.Y;
	}
	else
	{
		//a.X = p1.X;
		//a.Y = (int)((-C - A * p1.X) / B);
		//b.X = p1.X;
		//b.Y = (int)((-C - A * p2.X) / B);
	}
	*/
	return points;
}
