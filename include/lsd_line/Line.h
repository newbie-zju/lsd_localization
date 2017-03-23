#ifndef Line_H_
#define Line_H_

#include <opencv2/opencv.hpp>
#include <list>
#include <vector>
#include "cmath"

using namespace std;

class Line
{
public:
	    double A,B,C;
        double theta,b;
        double d;
		double length;
		CvPoint point1;
        CvPoint point2;

public:
	    Line();
		Line(CvPoint p1, CvPoint p2);
        Line(double angle, double distance, CvPoint near);
        ~Line();
        double Get_theta();
        double Get_distance();
        CvPoint p1();
        CvPoint p2();
        vector<CvPoint> Get_boundary();
		vector<CvPoint> Get_boundary(CvPoint p1, CvPoint p2);

public:

};
#endif
