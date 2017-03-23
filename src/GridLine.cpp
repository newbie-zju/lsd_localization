#include "lsd_line/GridLine.h"

GridLine::GridLine(Line line)
{
	d = line.d;
	A = line.A;
	B = line.B;
	C = line.C;
	theta = line.theta;
	length = line.length;
	point1 = line.point1;
	point2 = line.point2;

}

GridLine::GridLine()
{
	stability = 0.5;
}

GridLine::~GridLine(void)
{
}

void GridLine::toPre(Line line, bool dir)
{
	if(dir == HORIZONTAL)
	{
		double temp_d1,temp_d2;
		if(line.B * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(B*C >0) temp_d2 = -d;
		else temp_d2 = d;

		d_to_pre = abs(temp_d1*cos(line.theta) - temp_d2*cos(theta));
	}
	else
	{
		double temp_d1,temp_d2;
		if(line.A * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(A*C >0) temp_d2 = -d;
		else temp_d2 = d;

		d_to_pre = abs(temp_d1*sin(line.theta) - temp_d2*sin(theta));
	}
}

void GridLine::toPost(Line line, bool dir)
{
	if(dir == HORIZONTAL)
	{
		double temp_d1,temp_d2;
		if(line.B * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(B*C >0) temp_d2 = -d;
		else temp_d2 = d;

		d_to_post = abs(temp_d1*cos(line.theta) - temp_d2*cos(theta));
	}
	else
	{
		double temp_d1,temp_d2;
		if(line.A * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(A*C >0) temp_d2 = -d;
		else temp_d2 = d;

		d_to_post = abs(temp_d1*sin(line.theta) - temp_d2*sin(theta));
	}
}

void GridLine::toPreNull()
{
	    d_to_pre = -1;
}

void GridLine::toPostNull()
{
	    d_to_post = -1;
}

double GridLine::toGridLine(GridLine line,bool dir)
{
		if(dir == HORIZONTAL)
	{
		double temp_d1,temp_d2;
		if(line.B * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(B*C >0) temp_d2 = -d;
		else temp_d2 = d;

		return abs(temp_d1*cos(line.theta) - temp_d2*cos(theta));
	}
	else
	{
		double temp_d1,temp_d2;
		if(line.A * line.C >0) temp_d1 = -line.d;
		else temp_d1 = line.d;

		if(A*C >0) temp_d2 = -d;
		else temp_d2 = d;

		return abs(temp_d1*sin(line.theta) - temp_d2*sin(theta));
	}
}

double GridLine::GetToPost()
{
	return d_to_post;
}

double GridLine::GetToPre()
{
    return d_to_pre;
}

double GridLine::toPoint(CvPoint p)
{
	double d = abs((A*p.x + B*p.y + C) / sqrt(A*A+B*B));
	return d;
}
