#include "lsd_line/square.h"

square::square(void)
{
	area = 0;
	id = 0;
	x_pos = 0;
	y_pos = 0;
	for(int i=0; i<4; i++)
	{
	    for(int j=0;j<3;j++)
		{
			line[i][j] = 0;
		}
	}

	for(int i=0; i<4; i++)
	{
	    for(int j=0;j<2;j++)
		{
			vertex[i][j] = 0;
		}
	}
}

square::square(CvPoint2D64f px1,CvPoint2D64f px2,CvPoint2D64f px3,CvPoint2D64f px4,int count,double re,int pos_x,int pos_y,int set_scale)
{
	p1 = px1;
	p2 = px2;
	p3 = px3;
	p4 = px4;
	id = count;
	reliability = re;
	x_pos = pos_x;
	y_pos = pos_y;
	scale = set_scale;
}

square::~square(void)
{
}
