#ifndef square_H_
#define square_H_

#include <opencv2/opencv.hpp>

class square
{
public:
	CvPoint2D64f p1,p2,p3,p4;
	int area;
	int id;
	int x_pos,y_pos;
	int scale;
	double line[4][3];
	double vertex[4][2];
	double reliability;


public:
	square(void);
	~square(void);
	square(CvPoint2D64f px1,CvPoint2D64f px2,CvPoint2D64f px3,CvPoint2D64f px4,
		   int count, double re,
		   int pos_x,int pos_y,
		   int set_scale);
};

#endif
