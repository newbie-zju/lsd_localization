//#pragma once
#ifndef ROBOT_UNIT_H_
#define ROBOT_UNIT_H_
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

class robot_unit
{
public:
	
	Point2i img_pos;
	Point2d real_pos;
	int id;
	bool active;
	int life;

public:
	robot_unit(void);
	robot_unit(Point2i imgpos,vector<robot_unit> last);
	~robot_unit(void);
	double FromImgPoint(Point2d point);
	void Found(void);
	void Update(Point2i imgpos);
	void lost();
	void SetRealXY(Point2d p);
};
#endif
