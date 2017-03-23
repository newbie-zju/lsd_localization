#include "lsd_line/robot_unit.h"


robot_unit::robot_unit(void)
{
	img_pos = Point2i(0,0);
	real_pos = Point2d(0,0);
	id = 0;
	active = false;
	life = 0;
}

robot_unit::robot_unit(Point2i imgpos,vector<robot_unit> last)
{
	img_pos = imgpos;
	real_pos = Point2d(0,0);
	bool same_id;

	do
	{
		same_id = false;
		id = rand()%1000;
		for(int i=0;i<last.size();i++)
		{
			if(last.at(i).id==id)
			{
				same_id = true;
			}
		}
	}while(same_id);
	
	active = true;
	life = 15;
}

robot_unit::~robot_unit(void)
{
}

double robot_unit::FromImgPoint(Point2d point)
{
	double d = sqrt((point.x - img_pos.x)*(point.x - img_pos.x) + (point.y - img_pos.y)*(point.y - img_pos.y));
	return d;
}

void robot_unit::Found()
{
    active = true;
}

void robot_unit::Update(Point2i imgpos)
{
	img_pos = imgpos;
	active = true;
	life = 15;
}

void robot_unit::lost()
{
	active = false;
}

void robot_unit::SetRealXY(Point2d p)
{
	real_pos = p;
}
