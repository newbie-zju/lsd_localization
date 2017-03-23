#ifndef TINYROBOTS_H_
#define TINYROBOTS_H_
#include <vector>
#include <opencv2/opencv.hpp>
#include "lsd_line/robot_unit.h"

using namespace std;
using namespace cv;

class TinyRobots
{
public:
	vector<robot_unit> current;
	vector<robot_unit> last;

public:
	TinyRobots(void);
	~TinyRobots(void);

public:
	void FindCurrentCoordinates(IplImage *res_im);
	int FindRobot(Point2d pos);
	vector<robot_unit>  GetImgPos();


};
#endif
