#ifndef GridLine_H_
#define GridLine_H_

#include "lsd_line/Line.h"

#define HORIZONTAL 0
#define VERTICAL 1

class GridLine :
	public Line
{
public:
	double threshold;
	double d_to_pre;
	double d_to_post;
	int world_lable;
	int offset;
	double confidence_level;
	double stability;


public:
	GridLine();
	GridLine(Line line);
	~GridLine(void);
	void toPre(Line line,bool dir);
	void toPost(Line line,bool dir);
	void toPreNull();
	void toPostNull();
	double GetToPost();
	double GetToPre();
	double toGridLine(GridLine line,bool dir);
	double toPoint(CvPoint p);
};

#endif
