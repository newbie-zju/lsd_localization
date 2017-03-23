#ifndef LineGroup_H_
#define LineGroup_H_

#include <vector>
#include "lsd_line/Line.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>

extern "C"
{
#include "lsd_line/lsd.h"
};

#define Pi 3.1415926

using namespace cv;
class LineGroup
{
public:
	vector<Line> line_fusion;
    vector< vector<Line> > line_groups;
	bool show_info;

public:
	LineGroup(void);
	~LineGroup(void);

public:
	double length_threshold;
	double group_size_threshold;
	void Categorize();
	//void GetRawLines_(vector<Vec4i> detected_lines);
	void GetRawLines(int dim, ntuple_list detected_lines);
	void GetRawLines(vector<Vec4i> raw_points);
	void Fusion_average();
	void Draw2Image(IplImage* res_im);
	void DrawRawLines2Image(IplImage* res_im);
	void ShowInfo(bool show);
	vector<Line> GetLineFusion();
};
#endif
