//#pragma once     //compile once     #ifndef *** #define *** #endif
#ifndef Classifier_H_
#define Classifier_H_

#include <vector>
#include "lsd_line/Line.h"
#include <cstdio>

using namespace std;

class Classifier
{
public:

	vector<Line> raw_lines;
	vector<Line> horizontal;
	vector<Line> vertical;
	double horizontal_average_theta;
	double vertical_average_theta;
	double horizontal_last_average_theta;
	double vertical_last_average_theta;
	bool show_info;

public:
	Classifier(void);
	~Classifier(void);

public:
	void classify(vector<Line> line_fusion);
	void Draw2Image(IplImage* res_im);
	void ShowInfo(bool show);
	vector<Line> GetHorizontalLines();
	vector<Line> GetVerticalLines();
};

#endif
