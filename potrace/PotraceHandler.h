#pragma once

#include "potracelib.h"
#include "hj_mg_datas.h"
using namespace std;
using namespace hj_mgs_implement_edge;

class PotraceHandler
{
public:
	PotraceHandler();
	~PotraceHandler() = default;

	void potrace(vector<bool>& bin_image, int w, int h, string output, int x_point, int y_point, int Width, int Height);

	void potrace(vector<bool>& bin_image, int w, int h, vector<vector<Point>>& path);
	void writeSvg(string file, const vector<vector<Point>>& path, int w, int h, int x_point, int y_point);

private:
	potrace_bitmap_t* bmNew(vector<bool>& data, int w, int h);
	void bmFree(potrace_bitmap_t* bm);
	void add_bezier_points(const Point& pt0, const Point& pt1, const Point& pt2, const Point& pt3, vector<Point>& dst);
};

