#include "../potrace/PotraceHandler.h"

#include "../potrace/bitmap.h"
#include "../potrace/hj_modifying_graphics_public.h"

PotraceHandler::PotraceHandler()
{
}

vector<Point> removeDuplicatePoint(const vector<Point>& points)
{
	vector<Point> result;
	result.push_back(points.front());
	for (int i = 1; i < points.size(); ++i) {
		if (result.back() != points[i]) {
			result.push_back(points[i]);
		}
	}
	return result;
}

void PotraceHandler::potrace(vector<bool>& bin_image, int w, int h, string output, int x_point, int y_point, int Width, int Height)
{
	vector<vector<Point>> paths;
	potrace(bin_image, w, h, paths);
	writeSvg(output, paths,Width,Height, x_point,y_point);
}

void PotraceHandler::potrace(vector<bool>& bin_image, int w, int h, vector<vector<Point>>& path)
{
	//
	//bitmap
	//
	potrace_bitmap_t* bm = bmNew(bin_image, w, h);
	//
	//parameters
	//
	potrace_param_t* param = potrace_param_default();
	param->turdsize = 0;
	//
	//potrace
	//
	potrace_state_t* st = potrace_trace(param, bm);
	if (!st || st->status != POTRACE_STATUS_OK) {
		fprintf(stderr, "Error tracing bitmap: %s\n", strerror(errno));
		return;
	}
	bmFree(bm);
	//
	//draw each curve
	//
	path.clear();
	Point pt;
	vector<Point> t;
	potrace_path_t* p = st->plist;
	//const float OFF = -1.5;
	while (p)
	{
		int n = p->curve.n;
		int* tag = p->curve.tag;
		auto* c = p->curve.c;
		t.clear();
		t.push_back(Point(float(c[n - 1][2].x), float(c[n - 1][2].y)));
		for (int i = 0; i < n; i++)
		{
			switch (tag[i])
			{
			case POTRACE_CORNER:
				t.push_back(Point(float(c[i][1].x), float(c[i][1].y)));
				t.push_back(Point(float(c[i][2].x), float(c[i][2].y)));
				break;
			case POTRACE_CURVETO:
				pt = t.back();
				add_bezier_points(pt,
					Point(float(c[i][0].x), float(c[i][0].y)),
					Point(float(c[i][1].x), float(c[i][1].y)),
					Point(float(c[i][2].x), float(c[i][2].y)), t);
				break;
			}
		}
		if (t.size() > 1) {
			t = removeDuplicatePoint(t);
		}
		if (t.size() > 1 && (t.front() == t.back()))
			path.push_back(t);
		p = p->next;
	}
	//
	//free
	//
	potrace_state_free(st);
	potrace_param_free(param);
}

void PotraceHandler::writeSvg(string file, const vector<vector<Point>>& path,int w, int h, int x_point, int y_point)
{
	write_path_data_to_svg(vector<vector<Poly>>{path}, file,w,h,x_point, y_point);
}

potrace_bitmap_t* PotraceHandler::bmNew(vector<bool>& data, int w, int h)
{
	int dy = (w + BM_WORDBITS - 1) / BM_WORDBITS;
	potrace_bitmap_t* bm = (potrace_bitmap_t*)malloc(sizeof(potrace_bitmap_t));
	if (!bm) return NULL;
	bm->w = w;
	bm->h = h;
	bm->dy = dy;
	bm->map = (potrace_word*)calloc(h, dy * BM_WORDSIZE);
	if (!bm->map) {
		free(bm);
		return NULL;
	}
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
			BM_PUT(bm, x, y, data[y * w + x] ? 1 : 0);
	return bm;
}

void PotraceHandler::bmFree(potrace_bitmap_t* bm)
{
	if (bm != NULL) {
		free(bm->map);
	}
	free(bm);
}

void PotraceHandler::add_bezier_points(const Point& pt0, const Point& pt1, const Point& pt2, const Point& pt3, vector<Point>& dst)
{
	const int DIV_NUM = 8;
	static const double B0[DIV_NUM] = {
		1.000000000000, 0.629737609329, 0.364431486880, 0.186588921283,
		0.078717201166, 0.023323615160, 0.002915451895, 0.000000000000 };
	static const double B1[DIV_NUM] = {
		0.000000000000, 0.314868804665, 0.437317784257, 0.419825072886,
		0.314868804665, 0.174927113703, 0.052478134111, 0.000000000000 };
	static const double B2[DIV_NUM] = {
		0.000000000000, 0.052478134111, 0.174927113703, 0.314868804665,
		0.419825072886, 0.437317784257, 0.314868804665, 0.000000000000 };
	static const double B3[DIV_NUM] = {
		0.000000000000, 0.002915451895, 0.023323615160, 0.078717201166,
		0.186588921283, 0.364431486880, 0.629737609329, 1.000000000000 };
	//////////////////////////////////////////////////////////////////////////////////////
	Point pt;
	for (int j = 0; j < DIV_NUM; j++) //- 1/*最后一点与下条线段的第一点冗余*/
	{
		pt.x = float(pt0.x * B0[j] + pt1.x * B1[j] + pt2.x * B2[j] + pt3.x * B3[j]);
		pt.y = float(pt0.y * B0[j] + pt1.y * B1[j] + pt2.y * B2[j] + pt3.y * B3[j]);
		dst.push_back(pt);
	}
}
