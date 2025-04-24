#ifndef HJ_MODIFYING_GRAPHICS_PUBLIC
#define HJ_MODIFYING_GRAPHICS_PUBLIC

#include "../utils/nanosvg.h"
#include "../utils/file_io.h"
#include "hj_mg_datas.h"

#include <string>
#include <vector>
#include <utility>
#include <iomanip>
#include <iostream>
using namespace std;

namespace hj_mgs_implement_edge
{
	static string errorFilePath;
	//-------------------------------------------------------------------------------------------------
	//svg_to_paths
	//-------------------------------------------------------------------------------------------------
	inline void pts_to_path(float* a, int npts, vector<Point>& path)
	{
		int pt_num = ((npts + 2) / 3 - 1) * 3 + 1;//合法点数3n+1
		path.clear();
		if (pt_num >= 4)
		{
			path.resize(pt_num);
			for (int i = 0; i < pt_num; i++)
				path[i] = Point(a[i * 2], a[i * 2 + 1]);
		}
	}
	inline void read_svg(const string& svg_file, vector<vector<Point>>& dst)
	{
		dst.clear();
		vector<Point> t_path;
		auto* nv_image = nsvgParseFromFile((char*)(svg_file.c_str()), "px", 100.0f);
		for (auto* shape = nv_image->shapes; shape != NULL; shape = shape->next)
		{
			for (auto* path = shape->paths; path != NULL; path = path->next)
			{
				pts_to_path(path->pts, path->npts, t_path);
				if (t_path.size() >= 4)
					dst.push_back(t_path);
			}
		}
		nsvgDelete(nv_image);
	}
	inline void read_svg(const string& svg_file, vector<vector<vector<Point>>>& dst)
	{
		vector<Point> t_path;
		dst.clear();
		auto* nv_image = nsvgParseFromFile((char*)(svg_file.c_str()), "px", 100.0f);
		int pos = 0;
		for (auto* shape = nv_image->shapes; shape != NULL; shape = shape->next)
		{
			dst.resize(pos + 1);
			for (auto* path = shape->paths; path != NULL; path = path->next)
			{
				pts_to_path(path->pts, path->npts, t_path);
				if (t_path.size() >= 4)
					dst[pos].push_back(t_path);
			}
		}
		nsvgDelete(nv_image);
	}
	//-------------------------------------------------------------------------------------------------
	//subdivide_bezier
	//-------------------------------------------------------------------------------------------------
	inline bool is_one_line(const Point& pt1, const Point& pt2, const Point& pt3, const Point& pt4)
	{
		const float D = 0.01f;
		float dx = pt4.x - pt1.x;
		float dy = pt4.y - pt1.y;
		if (fabs(dx) < D && fabs(dy) < D) return false;
		if (fabs(dx) > fabs(dy)) //以x为准
		{
			float y2 = pt1.y + (pt2.x - pt1.x) * dy / dx;
			float y3 = pt1.y + (pt3.x - pt1.x) * dy / dx;
			return (fabs(pt2.y - y2) < D && fabs(pt3.y - y3) < D);
		}
		else //以y为准
		{
			float x2 = pt1.x + (pt2.y - pt1.y) * dx / dy;
			float x3 = pt1.x + (pt3.y - pt1.y) * dx / dy;
			return (fabs(pt2.x - x2) < D && fabs(pt3.x - x3) < D);
		}
		return false;
	}
	inline bool same_pt(const Point& pt1, const Point& pt2)
	{
		const float EP = 0.101f;
		return fabs(pt1.x - pt2.x) < EP && fabs(pt1.y - pt2.y) < EP;
	}
	inline bool is_one_point(const vector<Point>& path)
	{
		for (int i = 1; i<int(path.size()); i++)
			if (!same_pt(path[0], path[i])) return false;
		return true;
	}
	inline void get_bezier_pt(const Point& c1, const Point& c2, const Point& c3, const Point& c4, double t, Point& pt)
	{
		double v = 1.0 - t;
		double B1 = v * v * v;
		double B2 = 3.0 * v * v * t;
		double B3 = 3.0 * v * t * t;
		double B4 = t * t * t;
		pt.x = float(B1 * c1.x + B2 * c2.x + B3 * c3.x + B4 * c4.x);
		pt.y = float(B1 * c1.y + B2 * c2.y + B3 * c3.y + B4 * c4.y);
	}
	inline void subdivide_bezier(const vector<Point>& bezier, const int& SUB_NUM, const float& DELTA, vector<Point>& dst)
	{
		if (bezier.size() < 4) return;
		int pt_num = int(bezier.size());//合法点数3n+1
		dst.reserve(pt_num * SUB_NUM / 3);
		dst.clear();
		dst.push_back(bezier[0]);
		Point pre = bezier[0], pt;
		for (int i = 0; i < pt_num - 3; i += 3)
		{
			const Point& pt0 = bezier[i];
			const Point& pt1 = bezier[i + 1];
			const Point& pt2 = bezier[i + 2];
			const Point& pt3 = bezier[i + 3];
			if (is_one_line(pt0, pt1, pt2, pt3))
			{
				dst.push_back(pt3);
			}
			else
			{
				for (int j = 0; j <= SUB_NUM; j++)
				{
					double t = double(j) / double(SUB_NUM);
					get_bezier_pt(pt0, pt1, pt2, pt3, t, pt);
					if (j == SUB_NUM || fabs(pre.x - pt.x) > DELTA || fabs(pre.y - pt.y) > DELTA)
					{
						dst.push_back(pt);
						pre = pt;
					}
				}
			}
		}
	}
	inline void subdivide(const vector<vector<Point>>& bezier, const int& SUB_NUM, const float& DELTA, vector<vector<Point>>& dst)
	{
		int path_num = int(bezier.size());
		dst.clear();
		dst.resize(path_num);
		for (int i = 0; i < path_num; i++)
			subdivide_bezier(bezier[i], SUB_NUM, DELTA, dst[i]);
	}
	inline void subdivide(const vector<vector<vector<Point>>>& bezier, const int& SUB_NUM, const float& DELTA, vector<vector<vector<Point>>>& dst)
	{
		int path_num = int(bezier.size());
		dst.clear();
		dst.resize(path_num);
		for (int i = 0; i < path_num; i++)
			subdivide(bezier[i], SUB_NUM, DELTA, dst[i]);
	}
	//-------------------------------------------------------------------------------------------------
	//bezier_path_to_svg
	//-------------------------------------------------------------------------------------------------
	inline float to_degree(float radian)
	{
		return (radian * 180.0f / 3.1415926f);
	}
	inline float to_angle(const float& to_x, const float& to_y)
	{
		if (fabs(to_x) < 0.001f && fabs(to_y) < 0.001f) return 0.0f;
		float angle = 0.0f;
		float d = float(sqrt(to_x * to_x + to_y * to_y));
		float proto_angle = to_degree(asinf((float)fabs(to_y) / d));
		if (to_y >= 0.0f)
		{
			if (to_x >= 0.0f) angle = proto_angle;          //第一象限
			else              angle = 180.0f - proto_angle; //第二象限
		}
		else
		{
			if (to_x < 0.0f)  angle = 180.0f + proto_angle; //第三象限
			else              angle = 360.0f - proto_angle; //第四象限
		}
		return angle;
	}
	inline bool angle_equal(const float& angle1, const float& angle2)
	{
		float d = (float)fabs(angle1 - angle2);
		if (d < 0.01f || fabs(d - 360.0f) < 0.01f)
			return true;
		return false;
	}
	inline void pend_bezier_path_to_string(const vector<Point>& path, string& dst)
	{
		static char s[64];
		int pt_num = path.size();
		if (pt_num >= 2)
		{
			s[0] = '\0';
			//起点
			sprintf(s, "M %.4f %.4f ", path[0].x, path[0].y);
			dst += s;
			for (int i = 1; i < pt_num - 2; i += 3)
			{
				if (is_one_line(path[i - 1], path[i], path[i + 1], path[i + 2]))
				{
					//直线
					sprintf(s, "L %.4f %.4f ", path[i + 2].x, path[i + 2].y);
					dst += s;
				}
				else
				{
					//贝塞尔曲线
					sprintf(s, "C %.4f %.4f %.4f %.4f %.4f %.4f ", path[i].x, path[i].y, path[i + 1].x, path[i + 1].y, path[i + 2].x, path[i + 2].y);
					dst += s;
				}
			}
		}
		//dst += "Z";
	}
	inline void bezier_paths_to_string(const vector<vector<Point>>& path, string& dst)
	{
		int pt_num = 0;
		int path_num = path.size();
		for (int i = 0; i < path_num; i++)
			pt_num += int(path[i].size());
		dst.reserve(pt_num * 26);
		dst.clear();
		for (int i = 0; i < path_num; i++)
			pend_bezier_path_to_string(path[i], dst);
	}
	inline void write_svg_bezier_path_string(const vector<vector<vector<Point>>>& path, string& path_str)
	{
		//预先申请空间
		//int pt_num = 0;
		int path_num = int(path.size());
		//for (int i = 0; i < path_num; i++)
		//	pt_num += int(path[i].size());
		//path_str.reserve(pt_num * 32);
		//逐个路径添加
		string t;
		path_str.clear();
		string head = "<path d=\"";
		string tail = "\" style=\"fill:none;stroke:black\" />\n";
		//printf("\npath_num:%d------------------------------------\n", path_num);
		for (int i = 0; i < path_num; i++)
		{
			bezier_paths_to_string(path[i], t);
			path_str += head;
			path_str += t;
			path_str += tail;
		}
	}
	inline void write_bezier_data_to_svg(const vector<vector<vector<Point>>>& path, string svg_file)
	{
		string path_str;
		write_svg_bezier_path_string(path, path_str);

		string head = "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" xml:space=\"preserve\">\n";
		string tail = "</svg>";
		string svg_str = head + path_str + tail;
		file_io::write_string_to_file(svg_str, svg_file);
	}
	//-------------------------------------------------------------------------------------------------
	//path_to_svg
	//-------------------------------------------------------------------------------------------------
	inline void pend_path_to_string(const vector<Point>& path, string& dst, int x_point, int y_point)
	{
		static char s[64];
		int pt_num = path.size();
		if (pt_num >= 2)
		{
			s[0] = '\0';
			//起点
			sprintf(s, "M %.4f %.4f ", path[0].x + x_point, path[0].y + y_point);
			dst += s;
			for (int i = 1; i < pt_num; i++)
			{
				//直线
				sprintf(s, "L %.4f %.4f ", path[i].x + x_point, path[i].y + y_point);
				dst += s;
			}
		}
		//dst += "Z";
	}
	inline void pend_paths_to_string(const vector<vector<Point>>& path, string& dst, int x_point, int y_point)
	{
		int path_num = path.size();
		for (int i = 0; i < path_num; i++)
			pend_path_to_string(path[i], dst, x_point, y_point);
	}
	inline void pend_pathss_to_string(const vector<vector<vector<Point>>>& path, string& path_str,int x_point, int y_point)
	{
		//预先申请空间
		//int pt_num = 0;
		int path_num = int(path.size());
		//for (int i = 0; i < path_num; i++)
		//	pt_num += int(path[i].size());
		//path_str.reserve(pt_num * 32);
		//逐个路径添加
		string t;
		path_str.clear();
		string head = "<path d=\"";
		string tail = "\" style=\"fill:none;stroke:black\" />\n";
		//printf("\npath_num:%d------------------------------------\n", path_num);
		for (int i = 0; i < path_num; i++)
		{
			pend_paths_to_string(path[i], t,x_point,y_point);
			path_str += head;
			path_str += t;
		}
		path_str += tail;
	}
	inline void write_path_data_to_svg(const vector<vector<vector<Point>>>& path, string svg_file, int w, int h,int x_point, int y_point)
	{
		string path_str;
		pend_pathss_to_string(path, path_str, x_point,y_point);

		string head = "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" x=\"0px\" y=\"0px\" xml:space=\"preserve\" width=\"" + std::to_string(w) + "px\" height=\"" + std::to_string(h) + "px\">\n";
		string tail = "</svg>";
		string svg_str = head + path_str + tail;
		file_io::write_string_to_file(svg_str, svg_file);
	}

	//-------------------------------------------------------------------------------------------------
	//json
	//-------------------------------------------------------------------------------------------------
	inline bool read_json(string json_file, nlohmann::json& root)
	{
		std::ifstream jsonFile(json_file, std::ifstream::binary);
		if (!jsonFile)
		{
			printf("Cannot find %s!\n", json_file.c_str());
			return false;
		}

		std::stringstream buffer;
		buffer << jsonFile.rdbuf();
		root = nlohmann::json::parse(buffer.str());
		jsonFile.close();
		return true;
	}
	inline bool write_json_file(const nlohmann::json& output, string dst_file)
	{
		std::ofstream file(dst_file);
		if (file.is_open()) {
			file << std::setw(4) << output << std::endl;
			file.close();
			//std::cout << "JSON data has been written to file." << std::endl;
			return true;
		}
		else {
			std::cerr << "w_path_process Failed to open file for writing." << std::endl;
			return false;
		}
	}
	inline nlohmann::json pathToJson(const vector<Poly>& paths)
	{
		nlohmann::json root = nlohmann::json::object();
		nlohmann::json ploys = nlohmann::json::array();
		for (size_t i = 0; i < paths.size(); i++)
		{
			nlohmann::json array = nlohmann::json::array();
			for (size_t j = 0; j < paths[i].size(); j++)
			{
				nlohmann::json pt = nlohmann::json::object();
				pt["x"] = paths[i][j].x;
				pt["y"] = paths[i][j].y;
				array.push_back(pt);
			}
			ploys.push_back(array);
		}
		root["polys"] = ploys;
		return root;
	}
	inline nlohmann::json linesToJson(const vector<Line>& lines)
	{
		nlohmann::json root = nlohmann::json::object();
		nlohmann::json ls = nlohmann::json::array();
		for (size_t i = 0; i < lines.size(); i++)
		{
			nlohmann::json l = nlohmann::json::object();
			nlohmann::json s = nlohmann::json::object();
			s["x"] = lines[i].sp.x;
			s["y"] = lines[i].sp.y;
			nlohmann::json e = nlohmann::json::object();
			e["x"] = lines[i].ep.x;
			e["y"] = lines[i].ep.y;
			l["start"] = s;
			l["end"] = e;
			ls.push_back(l);
		}
		root["lines"] = ls;
		return root;
	}
	template <class T>
	static bool read_json_data(const nlohmann::json& o, T& data)
	{
		if (o.is_null()) return false;
		try {
			data = o.get<T>();
			return true;
		}
		catch (const std::exception& e)
		{
			printf("%s Exception!!!\n", e.what());
			return false;
		}
	}
	static nlohmann::json read_json_ele(const nlohmann::json& o, string key, bool &ok)
	{
		nlohmann::json ele;
		try {
			ele = o.at(key);
			ok = true;
		}
		catch (const std::exception& e) {
			printf("%s Exception!!!\n", e.what());
			ok = false;
		}
		return ele;
	}

	inline void check(bool ok) {
		if (!ok) {
			return;
		}
	}

	template<typename T>
	static bool vector_contain(const vector<T>& vec, T val)
	{
		return std::find(vec.begin(), vec.end(), val) != vec.end();
	}

	template<typename T>
	static void vector_append(vector<T>& vec, const vector<T>& val)
	{
		for (size_t i = 0; i < val.size(); i++)
		{
			vec.push_back(val[i]);
		}
	}

	template <typename T>
	int vector_indxOf(const std::vector<T>& d, const T& t)
	{
		auto b = d.begin();
		auto e = d.end();
		while (b != e) {
			if (*b == t)
				return b - d.begin();
			b++;
		}
		return -1;
	}
}

#endif