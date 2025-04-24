#define _CRT_SECURE_NO_WARNINGS
#ifndef FILE_IO_H
#define FILE_IO_H

#include "../utils/json.hpp"
#include "../utils/stb_image.h"
#include "../utils/stb_image_write.h"

#include <string>
#include <fstream>
#include <sstream>
#include <string.h>
#include <iostream>
using namespace std;

namespace file_io
{
	//--------------------------------------------------------------------------------------------------
	//读取json
	//--------------------------------------------------------------------------------------------------
	//将文件读取为json对象
	inline bool parse_json_file(string filename, nlohmann::json& json)
	{
		std::ifstream jsonFile(filename, std::ifstream::binary);
		if (!jsonFile)
		{
			printf("Cannot find %s!\n", filename.c_str());
			return false;
		}

		std::stringstream buffer;
		buffer << jsonFile.rdbuf();
		json = nlohmann::json::parse(buffer.str());
		return true;
	}
	//检查是否存在某个item
	inline bool check_item(const nlohmann::json& json, const string& item_name)
	{
		bool b = (json.find(item_name) != json.end());
		return b;
	}
	//读取对象数据
	template <class T>
	inline bool read_json_data(const nlohmann::json& o, T& data)
	{
		if (o.is_null())
			return false;
		try
		{
			data = o.get<T>();
			return true;
		}
		catch (const std::exception& e)
		{
			printf("%s Exception!!!\n", e.what());
			return false;
		}
	}
	//尝试读取一个item
	template <class T>
	inline bool read_json_item(const nlohmann::json& json, const string& item_name, T& data)
	{
		if (check_item(json, item_name) && read_json_data(json[item_name], data))
			return true;

		printf("read json missing %s!\n", item_name.c_str());
		return false;

	}
	//--------------------------------------------------------------------------------------------------
	//读取image
	//--------------------------------------------------------------------------------------------------
	inline bool read_image_RGBA(string filename, vector<uint8_t> &rgba, int &w, int &h)
	{
		int channels = 0;
		uint8_t *t =  stbi_load(filename.c_str(), &w, &h, &channels, 4);
		if (!t) return false;

		int n = w * h * 4;
		rgba.resize(n);
		for (int i = 0; i < n; i++)
			rgba[i] = t[i];
		stbi_image_free(t);
		return true;
	}
	inline bool read_image_RGB(string filename, vector<uint8_t>& rgb, int& w, int& h)
	{
		int channels = 0;
		uint8_t* t = stbi_load(filename.c_str(), &w, &h, &channels, 3);
		if (!t) return false;

		int n = w * h * 3;
		rgb.resize(n);
		for (int i = 0; i < n; i++)
			rgb[i] = t[i];
		stbi_image_free(t);
		return true;
	}
	inline void save_png(const uint8_t rgba[], int w, int h, string filename)
	{
		stbi_write_png(filename.c_str(), w, h, 4, rgba, w * 4);
	}
	inline void save_png_GRAY(const uint8_t I[], int w, int h, string filename)
	{
		stbi_write_png(filename.c_str(), w, h, 1, I, w * 1);
	}
	inline void save_png_RGB(const uint8_t rgb[], int w, int h, string filename)
	{
		stbi_write_png(filename.c_str(), w, h, 3, rgb, w * 3);
	}
	inline void save_png(const vector<uint8_t> &rgba, int w, int h, string filename)
	{
		stbi_write_png(filename.c_str(), w, h, 4, &rgba[0], w * 4);
	}
	inline void save_png_RGB(const vector<uint8_t> &rgb, int w, int h, string filename)
	{
		stbi_write_png(filename.c_str(), w, h, 3, &rgb[0], w * 3);
	}
	//--------------------------------------------------------------------------------------------------
	//其他读取
	//--------------------------------------------------------------------------------------------------
	inline string read_file_to_string(string filename)
	{
		ifstream in(filename, ios::in);
		istreambuf_iterator<char> beg(in), end;
		string s(beg, end);
		in.close();
		return s;
	}
	inline bool write_string_to_file(string str, string filename)
	{
		FILE *fp = fopen(filename.c_str(), "w");
		fwrite(str.c_str(), 1, str.length(), fp);
		fclose(fp);
		return false;
	}
}


#endif