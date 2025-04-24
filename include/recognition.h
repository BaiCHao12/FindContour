#ifndef RECOGNITION_H
#define RECOGNITION_H

//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "../utils/stb_image_write.h"
//#define STB_IMAGE_IMPLEMENTATION
//#include "../utils/stb_image.h"
//#define NANOSVG_IMPLEMENTATION
//#include "../utils/nanosvg.h"

#include "../potrace/PotraceHandler.h"
#include "edgeTest.h"
#include "WriteFile.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "json.hpp"
#include <vector>
#include <time.h>
#include <chrono>
#include <algorithm>

using namespace hj_mgs_implement_edge;


namespace ContourRecognition {

using json = nlohmann::json;
void imageRecognition(const std::string& json_path);
void hsvLaplaceSharpen(const cv::Mat& img, cv::Mat& enhancedBgr, float intensity=0.5);
void perfect_reflective_white_balance(const cv::Mat& img, cv::Mat& result, int percentile = 95);
void contrast_stretching(cv::Mat& srcImage, cv::Mat& result);

void enchance_image(const cv::Mat& image, cv::Mat& result, float saturation_factor = 1.5);
void processContours(const std::string& imagePath, const std::string& outputPath,int filter_value, bool only_external,int x,int y,int x2,int y2, int low_pass_size = 40, int high_pass_thresh = 40);
std::vector<std::vector<cv::Point>> remain_contours(std::vector<std::vector<cv::Point>> contours, cv::Mat image);
void processContoursHR(const std::string& imagePath, const std::string& outputPath, int filter_value, bool only_external, int x, int y, int x2, int y2, int low_pass_size = 40, int high_pass_thresh = 40);

void saveContoursToSVG(const std::string& filename, const std::vector<std::vector<cv::Point>>& contours, int width, int height);
void DetectExternal(const cv::Mat& edges, const cv::Mat& gray_image, const std::string& outputPath,int x_point,int y_point,int Width,int Height);
void DetectExternalInner(const cv::Mat& gray_image, const std::string& outputPath, int low_pass_size, int high_pass_thresh,int x_point,int y_point, int Width, int Height);
}

#endif // !RECOGNITION_H
