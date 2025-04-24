//#define STB_IMAGE_WRITE_IMPLEMENTATION
//#include "../utils/stb_image_write.h"
//#define STB_IMAGE_IMPLEMENTATION
//#include "../utils/stb_image.h"
//#define NANOSVG_IMPLEMENTATION
//#include "../utils/nanosvg.h"
//
//#include "../potrace/PotraceHandler.h"
//
//#include<opencv2/opencv.hpp>
//#include<iostream>
//#include <string>
#include "../include/recognition.h"

using namespace cv;
using namespace std;

//static int width, height;

//void readPhoto(const std::string& filename, std::vector<bool>& pixels, int rd) {
//    int channels;
//    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
//    if (!data) {
//        std::cerr << "Failed to load image: " << filename << std::endl;
//        return;
//    }
//
//    pixels.resize(width * height);
//    // 二值化处理
//    long sum = 0;
//    for (int y = 0; y < height; ++y) {
//        for (int x = 0; x < width; ++x) {
//            unsigned char r = data[(y * width + x) * channels];
//            unsigned char g = data[(y * width + x) * channels + 1];
//            unsigned char b = data[(y * width + x) * channels + 2];
//
//            unsigned char gray = 0.2989 * r + 0.5870 * g + 0.1140 * b;
//            pixels[y * width + x] = (gray < rd);
//        }
//    }
//
//    stbi_image_free(data);
//}

int main() {

    //String outputStr = "C:/Users/mk12878/Desktop/20250321-150841.jpg";
    //std::vector<bool> imageData;
    //readPhoto(outputStr, imageData, 50);
    //
    //using namespace hj_mgs_implement;
    //PotraceHandler h;
    //outputStr = "C:/Users/mk12878/Desktop/output.svg";
    //h.potrace(imageData, width, height, outputStr);
    std::string path = "C:/Users/mk12878/Desktop/input_edge.json";

    ContourRecognition::imageRecognition(path);

	return 0;
}