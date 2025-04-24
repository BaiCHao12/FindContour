#include "../include/recognition.h"
namespace ContourRecognition
{
    using json = nlohmann::json;


    void enchance_image(const cv::Mat& image,cv::Mat &result,float saturation_factor) {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv, hsv_channels);
        cv::Mat& h = hsv_channels[0];
        cv::Mat& s = hsv_channels[1];
        cv::Mat& v = hsv_channels[2];

        s *= saturation_factor;
        cv::min(s, 255, s);
        cv::merge(hsv_channels, hsv);
        cv::cvtColor(hsv, result, cv::COLOR_HSV2BGR);
    }

    void hsvLaplaceSharpen(const cv::Mat& img, cv::Mat& enhancedBgr, float intensity) {
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsvChannels;
        cv::split(hsv, hsvChannels);
        cv::Mat& v = hsvChannels[2];

        cv::Mat laplacian;
        cv::Laplacian(v, laplacian, CV_32F);

        cv::Mat vSharp;
        cv::addWeighted(v, 1.0, laplacian, intensity, 0, vSharp, CV_32F);

        cv::Mat vSharpClipped;
        cv::convertScaleAbs(vSharp, vSharpClipped);

        hsvChannels[2] = vSharpClipped;
        cv::merge(hsvChannels, hsv);

        cv::cvtColor(hsv, enhancedBgr, cv::COLOR_HSV2BGR);
    }
       
    void perfect_reflective_white_balance(const cv::Mat& img, cv::Mat& result, int percentile) {
      
        cv::Mat img_rgb;
        cv::cvtColor(img, img_rgb, cv::COLOR_BGR2RGB);
        img_rgb.convertTo(img_rgb, CV_32F);

        // 计算每个像素的亮度总和
        cv::Mat brightness;
        cv::reduce(img_rgb, brightness, 2, cv::REDUCE_SUM);

        // 确定亮度阈值（前percentile%）
        double threshold;
        cv::Mat brightness_flat = brightness.reshape(1, brightness.total());
        cv::sort(brightness_flat, brightness_flat, cv::SORT_ASCENDING);
        threshold = brightness_flat.at<float>(static_cast<int>(brightness_flat.total() * percentile / 100.0));

        // 创建掩膜选择高光区域
        cv::Mat mask;
        cv::compare(brightness, threshold, mask, cv::CMP_GE);

        // 计算参考白点（各通道均值）
        std::vector<float> white_point(3, 255.0f);
        for (int channel = 0; channel < 3; ++channel) {
            std::vector<float> channel_values;
            for (int i = 0; i < img_rgb.rows; ++i) {
                for (int j = 0; j < img_rgb.cols; ++j) {
                    if (mask.at<uchar>(i, j)) {
                        channel_values.push_back(img_rgb.at<cv::Vec3f>(i, j)[channel]);
                    }
                }
            }
            if (!channel_values.empty()) {
                white_point[channel] = std::accumulate(channel_values.begin(), channel_values.end(), 0.0f) / channel_values.size();
            }
        }
        // 防止除零，设置最小值
        for (auto& value : white_point) {
            value = std::max(value, 1e-10f);
        }

        // 计算增益（目标255）
        std::vector<float> gains(3);
        for (int i = 0; i < 3; ++i) {
            gains[i] = 255.0f / white_point[i];
        }
        // 应用增益调整图像
        cv::Mat balanced = img_rgb.clone();
        for (int i = 0; i < balanced.rows; ++i) {
            for (int j = 0; j < balanced.cols; ++j) {
                for (int channel = 0; channel < 3; ++channel) {
                    balanced.at<cv::Vec3f>(i, j)[channel] = std::min(balanced.at<cv::Vec3f>(i, j)[channel] * gains[channel], 255.0f);
                }
            }
        }
        balanced.convertTo(balanced, CV_8U);
        cv::cvtColor(balanced, result, cv::COLOR_RGB2BGR);
    }

    void contrast_stretching(cv::Mat& srcImage, cv::Mat& result) {

        std::vector<cv::Mat> channels;
        split(srcImage, channels);

        for (int i = 0; i < channels.size(); ++i) {
            double minVal, maxVal;
            minMaxLoc(channels[i], &minVal, &maxVal);

            if (maxVal > minVal) {
                double scale = 255.0 / (maxVal - minVal);
                double offset = -minVal * scale;

                channels[i].convertTo(channels[i], CV_8U, scale, offset);
            }
        }
        merge(channels, result);
    }

    void saveContoursToSVG(const std::string& filename, const std::vector<std::vector<cv::Point>>& contours, int width, int height) {
        std::ofstream svgFile(filename);

        if (!svgFile.is_open()) {
            std::cerr << "Could not open the file to save SVG!" << std::endl;
            return;
        }

        svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\"" << height << "\">"  << std::endl;


        svgFile << "<path d=\"";
        for (const auto& contour : contours) {
          /*svgFile << "<path d=\"M ";*/
            svgFile << "M ";
            for (size_t i = 0; i < contour.size(); ++i) {
                svgFile << contour[i].x << " " << contour[i].y;
                if (i < contour.size() - 1) {
                    svgFile << " L ";
              }
          }
            svgFile << " Z ";
        }
        svgFile << " Z\" fill=\"none\" stroke=\"black\" stroke-width=\"2\" />" << std::endl;
        svgFile << "</svg>" << std::endl;

        svgFile.close();
    }

    std::vector<std::vector<cv::Point>> remain_contours(std::vector<std::vector<cv::Point>> contours, cv::Mat image) {
        int kernel_size = 5;
        cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::drawContours(mask, contours, -1, 255, 1);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        cv::Mat dilated;
        cv::dilate(mask, dilated, kernel, cv::Point(-1, -1), 1);
        std::vector<std::vector<cv::Point>> new_contours;
        cv::findContours(dilated, new_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return new_contours;
    }

    void processContours(const std::string& imagePath, const std::string& outputPath,int filter_value, bool only_external,int x,int y,int x2,int y2, int low_pass_size, int high_pass_thresh) {
        cv::Mat image = cv::imread(imagePath);
        if (image.empty()) {
            std::cerr << "Could not load image..." << std::endl;
            return;
        }
        cv::Rect roi(x, y, x2-x, y2-y);
        cv::Mat image_roi = image(roi);

        // ******************image edge detect kernel code**************************
        // mthold1: sscannogram edge detect

        auto start = std::chrono::steady_clock::now();
        cv::Mat new_image_roi;
        enchance_image(image_roi, new_image_roi);

        cv::Mat hsv_lap;
        hsvLaplaceSharpen(new_image_roi, hsv_lap);

        cv::Mat cont_img;
        contrast_stretching(hsv_lap, cont_img);

        //cv::Mat pr_img;
        //perfect_reflective_white_balance(cont_img, pr_img);

        cv::Mat fnm_image;
        cv::fastNlMeansDenoisingColored(cont_img, fnm_image, 10, 7, 7, 21);

        cv::Mat gaussian_blur;
        cv::GaussianBlur(fnm_image, gaussian_blur, cv::Size(3, 3), 5);

        cv::Mat bilater_bulr;
        cv::bilateralFilter(gaussian_blur, bilater_bulr, 15, 20, 20);

        cv::Mat msf_sharpened;
        cv::pyrMeanShiftFiltering(bilater_bulr, msf_sharpened, 10, low_pass_size);

        cv::Mat bilater_bulr2;
        cv::bilateralFilter(msf_sharpened, bilater_bulr2, 15, 25, 15);
        
        cv::Mat gaussian_blur2;
        cv::GaussianBlur(bilater_bulr2, gaussian_blur2, cv::Size(5, 5), 0);

        cv::Mat edges;
        int high_pass_thresh_low = std::max(0, (high_pass_thresh - 40));
        cv::imwrite("C:/Users/mk12878/Desktop/111111111.jpg", gaussian_blur2);

        cv::Mat grayImage;
        cv::cvtColor(gaussian_blur2, grayImage, cv::COLOR_BGR2GRAY);

        int Width = image.cols;
        int Height = image.rows;

        if (only_external) {
            cv::Canny(gaussian_blur2, edges, high_pass_thresh_low, high_pass_thresh);
            DetectExternal(edges, grayImage, outputPath,x,y,Width,Height);
        }
        else {
            DetectExternalInner(grayImage, outputPath, low_pass_size, high_pass_thresh,x,y,Width,Height);
        }
    }

    void processContoursHR(const std::string& imagePath, const std::string& outputPath, int filter_value, bool only_external, int x, int y, int x2, int y2, int low_pass_size, int high_pass_thresh){

        cv::Mat image = cv::imread(imagePath);
        if (image.empty()) {
            std::cerr << "Could not load image..." << std::endl;
            return;
        }
        cv::Rect roi(x, y, x2 - x, y2 - y);
        cv::Mat image_roi = image(roi);

        cv::Mat fnm_image;
        cv::fastNlMeansDenoisingColored(image_roi, fnm_image, 10, 7, 7, 15);

        cv::Mat gaussian_blur;
        cv::GaussianBlur(fnm_image, gaussian_blur, cv::Size(3, 3), 5);

        cv::Mat bilater_bulr;
        cv::bilateralFilter(gaussian_blur, bilater_bulr, 15, low_pass_size, 20);

        cv::Mat edges;
        int high_pass_thresh_low = std::max(0, (high_pass_thresh - 40));

        cv::Mat grayImage;
        cv::cvtColor(gaussian_blur, grayImage, cv::COLOR_BGR2GRAY);

        int Width = image.cols;
        int Height = image.rows;

        if (only_external) {
            cv::Canny(bilater_bulr, edges, high_pass_thresh_low, high_pass_thresh);
            DetectExternal(edges, grayImage, outputPath,x,y,Width,Height);
        }
        else {
            DetectExternalInner(grayImage, outputPath, low_pass_size,high_pass_thresh,x,y, Width,Height);
        }

    }

    void DetectExternal(const cv::Mat& edges, const cv::Mat& gray_image, const std::string& outputPath, int x_point, int y_point, int Width, int Height) {

            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;

            cv::Mat kernel_dilate = cv::Mat::ones(7, 7, CV_8UC1);
            /*cv::dilate(edges, edges, kernel_dilate, cv::Point(-1, -1), 1);*/
            cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel_dilate);

            cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::Mat external_image = cv::Mat::ones(gray_image.size(), CV_8UC1) * 255;

            cv::drawContours(external_image, contours, -1, cv::Scalar(0), -1);

            cv::Mat kernel_erode = cv::Mat::ones(3, 3, CV_8UC1);
            cv::Mat new_external_image;
            //cv::erode(external_image, new_external_image, kernel_erode, cv::Point(-1, -1), 1);
            cv::morphologyEx(external_image, new_external_image, cv::MORPH_CLOSE, kernel_erode);
            std::vector<bool> pixels;
            int width = gray_image.cols;
            int height = gray_image.rows;

            pixels.resize(width * height);

            long sum = 0;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    uchar pixelValue = new_external_image.at<uchar>(y, x);
                    pixels[y * width + x] = (pixelValue == 0);
                }
            }
            PotraceHandler h;
            h.potrace(pixels, width, height,outputPath,x_point,y_point,Width,Height);
    }
    
    void DetectExternalInner(const cv::Mat& gray_image, const std::string& outputPath, int low_pass_size, int high_pass_thresh, int x_point, int y_point, int Width, int Height)
    {


        cv::Mat dstImage;
        double* x, * y;          /* x[n] y[n] coordinates of result contour point n */
        int* curve_limits;  /* limits of the curves in the x[] and y[] */
        int N, M;         /* result: N contour points, forming M curves */
        double S = 1; /* default sigma=0 */
        //double H = 4.2; /* default th_h=0  4.2 */
        //double L = 0.81; /* default th_l=0  0.81  */
        double W = 1.0;

        double H = std::max(0.0,(4.6 + ((high_pass_thresh - 40) * 1.0) / 10.0));
        double L = std::max(0.0,(0.81 + ((low_pass_size - 40) * 1.0) / 10.0));

        dstImage = gray_image;
        const int iHeight = dstImage.rows;
        const int iWidth = dstImage.cols;
        uchar* pSrc = gray_image.data;//new uchar[iHeight*iWidth];
        uchar* pDst = dstImage.data;

        devernay(&x, &y, &N, &curve_limits, &M, pSrc, pDst, iWidth, iHeight, S, H, L);
;
        cv::Mat test = cv::Mat::zeros(iHeight, iWidth, CV_8UC1);

        for (int k = 0; k < M; k++) /* write curves */
        {
            for (int i = curve_limits[k]; i < curve_limits[k + 1]; i++)
                test.at<uchar>(y[i], x[i]) = 255;
        }
        if (outputPath.c_str() != NULL) write_curves_svg(x, y, curve_limits, M, outputPath.c_str(), iWidth, iHeight, W,x_point,y_point,Width,Height);
    }

    void imageRecognition(const std::string& json_path) {

        std::ifstream configFile(json_path);
        if (!configFile.is_open()) {
            std::cerr << "Could not open config file..." << std::endl;
            return;
        }

        std::stringstream buffer;
        buffer << configFile.rdbuf();

        json config = nlohmann::json::parse(buffer.str());

        bool HR_image = config["HighRselutionImage"];
        const std::string imagePath = config["imagePath"];
        const std::string outputPath = config["outputPath"];
        bool only_external = config["only_external"];
        int low_pass_size = int(config["low_pass_size"]);//std::stoi(config["low_pass_size"].get<std::string>()) * 2 + 1;
        int high_pass_thresh = config["high_pass_thresh"];// std::stoi(config["high_pass_thresh"].get<std::string>());
        int filter_value = config["filter_value"];// std::stoi(config["filter_value"].get<std::string>());
        int x = config["x"];
        int y = config["y"];
        int x2 = config["x2"];
        int y2 = config["y2"];

        clock_t start_time = clock();

        //  methold1:sscannogram edge detect 
        if (!HR_image) {
            processContours(imagePath, outputPath, filter_value, only_external, x, y, x2, y2, low_pass_size, high_pass_thresh);
        }
        else {
            processContoursHR(imagePath, outputPath, filter_value, only_external, x, y, x2, y2, low_pass_size, high_pass_thresh);
        }
        clock_t end_time = clock();
        double time_spent = (double)(end_time - start_time) / CLOCKS_PER_SEC;
        printf("time: %.6f s\n", time_spent);

        return;
    }
}