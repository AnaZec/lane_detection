#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace utils {

std::vector<std::string> glob(const std::string& pattern);

void putTextInfo(cv::Mat& img,
                 const std::string& text,
                 int line,
                 double scale = 0.8);

}
