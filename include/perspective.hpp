#pragma once
#include <opencv2/opencv.hpp>

struct Perspective {
    cv::Mat M;
    cv::Mat Minv;
    cv::Size warpSize;
};

Perspective createPerspective(const cv::Size& imgSize);

cv::Mat warpPerspectiveBinary(const cv::Mat& binary,
                              const Perspective& p);
