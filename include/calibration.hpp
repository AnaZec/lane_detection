#pragma once
#include <opencv2/opencv.hpp>
#include <string>

struct Calibration {
    cv::Mat K;
    cv::Mat dist;
};

bool calibrateCameraFromChessboards(
    const std::string& calibDir,
    int nx,
    int ny,
    Calibration& calib
);

bool saveCalibration(const std::string& path, const Calibration& calib);
bool loadCalibration(const std::string& path, Calibration& calib);

cv::Mat undistortImage(const cv::Mat& img, const Calibration& calib);
