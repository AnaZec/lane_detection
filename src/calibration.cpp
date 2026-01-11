#include "calibration.hpp"
#include "utils.hpp"
#include <iostream>

namespace utils {
std::vector<std::string> glob(const std::string& pattern) {
    std::vector<std::string> files;
    cv::glob(pattern, files, false);
    std::sort(files.begin(), files.end());
    return files;
}
}

bool calibrateCameraFromChessboards(
    const std::string& calibDir,
    int nx,
    int ny,
    Calibration& calib
) {
    std::vector<cv::Point3f> objp;
    for (int j = 0; j < ny; j++)
        for (int i = 0; i < nx; i++)
            objp.emplace_back(i, j, 0.0f);

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    auto images = utils::glob(calibDir + "/calibration*.jpg");
    cv::Size imageSize;
    int used = 0;

    for (const auto& path : images) {
        cv::Mat img = cv::imread(path);
        if (img.empty()) continue;

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        imageSize = gray.size();

        std::vector<cv::Point2f> corners;
        if (!cv::findChessboardCorners(gray, {nx, ny}, corners))
            continue;

        cv::cornerSubPix(
            gray, corners, {11,11}, {-1,-1},
            cv::TermCriteria(cv::TermCriteria::EPS +
                             cv::TermCriteria::MAX_ITER, 30, 1e-3)
        );

        objectPoints.push_back(objp);
        imagePoints.push_back(corners);
        used++;
    }

    if (used < 5) return false;

    calib.K = cv::Mat::eye(3,3,CV_64F);
    calib.dist = cv::Mat::zeros(1,5,CV_64F);

    cv::calibrateCamera(
        objectPoints, imagePoints, imageSize,
        calib.K, calib.dist,
        cv::noArray(), cv::noArray()
    );

    return true;
}

bool saveCalibration(const std::string& path, const Calibration& calib) {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) return false;
    fs << "K" << calib.K;
    fs << "dist" << calib.dist;
    return true;
}

bool loadCalibration(const std::string& path, Calibration& calib) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["K"] >> calib.K;
    fs["dist"] >> calib.dist;
    return true;
}

cv::Mat undistortImage(const cv::Mat& img, const Calibration& calib) {
    cv::Mat out;
    cv::undistort(img, out, calib.K, calib.dist);
    return out;
}
