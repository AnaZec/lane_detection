#include "perspective.hpp"
#include <opencv2/opencv.hpp>

Perspective createPerspective(const cv::Size& imgSize) {
    Perspective p;
    p.warpSize = imgSize;

    float w = (float)imgSize.width;
    float h = (float)imgSize.height;

    // scaled points (work across 960x540, 1280x720, 1920x1080)
    std::vector<cv::Point2f> src = {
        {w * 0.46f, h * 0.64f}, // top-left
        {w * 0.54f, h * 0.64f}, // top-right
        {w * 0.88f, h * 1.00f}, // bottom-right
        {w * 0.12f, h * 1.00f}  // bottom-left
    };

    std::vector<cv::Point2f> dst = {
        {w * 0.25f, 0.0f},
        {w * 0.75f, 0.0f},
        {w * 0.75f, h},
        {w * 0.25f, h}
    };

    p.M    = cv::getPerspectiveTransform(src, dst);
    p.Minv = cv::getPerspectiveTransform(dst, src);
    return p;
}

cv::Mat warpPerspectiveBinary(const cv::Mat& binary, const Perspective& p) {
    cv::Mat warped;
    cv::warpPerspective(binary, warped, p.M, p.warpSize);
    return warped;
}
