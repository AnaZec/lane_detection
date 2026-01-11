#pragma once
#include <opencv2/opencv.hpp>
#include "calibration.hpp"
#include "perspective.hpp"
#include "lane_detect.hpp"
#include <string>  

struct PipelineState {
    Calibration calib;
    Perspective persp;
    LaneState laneState;

    std::string output_dir; 
};

cv::Mat processFrame(const cv::Mat& frame, PipelineState& state);
