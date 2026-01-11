#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <filesystem>

#include "calibration.hpp"
#include "pipeline.hpp"
#include "perspective.hpp"

static void usage() {
    std::cout
        << "Usage:\n"
        << "  ./lane_finding --calibrate\n"
        << "  ./lane_finding --image <path_to_image>\n"
        << "  ./lane_finding --video <input_video> <output_video.avi>\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }

    std::string cmd = argv[1];

    // ------------------------------------------------------------
    // CALIBRATION MODE
    // ------------------------------------------------------------
    if (cmd == "--calibrate") {
        Calibration calib;

        if (!calibrateCameraFromChessboards("camera_cal", 9, 6, calib)) {
            std::cerr << "Calibration failed: too few chessboards detected.\n";
            return 1;
        }

        std::filesystem::create_directories("calibration");

        if (!saveCalibration("calibration/camera_calib.yml", calib)) {
            std::cerr << "Failed to save calibration file.\n";
            return 1;
        }

        // save one undistorted sample image for report
        cv::Mat sample = cv::imread("camera_cal/calibration10.jpg");
        if (!sample.empty()) {
            cv::Mat und = undistortImage(sample, calib);
            cv::imwrite("debug/undistorted_calibration10.jpg", und);
            std::cout << "Saved undistorted calibration image.\n";
        }

        std::cout << "Calibration completed successfully.\n";
        return 0;
    }

    // ------------------------------------------------------------
    // IMAGE MODE
    // ------------------------------------------------------------
    if (cmd == "--image") {
        if (argc < 3) {
            usage();
            return 1;
        }

        std::string imgPath = argv[2];

        PipelineState state;
        if (!loadCalibration("calibration/camera_calib.yml", state.calib)) {
            std::cerr << "Calibration not found. Run --calibrate first.\n";
            return 1;
        }

        cv::Mat img = cv::imread(imgPath);
        if (img.empty()) {
            std::cerr << "Could not read image: " << imgPath << "\n";
            return 1;
        }

        state.persp = createPerspective(img.size());

        std::string base = std::filesystem::path(imgPath).stem().string(); // for images only
        state.output_dir = "output/images/" + base;

        cv::Mat result = processFrame(img, state);


        return 0;
    }

    // ------------------------------------------------------------
    // VIDEO MODE
    // ------------------------------------------------------------
    if (cmd == "--video") {
        if (argc < 4) {
            usage();
            return 1;
        }

        std::string inPath  = argv[2];
        std::string outPath = argv[3];

        PipelineState state;
        if (!loadCalibration("calibration/camera_calib.yml", state.calib)) {
            std::cerr << "Calibration not found. Run --calibrate first.\n";
            return 1;
        }

        cv::VideoCapture cap(inPath);
        if (!cap.isOpened()) {
            std::cerr << "Could not open input video: " << inPath << "\n";
            return 1;
        }

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Could not read first frame.\n";
            return 1;
        }

        state.persp = createPerspective(frame.size());

        double fps = cap.get(cv::CAP_PROP_FPS);
        int width  = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);


        // mjpg in avi
        int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
        cv::VideoWriter writer(outPath, fourcc, fps, cv::Size(width, height), true);

        if (!writer.isOpened()) {
            std::cerr << "Could not open VideoWriter: " << outPath << "\n";
            std::cerr << "Make sure output ends with .avi\n";
            return 1;
        }

        int frameCount = 0;
        do {
            cv::Mat out = processFrame(frame, state);
            writer.write(out);

            frameCount++;
            if (frameCount % 50 == 0)
                std::cout << "Processed " << frameCount << " frames\n";

        } while (cap.read(frame) && !frame.empty());

        std::cout << "Video processing complete.\n";
        std::cout << "Saved: " << outPath << "\n";
        return 0;
    }

    usage();
    return 1;
}
