//
// Created by biba_bo on 2020-10-26.
//

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// not sure if all of these includes are needed:
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>

constexpr int XL_POINT_COORD = /*50;//*/200;
constexpr int YL_POINT_COORD = /*200;//*/400;
constexpr int XR_POINT_COORD = /*550;//*/800;
constexpr int YR_POINT_COORD = /*200;//*/400;

constexpr int X_STEP_LENGTH = 5;

constexpr float MIN_POSSIBLE_DISTANCE = 0.6f;
constexpr float MAX_POSSIBLE_DISTANCE = 2.0f;
constexpr float MAX_DIFF_LR_DIST = 0.7f;

using namespace std;
using namespace cv;

bool set_points(cv::Point &left_point, cv::Point &right_point, const rs2::depth_frame &depth) {
    left_point.x = XL_POINT_COORD;
    right_point.x = XR_POINT_COORD;
    float left_point_dist, right_point_dist;
    bool is_not_fit = true;
    while (is_not_fit) {
        left_point_dist = depth.get_distance(left_point.x, left_point.y);
        right_point_dist = depth.get_distance(right_point.x, right_point.y);
        if (left_point_dist >= MIN_POSSIBLE_DISTANCE && left_point_dist <= MAX_POSSIBLE_DISTANCE) {
            if (right_point_dist >= MIN_POSSIBLE_DISTANCE && right_point_dist <= MAX_POSSIBLE_DISTANCE) {
                if (abs(right_point_dist - left_point_dist) <= MAX_DIFF_LR_DIST)
                    is_not_fit = false;
                else if (right_point_dist > left_point_dist) {
                    right_point.x -= X_STEP_LENGTH;
                } else {
                    left_point.x += X_STEP_LENGTH;
                }
            } else {
                right_point.x -= X_STEP_LENGTH;
            }
        } else {
            left_point.x += X_STEP_LENGTH;
            if (!(right_point_dist >= MIN_POSSIBLE_DISTANCE && right_point_dist <= MAX_POSSIBLE_DISTANCE)) {
                right_point.x -= X_STEP_LENGTH;
            }
        }
        if (right_point.x - left_point.x <= 0)
            break;
    }
    return !is_not_fit;
}

int main() {
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config rsConfig;
    rsConfig.disable_all_streams();
    //rsConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480);
    rsConfig.enable_stream(RS2_STREAM_DEPTH, 1280, 720);
    rsConfig.enable_stream(RS2_STREAM_INFRARED, 1);
    rsConfig.enable_stream(RS2_STREAM_COLOR, 640, 480);

    // Start streaming with default recommended configuration
    pipe.start(rsConfig);

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    cv::Rect human_box = cv::Rect(200, 100, 600, 600);
    //cv::Rect human_box = cv::Rect(50, 100, 500, 300);
    cv::Point left_point(XL_POINT_COORD, YL_POINT_COORD);
    cv::Point right_point(XR_POINT_COORD, YR_POINT_COORD);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::depth_frame depth = data.get_depth_frame();

        std::cout << "depth param: W->" << depth.get_width() << ", H->" << depth.get_height() << "\n";
        std::cout << "color param: W->" << data.get_color_frame().get_width() << ", H->" << data.get_color_frame().get_height() << "\n\n\n";

        float dist_to_left_point = -0.1f;
        float dist_to_right_point = -0.1f;
        if (set_points(left_point, right_point, depth)){
            dist_to_left_point = depth.get_distance(left_point.x, left_point.y);
            dist_to_right_point = depth.get_distance(right_point.x, right_point.y);
        }

        std::string left_info_str = "Left point dist:   " + std::to_string(dist_to_left_point);
        std::string right_info_str = "Right point dist: " + std::to_string(dist_to_right_point);
        std::string rotation_info_str = "Rotation angle:   " + std::to_string(-0.1);


        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data (bgr values)
        Mat depth_img(Size(w, h), CV_16UC1, (void *) depth.get_data(), Mat::AUTO_STEP);

        // Convert 16bit image to 8bit image
        depth_img.convertTo(depth_img, CV_8UC1, 15 / 256.0);

        cv::rectangle(depth_img, human_box, cv::Scalar(255, 255, 255), 2, 0, 0);
        cv::circle(depth_img, left_point, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_8, 0);
        cv::circle(depth_img, right_point, 10, cv::Scalar(255, 255, 255), 2, cv::LINE_8, 0);
        cv::putText(depth_img, left_info_str, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::putText(depth_img, right_info_str, cv::Point(20, 50), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        cv::putText(depth_img, rotation_info_str, cv::Point(20, 80), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
                    cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        //Update the window with new data
        imshow(window_name, depth_img);
    }

    return EXIT_SUCCESS;
}