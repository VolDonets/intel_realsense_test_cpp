//
// Created by biba_bo on 2020-10-23.
//

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <opencv2/opencv.hpp>
//#include "example.h"          // Include short list of convenience functions for rendering

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    //window app(1280, 720, "RealSense Capture Example");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;
    // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::rates_printer printer;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Start streaming with default recommended configuration
    // The default video configuration contains Depth and Color streams
    // If a device is capable to stream IMU data, both Gyro and Accelerometer are enabled by default
    pipe.start();
    //cv::namedWindow("win1", cv::WINDOW_AUTOSIZE);

    int counter = 10;
    while (counter-- > 0) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
                apply_filter(printer).     // Print each enabled stream frame rate
                apply_filter(color_map);   // Find and colorize the depth data

        // The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
        // Each texture is displayed on different viewport according to it's stream unique id
        rs2::video_frame rs_frame_color = data.get_color_frame();
        std::cout << "frame info -> width: " << rs_frame_color.get_width() << ", height: " << rs_frame_color.get_height() << "\n";
        //std::cout << data.get_timestamp() << "\n";
        cv::Mat cv_frame_color(cv::Size(rs_frame_color.get_width(), rs_frame_color.get_height()), CV_8UC3, (void*) rs_frame_color.get_data(), cv::Mat::AUTO_STEP);
        //cv::imshow("win1", cv_frame);

        rs2::depth_frame rs_frame_depth = data.get_depth_frame();
        cv::Mat cv_frame_depth(cv::Size(rs_frame_depth.get_width(), rs_frame_depth.get_height()), CV_8UC3, (void*) rs_frame_depth.get_data(), cv::Mat::AUTO_STEP);

        std::string img_col_name = "img_color_" + std::to_string(counter) + ".jpg";
        cv::imwrite(img_col_name, cv_frame_color);
        std::string img_depth_name = "img_depth_" + std::to_string(counter) + ".jpg";
        cv::imwrite(img_depth_name, cv_frame_depth);
    }

    //cv::waitKey(0);
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}