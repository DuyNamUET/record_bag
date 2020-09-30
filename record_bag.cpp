#include <iostream>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui_c.h>

using namespace cv;

// return depth map that apply colormap
void colorDepthMap(const Mat& depth_raw, const int& color_map, Mat& depth_show)
{
    depth_show = Mat(depth_raw.size(), CV_8U);
    depth_raw.convertTo(depth_show, CV_8U, 255.0/65535);
    convertScaleAbs(depth_show, depth_show, 8.0);
    applyColorMap(depth_show, depth_show, color_map);
    return;
}

void visualization(const Mat& color_img, const Mat& depth_img)
{
    Mat show_img;
    hconcat(color_img, depth_img, show_img);
    imshow("Image", show_img);
}

int main()
{
    // config depth
    rs2::pipeline pipeline;
    rs2::config config;

    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    config.enable_record_to_file("test.bag");
    
    // start stream
    pipeline.start(config);

    while(true)
    {   
        std::cout << "Starting...\n";
        // wait for frames
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        if(!depth_frame && !color_frame) continue;

        // convert to Mat
        Mat color_img(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat depth_img(Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        // visualize
        visualization(color_img, depth_img);

        // Press ESC to exit
        if ((char)waitKey(25) == 27) break;
    }
    pipeline.stop();
    return 0;
}