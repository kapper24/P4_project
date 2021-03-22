// P4_project.cpp : Defines the entry point for the application.
//

#include "P4_project.h"
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
	
	// Declarations
	rs2::pointcloud pc;
	rs2::points points;
	rs2::frame img;
	rs2::pipeline pipe;
	rs2::colorizer color_map;

	// Start your pipeline somewhere around here
	pipe.start();
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();
	rs2::frame colorimg = frames.get_color_frame();

	pc.map_to(color);
	points = pc.calculate(depth);
	points.export_to_ply("pointcloud.ply", color);
	const int w = depth.as<rs2::video_frame>().get_width();
	const int h = depth.as<rs2::video_frame>().get_height();
	Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

	// Update the window with new data
	imshow("window_name", image);
	cv::waitKey();
	return 0;
	

}
