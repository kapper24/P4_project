// P4_project.cpp : Defines the entry point for the application.
//

#include "P4_project.h"
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/grabber.h>


using namespace std;
using namespace cv;

typedef pcl::PointXYZ WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

WSPointCloudPtr points_to_pcl_no_texture(const rs2::points& points);

int main()
{
	
	// Declarations
	rs2::pointcloud pc;
	rs2::points points;
	rs2::frame img;
	rs2::pipeline pipe;
	rs2::colorizer color_map;
	rs2::config config;
	WSPointCloudPtr cloud;

	config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
	// Start your pipeline somewhere around here
	pipe.start(/*config*/);
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();
	rs2::frame colorimg = frames.get_color_frame();

	pc.map_to(color);
	points = pc.calculate(depth);
	cloud = points_to_pcl_no_texture(points);
	
	//points.export_to_ply("pointcloud.ply", color);
	const int w = color.as<rs2::video_frame>().get_width();
	const int h = color.as<rs2::video_frame>().get_height();
	Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

	// Update the window with new data
	imshow("window_name", image);

	bool isSuccess = imwrite("C:\\Users\\kaspe\\source\\repos\\P4_project\\out\\build\\x64-Debug\\MyImage.png", image); //write the image to a file as JPEG 
//bool isSuccess = imwrite("D:/MyImage.png", image); //write the image to a file as PNG
	if (isSuccess == false)
	{
		cout << "Failed to save the image" << endl;
		cin.get(); //wait for a key press
		return -1;
	}

	cout << "Image is succusfully saved to a file" << endl;

	cv::waitKey();
	return 0;
	

}


WSPointCloudPtr points_to_pcl_no_texture(const rs2::points& points)
{

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	WSPointCloudPtr cloud(new WSPointCloud());

	// Config of PCL Cloud object
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto vertices = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;
	}

	return cloud;
}

