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
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <librealsense2/rs_advanced_mode.hpp>

using namespace std;
using namespace cv;

typedef pcl::PointXYZ WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

//WSPointCloudPtr points_to_pcl_no_texture(const rs2::points& points);
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(rs2::points pts);
int main()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	float bckgr_gray_level = 0.0;
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	viewer->addCoordinateSystem(0.1);



	rs2::device dev_ = [] {
		rs2::context ctx;
		std::cout << "Waiting for device..." << std::endl;
		while (true) {
			for (auto&& dev : ctx.query_devices())
				return dev;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}();

	std::cout << "Device found:" << std::endl;
	std::cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " "
		<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
		<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;

	/*std::cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " "

		<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "

		<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;*/

	auto sensors = dev_.query_sensors();
	for (rs2::sensor& sensor : sensors) {
		std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
		for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
			if (profile.is<rs2::video_stream_profile>() && profile.stream_name() == "Depth") {
				rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
				std::cout << " Video stream: " << video_stream_profile.format() << " " <<
					video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << "Hz" << std::endl;
			}
			if (profile.is<rs2::motion_stream_profile>() && profile.stream_name() == "Accel") {
				rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
				std::cout << " Motion stream: " << motion_stream_profile.format() << " " <<
					motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << "Hz" << std::endl;
			}
			if (profile.is<rs2::pose_stream_profile>() && profile.stream_name() == "Gyro") {
				rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
				std::cout << " Pose stream: " << pose_stream_profile.format() << " " <<
					pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << "Hz" << std::endl;
			}
			std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
		}
	}
	rs2::config config;
	string serialnumber_ = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	std::cout << "Opening pipeline for " << serialnumber_ << std::endl;
	config.enable_device(serialnumber_);

	config.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90); //works fine!
		//config.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
		//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
		////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	
	
		//cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//	cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
	
	// Declarations
	rs2::pointcloud pc;
	rs2::points points;
	rs2::frame img;
	rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(config);
	rs2::colorizer color_map;
	//rs2::config config;
	WSPointCloudPtr cloud;

	//config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);

	// Start your pipeline somewhere around here
	//pipe.start(config);
	cout << "lol miguel" << endl;
	/*auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	//auto color = frames.get_color_frame();
	//rs2::frame colorimg = frames.get_color_frame();

	//pc.map_to(color);
	points = pc.calculate(depth);
	cloud = points_to_pcl(points);
	*/
	while (true)
	{
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();

		points = pc.calculate(depth);
		cloud = points_to_pcl(points);


		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_h(0, 255, 0);
		viewer->addPointCloud(cloud, cloud_color_h, "cloudname");

		viewer->spinOnce(1, true); 
	}
//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//viewer.showCloud(cloud);
	//while (!viewer.wasStopped())
	//{
	//}
	//points.export_to_ply("pointcloud.ply", color);
	//const int w = color.as<rs2::video_frame>().get_width();
	//const int h = color.as<rs2::video_frame>().get_height();
	//Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

	// Update the window with new data
	//imshow("window_name", image);

	//bool isSuccess = imwrite("C:\\Users\\kaspe\\source\\repos\\P4_project\\out\\build\\x64-Debug\\MyImage.png", image); //write the image to a file as JPEG 
//bool isSuccess = imwrite("D:/MyImage.png", image); //write the image to a file as PNG
/*	if (isSuccess == false)
	{
		cout << "Failed to save the image" << endl;
		cin.get(); //wait for a key press
		return -1;
	}

	cout << "Image is succusfully saved to a file" << endl;
	cv::waitKey();
	*/
	return 0;
	

}
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(rs2::points pts)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

	auto sp = pts.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(pts.size());
	auto ptr = pts.get_vertices();
	for (auto& p : cloud->points) {
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}
	return cloud;
	//mu_cloud_.lock();
//	cloud_ = cloud;
	//mu_cloud_.unlock();
	
}
/*
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
//find available streams
/*
auto sensors = dev_.query_sensors();
for (rs2::sensor& sensor : sensors) {
	std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
	for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
		if (profile.is<rs2::video_stream_profile>() && profile.stream_name() == "Depth") {
			rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
			std::cout << " Video stream: " << video_stream_profile.format() << " " <<
				video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << "Hz" << std::endl;
		}
		if (profile.is<rs2::motion_stream_profile>() && profile.stream_name() == "Accel") {
			rs2::motion_stream_profile motion_stream_profile = profile.as<rs2::motion_stream_profile>();
			std::cout << " Motion stream: " << motion_stream_profile.format() << " " <<
				motion_stream_profile.stream_type() << " @" << motion_stream_profile.fps() << "Hz" << std::endl;
		}
		if (profile.is<rs2::pose_stream_profile>() && profile.stream_name() == "Gyro") {
			rs2::pose_stream_profile pose_stream_profile = profile.as<rs2::pose_stream_profile>();
			std::cout << " Pose stream: " << pose_stream_profile.format() << " " <<
				pose_stream_profile.stream_type() << " @" << pose_stream_profile.fps() << "Hz" << std::endl;
		}
		//std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
	}
}



//

        dev_ = [] {​​​​

            rs2::context ctx;

            std::cout << "Waiting for device..." << std::endl;

            while (true) {​​​​

                for (auto&& dev : ctx.query_devices())

                    return dev;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));

            }​​​​

        }​​​​();         std::cout << "Device found:" << std::endl;

        std::cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " "

            << dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "

            << dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;


*/
