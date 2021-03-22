// P4_project.cpp : Defines the entry point for the application.
//

#include "P4_project.h"
#include <librealsense2/rs.hpp>

using namespace std;

int main()
{
	// Declarations
	rs2::pointcloud pc;
	rs2::points points;
	rs2::pipeline pipe;

	// Start your pipeline somewhere around here
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	auto color = frames.get_color_frame();

	pc.map_to(color);
	points = pc.calculate(depth);
	points.export_to_ply("pointcloud.ply", color);

	return 0;
}
