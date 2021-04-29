
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QFile>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <iostream>
#include <fstream>
#include <sstream>


#include <math.h>
#include <librealsense2/rs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>//mabe delete
#include <pcl/io/grabber.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>//mabe delete
#include <pcl/filters/passthrough.h>//mos def delete
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>

//using namespace std;
//using namespace cv;
// data structures

typedef pcl::PointXYZ WSPoint;
typedef pcl::PointCloud<WSPoint> WSPointCloud;
typedef WSPointCloud::Ptr WSPointCloudPtr;

// det skal måske bruges i extractObject (det er måske bedre til at sortere støj væk)
struct objectNormal {
	WSPointCloudPtr objects;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
};

struct objectSpecs {
	WSPointCloudPtr objectCloud;
	double orientation;
	double diameter;
	pcl::ModelCoefficients lineCoeff;
};

//Function Declarations
WSPointCloudPtr points2cloud(rs2::points pts);
WSPointCloudPtr filterCloud(WSPointCloudPtr cloud);
WSPointCloudPtr extractObject(WSPointCloudPtr filteredCloud);
WSPointCloudPtr getCenterObject(WSPointCloudPtr objects);
void saveRGB2File(std::string serialnumber_, WSPointCloudPtr centerObject);
objectSpecs fitCylinder(WSPointCloudPtr centerObject);
objectSpecs fitWine(WSPointCloudPtr centerObject);
objectSpecs fitCup(WSPointCloudPtr centerObject);



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui_OpenCloseButton = findChild<QPushButton*>("OpenCloseButton");
    ui_StartQuitButton = findChild<QPushButton*>("StartQuitButton");
    ui_GraspGorithmLabel = findChild<QLabel*>("GraspGorithmLabel");
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(PCLupdate()));
    timer->start();
    QMetaObject::connectSlotsByName(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::on_OpenCloseButton_clicked()
{
    std::cout << "clicked open" << std::endl;
   
    //Get grip information
    // Open/close hand
    //
}
void MainWindow::on_StartQuitButton_clicked()
{
    std::cout << "clicked Start" << std::endl;
 //Launch python program
//turn on camera and PCL
//close program
}
void MainWindow::PCLupdate()
{

	
#pragma region find devices
	rs2::device dev_ = [] {
		rs2::context ctx;
		std::cout << "Waiting for device..." << std::endl;
		while (true) {
			for (auto&& dev : ctx.query_devices()) {
				return dev;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}();

	std::cout << "Device found:" << std::endl;
	std::cout << dev_.get_info(RS2_CAMERA_INFO_NAME) << " "
		<< dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << " "
		<< dev_.get_info(RS2_CAMERA_INFO_PRODUCT_ID) << std::endl << std::endl;
#pragma endregion
#pragma region Variables
	// Declarations
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	int vp(0);

	rs2::config config;
	std::string serialnumber_ = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	auto sensors = dev_.query_sensors();
	auto advanceddev_ = dev_.as<rs400::advanced_mode>();

	STDepthTableControl depth_table = advanceddev_.get_depth_table();
	depth_table.depthUnits = 1000; // 0.001m
	depth_table.depthClampMin = 105; // måske 280 (28 cm)
	depth_table.depthClampMax = 700; // mm
	depth_table.disparityShift = 0;
	advanceddev_.set_depth_table(depth_table);

	rs2::pointcloud pc;
	rs2::points points;
	rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(config);
	WSPointCloudPtr cloud(new WSPointCloud);
	WSPointCloudPtr filteredCloud(new WSPointCloud);
	WSPointCloudPtr objects(new WSPointCloud);
	WSPointCloudPtr centerObject(new WSPointCloud);
	WSPointCloudPtr filteredObject(new WSPointCloud);
	pcl::ModelCoefficients lineCoeff;
	double orientation;
	double diameter;
	float bckgr_gray_level = 0.0;

#pragma endregion
#pragma region start pointcloud
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	viewer->addCoordinateSystem(0.1);

	for (rs2::sensor& sensor : sensors) {
		std::cout << "Sensor " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
		for (rs2::stream_profile& profile : sensor.get_stream_profiles()) {
			if (profile.is<rs2::video_stream_profile>() && profile.stream_name() == "Depth") {
				rs2::video_stream_profile video_stream_profile = profile.as<rs2::video_stream_profile>();
				std::cout << " Video stream: " << video_stream_profile.format() << " " <<
					video_stream_profile.width() << "x" << video_stream_profile.height() << " @" << video_stream_profile.fps() << "Hz" << std::endl;
			}
			std::cout << "  stream " << profile.stream_name() << " " << profile.stream_type() << " " << profile.format() << " " << " " << profile.fps() << std::endl;
		}
	}

	std::cout << "Opening pipeline for " << serialnumber_ << std::endl;
	config.enable_device(serialnumber_);

	config.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
	//config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	//config.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
#pragma endregion
		
	while (!glfwWindowShouldClose(Gwindow))
	{
		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();

		points = pc.calculate(depth);
		cloud = points2cloud(points);
		filteredCloud = filterCloud(cloud);

		
		objects = extractObject(filteredCloud);
		

		centerObject = getCenterObject(objects);
		int n;
		if (centerObject != objects) {
			//pcl::io::savePCDFileASCII("test.pcd", *centerObject); //til at gemme .pcd filer (fuck den lange funktion)
			saveRGB2File(serialnumber_, centerObject);
			///////////// return object classification from google here ///////////////////////
			std::string line;

			std::ifstream f("C:\\Users\\Melvin\\Documents\\GUI_test\\Read.txt");
			while (line == "") {
				while (getline(f, line))
				{
					//std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\Read.txt", std::ofstream::trunc);
					//send << "1 \n";
				}
			}
			objectSpecs objectInfo;
			
			if (line == "Grasp 1") {
				n = 1;
				objectInfo = fitCylinder(centerObject);
			}
			else if (line == "Grasp 2") {
				n = 2;
				objectInfo = fitWine(centerObject);
			}
			else if (line == "Grasp 3") {
				n = 3;
				objectInfo = fitCup(centerObject);
			}
			else if (line == "NULL") {
				n = 1;
				objectInfo = fitCylinder(centerObject);
			}
			filteredObject = objectInfo.objectCloud;
			orientation = objectInfo.orientation;
			diameter = objectInfo.diameter;
			lineCoeff = objectInfo.lineCoeff;
			cout << "orientation: " << orientation << endl;
			cout << "diameter: " << diameter << endl;

			pcl::visualization::PointCloudColorHandlerCustom<WSPoint> cloud_color_h(0, 255, 0);
			viewer->addPointCloud(filteredObject, cloud_color_h, "cloudname");
			viewer->addLine(lineCoeff, "line");
		}

		viewer->spinOnce(1, true);
		// advance interactive simulation for 1/60 sec
		//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
		//  this loop will finish on time for the next frame to be rendered at 60 fps.
		//  Otherwise add a cpu timer and exit this loop when it is time to render.
		mjtNum simstart = d->time;
		while (d->time - simstart < 1 / 60.0)

			mj_step(m, d);
		switch (n)
		{
		case 1:
			targetpos = can;
			keychange = 0;
			break;
		case 2:
			targetpos = wineglass;
			keychange = 0;
			break;
		case 3:
			targetpos = cup;
			keychange = 0;
			break;
		default:
			break;
		}
		/*if (keychange == 1) {
			switch (aMujoco)
			{
			case 2:
				targetpos = wineglass;
				keychange = 0;
				break;
			case 3:
				targetpos = cup;
				keychange = 0;
				break;
			case 1:
				targetpos = can;
				keychange = 0;
				break;
			default:
				break;
			}
		}*/
		for (int i = 0; i < 13; i++)
		{

			if (d->ctrl[i] > targetpos[i]) {
				d->ctrl[i] -= 0.01;
			}
			if (d->ctrl[i] < targetpos[i]) {
				d->ctrl[i] += 0.01;
			}
		}

		for (int j = 13; j < 16; j++) {
			if (d->ctrl[j] > handpos[j - 13]) {
				d->ctrl[j] -= 4;
			}
			if (d->ctrl[j] < handpos[j - 13]) {
				d->ctrl[j] += 4;
			}
		}


		// get framebuffer viewport
		mjrRect viewport = { 0, 0, 0, 0 };
		glfwGetFramebufferSize(Gwindow, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

		// swap OpenGL buffers (blocking call due to v-sync)
		glfwSwapBuffers(Gwindow);

		// process pending GUI events, call GLFW callbacks
		glfwPollEvents();
	}
		if (glfwWindowShouldClose(Gwindow)) {
			//free visualization storage
			mjv_freeScene(&scn);
				mjr_freeContext(&con);

				// free MuJoCo model and data, deactivate
				mj_deleteData(d);
				mj_deleteModel(m);
				mj_deactivate();

				// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
				glfwTerminate();
#endif
		}
}

WSPointCloudPtr points2cloud(rs2::points pts)
{
	WSPointCloudPtr cloud(new WSPointCloud());

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
WSPointCloudPtr filterCloud(WSPointCloudPtr cloud) {
	pcl::VoxelGrid<WSPoint> vox;
	// voxelgrid filter
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.004f, 0.004f, 0.004f); // sets distance between points shown
	vox.filter(*cloud);
	return cloud;
}
WSPointCloudPtr extractObject(WSPointCloudPtr filteredCloud) {

	objectNormal newObject;
	//Normal Estimation tegner normal vektorer på overfalder 
	// - Ret sikker på det kan bruges til at segmentere 
	// - https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation

	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	norm.setSearchMethod(tree);
	norm.setInputCloud(filteredCloud);
	norm.setKSearch(50);
	norm.compute(*cloudNormals);


	//Segmentation... det virker lidt vildt og er ikke helt 100 på hvad der sker. 
	//Normals bliver brugt og inliers osv... 
	//Det tror ikke rigtigt det virker.. 
	// - https://pcl.readthedocs.io/projects/tutorials/en/latest/cylinder_segmentation.html#cylinder-segmentation

	pcl::ModelCoefficients::Ptr coefficientsPlane(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersPlane(new pcl::PointIndices);
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.03);
	segNorm.setInputCloud(filteredCloud);
	segNorm.setInputNormals(cloudNormals);
	segNorm.segment(*inliersPlane, *coefficientsPlane);

	// Find planer
	pcl::ExtractIndices<WSPoint> extract;
	WSPointCloudPtr cloudSeg(new WSPointCloud);
	extract.setInputCloud(filteredCloud);
	extract.setIndices(inliersPlane);
	extract.setNegative(false);
	extract.filter(*cloudSeg);
	// fjern planer
	pcl::PointCloud<pcl::Normal>::Ptr cloudSeg_Normals(new pcl::PointCloud<pcl::Normal>);
	extract.setNegative(true);
	extract.filter(*cloudSeg);

	newObject.objects = cloudSeg;
	newObject.normals = cloudNormals;

	return cloudSeg;
}
WSPointCloudPtr getCenterObject(WSPointCloudPtr objectCloud) {
	WSPointCloudPtr fail = objectCloud;
	pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<WSPoint> cluster;
	std::vector<pcl::PointCloud<WSPoint>::Ptr> objects;
	std::vector<double> ecDist;
	pcl::PointCloud<WSPoint>::Ptr centerObject(new pcl::PointCloud<WSPoint>);

	//object cluster
	// - https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction
	tree->setInputCloud(objectCloud);
	cluster.setClusterTolerance(0.02); // 2cm
	cluster.setMinClusterSize(50);
	cluster.setMaxClusterSize(7000);
	cluster.setSearchMethod(tree);
	cluster.setInputCloud(objectCloud);
	cluster.extract(cluster_indices);

	objects.clear();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		WSPointCloudPtr cloud_cluster(new WSPointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->push_back((*objectCloud)[*pit]);
		}

		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		objects.push_back(cloud_cluster);
	}

	//Obejct tættest på center af x og y 
	//Display object med mindst dist til center
	// - https://download.java.net/media/java3d/javadoc/1.3.2/javax/vecmath/Tuple4f.html#w 
	// - https://pointclouds.org/documentation/group__common.html

	for (int i = 0; i < objects.size(); i++)
	{
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*objects[i], centroid);

		double x = centroid[0];
		double y = centroid[1];

		double tempDist = sqrt((x * x) + (y * y));
		ecDist.push_back(tempDist);
	}

	if (ecDist.size() < 1) {
		return fail;
	}
	double smallestValue = ecDist[0];
	int objectNumber = 0;

	for (int i = 0; i < ecDist.size(); i++)
	{
		if (ecDist[i] < smallestValue)
		{
			smallestValue = ecDist[i];
			objectNumber = i;
		}
	} // første objekt lig med 0, andet objekt er lig med 1
	// cout << "objektet er: " << objectNumber << "dist er: " << smallestValue << endl;

	*centerObject = *objects[objectNumber];

	return centerObject;
}
objectSpecs fitCylinder(WSPointCloudPtr centerObject) {
	objectSpecs newCylinder;

	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	pcl::ExtractIndices<WSPoint> extract;

	//Normals til cylinder 
	pcl::PointCloud<pcl::Normal>::Ptr cylinderNormals(new pcl::PointCloud<pcl::Normal>);
	norm.setInputCloud(centerObject);
	norm.setKSearch(50);
	norm.compute(*cylinderNormals);

	//Segmentere cylinder 
	// - https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#cylinder-segmentation

	pcl::ModelCoefficients::Ptr coefficientsCylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
	WSPointCloudPtr cylinder(new WSPointCloud);
	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_CYLINDER);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.5);
	segNorm.setRadiusLimits(0, 0.5);
	segNorm.setInputCloud(centerObject);
	segNorm.setInputNormals(cylinderNormals);
	segNorm.segment(*inliersCylinder, *coefficientsCylinder);

	extract.setInputCloud(centerObject);
	extract.setIndices(inliersCylinder);
	extract.setNegative(false);
	extract.filter(*cylinder);

	//coefficientsCylinder inderholder de paramtre ransac bruger til at finde en cylinder 
	// - https://pointclouds.org/documentation/classpcl_1_1_sample_consensus_model_cylinder.html
	// - http://www.pcl-users.org/Sphere-detection-by-RANSAC-td4038300.html
	
	double xdir = coefficientsCylinder->values[3];
	double ydir = coefficientsCylinder->values[4];
	double radius = coefficientsCylinder->values[6];


	// Calculate orientation
	double orientation;
	if (xdir > 0) {
		if (ydir > 0) {
			orientation = atan2(ydir, xdir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -atan2(abs(ydir), xdir) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else if (xdir < 0) {
		if (ydir > 0) {
			orientation = 90 + atan2(abs(xdir), ydir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -90 - atan2(abs(xdir), abs(ydir)) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else {
		orientation = 90;
	}

	if (orientation < 0) {
		orientation = orientation + 180;
	}

	pcl::ModelCoefficients lineCoeff; // dette bruges til at lave en linje i den retning som cylinderen peger
	lineCoeff.values.resize(6);  // We need 6 values
	lineCoeff.values[0] = coefficientsCylinder->values[0];
	lineCoeff.values[1] = coefficientsCylinder->values[1];
	lineCoeff.values[2] = coefficientsCylinder->values[2];

	lineCoeff.values[3] = coefficientsCylinder->values[3];
	lineCoeff.values[4] = coefficientsCylinder->values[4];
	lineCoeff.values[5] = coefficientsCylinder->values[5];

	newCylinder.objectCloud = cylinder;
	newCylinder.orientation = orientation;
	newCylinder.diameter = radius * 2;
	newCylinder.lineCoeff = lineCoeff;
	return newCylinder;
}
objectSpecs fitWine(WSPointCloudPtr centerObject) {
	//Segmentere af vinglas med circler
		// - virker faktisk bedre og bruger ikke normal. 
		// - Får en bedre diameter 
		// - https://pointclouds.org/documentation/classpcl_1_1_sample_consensus_model_circle3_d.html
	objectSpecs wineSpecs;

	WSPointCloudPtr wineGlass(new WSPointCloud);
	pcl::SACSegmentation<WSPoint> seg;
	pcl::ModelCoefficients::Ptr coefficientsCircle(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCircle(new pcl::PointIndices);
	pcl::ExtractIndices<WSPoint> extract;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.08);
	seg.setRadiusLimits(0, 0.5);
	seg.setInputCloud(centerObject);
	seg.segment(*inliersCircle, *coefficientsCircle);

	extract.setInputCloud(centerObject);
	extract.setIndices(inliersCircle);
	extract.setNegative(false);
	extract.filter(*wineGlass);

	double radius = coefficientsCircle->values[3];
	/*
	double orientation;
	double xdir = coefficientsCircle->values[4];
	double ydir = coefficientsCircle->values[5];
	if (xdir > 0) {
		if (ydir > 0) {
			orientation = atan2(ydir, xdir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -atan2(abs(ydir), xdir) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else if (xdir < 0) {
		if (ydir > 0) {
			orientation = 90 + atan2(abs(xdir), ydir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -90 - atan2(abs(xdir), abs(ydir)) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else {
		orientation = 90;
	}

	if (orientation < 0) {
		orientation = orientation + 180;
	}*/

	pcl::ModelCoefficients lineCoeff; // dette bruges til at lave en linje i den retning som cylinderen peger
	lineCoeff.values.resize(6);  // We need 6 values
	lineCoeff.values[0] = coefficientsCircle->values[0];
	lineCoeff.values[1] = coefficientsCircle->values[1];
	lineCoeff.values[2] = coefficientsCircle->values[2];

	lineCoeff.values[3] = coefficientsCircle->values[4];
	lineCoeff.values[4] = coefficientsCircle->values[5];
	lineCoeff.values[5] = coefficientsCircle->values[6];

	wineSpecs.objectCloud = wineGlass;
	wineSpecs.orientation = 90;
	wineSpecs.diameter = radius * 2;
	wineSpecs.lineCoeff = lineCoeff;
	return wineSpecs;
}
objectSpecs fitCup(WSPointCloudPtr centerObject) {
	objectSpecs cupSpecs;

	//Segmentere cylinder 
		// - https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#cylinder-segmentation
	pcl::ModelCoefficients::Ptr coefficientsCylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
	WSPointCloudPtr cylinderObject(new WSPointCloud);
	pcl::PointCloud<pcl::Normal>::Ptr cylinderNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	pcl::ExtractIndices<WSPoint> extract;

	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_CYLINDER);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.008);
	segNorm.setRadiusLimits(0, 0.1);
	segNorm.setInputCloud(centerObject);
	segNorm.setInputNormals(cylinderNormals);
	segNorm.segment(*inliersCylinder, *coefficientsCylinder);

	extract.setInputCloud(centerObject);
	extract.setIndices(inliersCylinder);
	extract.setNegative(false);
	extract.filter(*cylinderObject);

	//Segmentere cylinderen fra koppen 
	// - find ud af hvilke data vi kan få ud og hvordan det kan bruges 
	extract.setNegative(true);
	extract.filter(*cylinderObject);

	//Det her burde at give os bredden på hanken af koppen, men det er mærkeligt....
	pcl::SACSegmentation<WSPoint> seg;
	pcl::ModelCoefficients::Ptr coefficientsCup(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCup(new pcl::PointIndices);
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_STICK);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.08);
	seg.setRadiusLimits(0.002, 0.05);
	seg.setInputCloud(cylinderObject);
	seg.segment(*inliersCup, *coefficientsCup);
	//*cylinderObject = *centerObject;

	extract.setInputCloud(cylinderObject);
	extract.setIndices(inliersCup);
	extract.setNegative(false);
	extract.filter(*cylinderObject);

	double radius = coefficientsCup->values[6];
	double orientation;
	double xdir = coefficientsCup->values[3];
	double ydir = coefficientsCup->values[4];
	if (xdir > 0) {
		if (ydir > 0) {
			orientation = atan2(ydir, xdir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -atan2(abs(ydir), xdir) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else if (xdir < 0) {
		if (ydir > 0) {
			orientation = 90 + atan2(abs(xdir), ydir) * 180 / 3.1415;
		}
		else if (ydir < 0) {
			orientation = -90 - atan2(abs(xdir), abs(ydir)) * 180 / 3.1415;
		}
		else {
			orientation = 0;
		}
	}
	else {
		orientation = 90;
	}
	if (orientation < 0) {
		orientation = orientation + 180;
	}

	pcl::ModelCoefficients lineCoeff; // dette bruges til at lave en linje i den retning som cylinderen peger
	lineCoeff.values.resize(6);  // We need 6 values
	lineCoeff.values[0] = coefficientsCup->values[0];
	lineCoeff.values[1] = coefficientsCup->values[1];
	lineCoeff.values[2] = coefficientsCup->values[2];

	lineCoeff.values[3] = coefficientsCup->values[3];
	lineCoeff.values[4] = coefficientsCup->values[4];
	lineCoeff.values[5] = coefficientsCup->values[5];

	cupSpecs.objectCloud = cylinderObject;
	cupSpecs.orientation = 90;
	cupSpecs.diameter = radius * 2;
	cupSpecs.lineCoeff = lineCoeff;

	return cupSpecs;
}
void saveRGB2File(std::string serialnumber_, WSPointCloudPtr centerObject) {
	rs2::config config;
	config.enable_device(serialnumber_);
	config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
	rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(config);
	auto frames = pipe.wait_for_frames();
	rs2::frame colorimg = frames.get_color_frame();

	const int w = colorimg.as<rs2::video_frame>().get_width();
	const int h = colorimg.as<rs2::video_frame>().get_height();
	cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorimg.get_data(), cv::Mat::AUTO_STEP);


	/////////// Benjas kode /////////////////////
	WSPoint minPt;
	WSPoint maxPt;
	pcl::getMinMax3D(*centerObject, minPt, maxPt);

	int pixelScalar = 3; //Den her beskriver størrelsesforholdet mellem point cloud og RGB billedet
	float realMaxX = 212; //Her regnes der ift centrum af point cloud.
	float realMinX = 212; //Hvilket betyder at min og max er ift objektet
	float realMaxY = 120; //ikke pixel-værdierne
	float realMinY = 120;


	realMaxX = (realMaxX + ((realMaxX * 2) * (3 * maxPt.x))) * pixelScalar; //centrum + Xprocent ud af rgb Xaksen
	realMinX = (realMinX + ((realMinX * 2) * (3 * minPt.x))) * pixelScalar;
	realMaxY = (realMaxY + ((realMaxY * 2) * (3 * maxPt.y))) * pixelScalar;
	realMinY = (realMinY + ((realMinY * 2) * (3 * minPt.y))) * pixelScalar;

	float ROIscalar = 0.2; //Vi vil gerne have 20% af objektet
	float ROIscalarUp = 1 + ROIscalar; //Objektet + 20% af objektet

	int width = abs(realMaxX - realMinX) * ROIscalarUp; //Bredden af objektet
	int height = abs(realMaxY - realMinY) * ROIscalarUp; //Højden af objektet

	int minX = realMinX - ((width / ROIscalarUp) / 2) * ROIscalar; //x til upper left corner af objekt
	int maxY = realMaxY + ((height / ROIscalarUp) / 2) * ROIscalar; //y til upper left corner af objekt

	int maxX = realMaxX + ((width / ROIscalarUp) / 2) * ROIscalar; //x til buttom left corner af objekt
	int minY = realMinY - ((height / ROIscalarUp) / 2) * ROIscalar; //y til buttom left corner af objekt

	if (minX < 1) //Hvis objektets x1 er uden for billedet: (kan ske grundet de ekstra 20% kant)
	{
		minX = 1; //Objektets x1 sættes i billedets hjørne
	}
	if (minY < 1) //Hvis objektets y1 er uden for billedet:
	{
		minY = 1; //Objektets y1 sættes i billedets hjørne
	}
	if (maxX > 1279) //Hvis objektets x2 er uden for billedet:
	{
		maxX = 1279; //Objektets x2 sættes i billedets hjørne (-1 px fordi boundingbox går 1 px udenfor)
	}
	if (maxY > 719) //Hvis objektets y2 er uden for billedet:
	{
		maxY = 719; //Objektets y2 sættes i billedets hjørne (-1 px fordi boundingbox går 1 px udenfor)
	}

	cv::Point P1(minX, minY); // Upper left corner
	cv::Point P2(minX, maxY); // Buttom left corner
	cv::Point P3(maxX, maxY); // Upper right corner

	cv::RotatedRect rotatedROI = cv::RotatedRect(P1, P2, P3); //Rektangel rotatret rundt om objekt (+20%)
	cv::Rect rectROI = rotatedROI.boundingRect(); //Laver en Rect (retvinklet) for at kunne croppe
	cv::Mat imgROI = image(rectROI); //Nu er imgROI vores croppede
	cv::waitKey(100);

	imwrite("C:\\Users\\Melvin\\Documents\\GUI_test\\images\\MyImage.png", imgROI); //write the image to a file as JPEG 
	cv::waitKey(100);
	pipe.stop();
	cv::waitKey();
}
