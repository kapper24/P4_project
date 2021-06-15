#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QFile>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	ui_OpenCloseButton = findChild<QPushButton*>("OpenCloseButton");
	ui_StartQuitButton = findChild<QPushButton*>("StartQuitButton");
	ui_IdleButton = findChild<QPushButton*>("IdleButton");
	ui_GraspGorithmLabel = findChild<QLabel*>("GraspGorithmLabel");

	std::ofstream send(readFromPCLPath, std::ofstream::trunc);
	send << "Idle" << "\n";
	mjHand = HandState::Idle;

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
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	
	
	int vp(0);
	rs2::config config;
	serialnumber_ = dev_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
	auto sensors = dev_.query_sensors();
	auto advanceddev_ = dev_.as<rs400::advanced_mode>();

	STDepthTableControl depth_table = advanceddev_.get_depth_table();
	depth_table.depthUnits = 1000; // 0.001m
	depth_table.depthClampMin = 105; // måske 280 (28 cm)
	depth_table.depthClampMax = 700; // mm
	depth_table.disparityShift = 0;
	advanceddev_.set_depth_table(depth_table);

	float bckgr_gray_level = 0.0;

#pragma endregion
#pragma region start pointcloud
	viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp);
	viewer->setCameraPosition(0.0, 0.0, -0.5, 0.0, -1.0, 0.0, vp);
	viewer->setSize(800, 600);
	viewer->setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, vp);
	//viewer->addCoordinateSystem(0.1);

	/*
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
	*/
	std::cout << "Opening pipeline for " << serialnumber_ << std::endl;
	//rs2::config rgbconfig;
	//rgbconfig.enable_device(serialnumber_);
	
	config.enable_device(serialnumber_);

	config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
	config.enable_stream(RS2_STREAM_DEPTH, 424, 240, RS2_FORMAT_Z16, 90); //works fine!
	
	//config.enable_stream(RS2_STREAM_DEPTH, 256, 144, RS2_FORMAT_Z16, 90);
	//rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(config);
	
	//rs2::pipeline_profile rgbprofile = rgbpipe.start(rgbconfig);
	////cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
	//config.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
	//cfg_.enable_stream(RS2_STREAM_DEPTH, 848, 100, RS2_FORMAT_Z16, 100); // USB3.0 only!
#pragma endregion

	PCLtimer = new QTimer(this);
	connect(PCLtimer, SIGNAL(timeout()), this, SLOT(PCLupdate()));
	PCLtimer->start();
	//PCLtimer->start();

	
	QMetaObject::connectSlotsByName(this);
	//PCLupdate();
}

MainWindow::~MainWindow()
{
	delete ui;
}
void MainWindow::on_OpenCloseButton_clicked()
{
	openclose++;
	if (mjHand == HandState::Idle && openclose == 2) {
		if (graspnum != 0) {
			std::ofstream send(readFromPCLPath, std::ofstream::trunc);
			send << "Grasp " << graspnum << "\n" << "diameter " << diameter << "\n"  << "orientation " << orientation << "\n" << "Open";
			std::cout << "open";
			openclose = -2;
			mjHand = HandState::Open;
		}
	}
	else if(mjHand == HandState::Open && openclose == 0)
	{
		std::ofstream send(readFromPCLPath, std::ofstream::trunc);
		send << "Grasp " << graspnum << "\n" << "Close";
		std::cout << "close";
		mjHand = HandState::Close;
	}
	else if (mjHand == HandState::Close && openclose == 2) {
		std::ofstream send(readFromPCLPath, std::ofstream::trunc);
		send << "Grasp " << graspnum << "\n" << "Open";
		std::cout << "Open";
		openclose = -2;
		mjHand = HandState::Open;
	}
	

}
void MainWindow::on_StartQuitButton_clicked()
{
	startClicked++;
	if (startClicked >= 2) {
		isStarted = !isStarted;
		startClicked = 0;
	}
	std::cout << "clicked Start" << std::endl;
	
	//Launch python program
   //turn on camera and PCL
   //close program
}
void MainWindow::on_IdleButton_clicked()
{
	std::ofstream send(readFromPCLPath, std::ofstream::trunc);
	send << "Idle"<< "\n";
	openclose = 0;
	mjHand = HandState::Idle;
}
void MainWindow::PCLupdate()
{

#pragma region OpenClose UI

	if (mjHand == HandState::Open)
		ui_OpenCloseButton->setText("Close");
	else if (mjHand == HandState::Close)
		ui_OpenCloseButton->setText("Open");
	else if (mjHand == HandState::Idle)
		ui_OpenCloseButton->setText("Open");

#pragma endregion
#pragma region Start and Quit UI
	if (isStarted)
		ui_StartQuitButton->setText("Quit");
	else {
		ui_StartQuitButton->setText("Start");
	}
#pragma endregion

	if (isStarted) {

		

		//Variables that are reset every loop
		rs2::pointcloud pc;
		rs2::points points;
		WSPointCloudPtr cloud(new WSPointCloud);
		WSPointCloudPtr filteredCloud(new WSPointCloud);
		WSPointCloudPtr objects(new WSPointCloud);
		WSPointCloudPtr centerObject(new WSPointCloud);
		WSPointCloudPtr filteredObject(new WSPointCloud);


		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		rs2::frame colorimg = frames.get_color_frame();

		//get point cloud from Depth Image
		points = pc.calculate(depth);
		cloud = points2cloud(points);


		checkCloud newCheckCloud;
		newCheckCloud = filterCloud(cloud);
		filteredCloud = newCheckCloud.cloud;
		
		bool check = newCheckCloud.check;


		if (check) {
			objects = extractObject(filteredCloud);
		
			centerObject = getCenterObject(objects);
			
		//	cout << "Size: " << centerObject->size() << "\n";
			if (centerObject != objects) {
				objectSpecs objectInfo;

				//if Hand is in Idle mode
				if (mjHand == HandState::Idle) {
					//If Object is too small tri grip is used
				graspnum = 5;
					WSPointCloudPtr cloudPCAprojection(new WSPointCloud);
					pcl::PCA<WSPoint> pca;
					pca.setInputCloud(centerObject);
					pca.project(*centerObject, *cloudPCAprojection);
					WSPoint minPt;
					WSPoint maxPt;
					pcl::getMinMax3D(*cloudPCAprojection, minPt, maxPt);
					double objectSizez = abs(minPt.z) + abs(maxPt.z);
					double objectSizey = abs(minPt.y) + abs(maxPt.y);
					std::cout << objectSizez <<" " << objectSizey << std::endl;
					if (centerObject->size() < 50) {
						//std::cout << "tripod" << endl;
						graspnum = 5;
						orientation = 90;
					}
					else if (objectSizez <= 0.03 || objectSizey <= 0.03) {
						//std::cout << "tripod" << endl;
							graspnum = 5;
							orientation = 90;
					}
					else {
						//Get RGB Image from rgb stream
						const int w = colorimg.as<rs2::video_frame>().get_width();
						const int h = colorimg.as<rs2::video_frame>().get_height();
						cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorimg.get_data(), cv::Mat::AUTO_STEP);

						
						cv::waitKey(100);

						//if ImageFolderPath is empty, means that no image is currently being processed by python
						if (std::filesystem::is_empty(ImageFolderPath)) {
							cv::Rect rectROI(280, 0, 720, 720);
							cv::Mat imgROI = image(rectROI); //Nu er imgROI vores croppede
							cv::waitKey(100);
							cv::imwrite(ImageFolderPath + "\\RGBImage.png", imgROI); //write the image to a file as JPEG 
							
							
							


							//std::cout << "pic saved \n";
						}
						///////////// return object classification from google here ///////////////////////
						std::string line;
					//	std::cout << "waiting for response from google vision...." << std::endl;
						while (!std::filesystem::is_empty(ImageFolderPath)) {

						}
					
						std::ifstream f(readFromPythonPath);
						//std::cout << "Reading Response" << std::endl;
						cv::waitKey(100);

						if (getline(f, line))
						//	std::cout << line << std::endl;
						if (line == "Class 1") {
							graspnum = 1;
							objectInfo = fitCylinder(centerObject);
							//std::cout << line << std::endl;
						}
						else if (line == "Class 2") {
							graspnum = 2;
							objectInfo = fitWine(centerObject);
							if (objectInfo.diameter < 4) {
								graspnum = 5;
							}
						}
						else if (line == "Class 3") {
							graspnum = 3;
							objectInfo = fitCup(centerObject);
						}
						else if (line == "Class 4") {
							graspnum = 4;
							objectInfo = fitSphere(centerObject);
						}
						else if (line == "NULL") {
							graspnum = 1;
							objectInfo = fitCylinder(centerObject);
							std::cout << line << std::endl;
						}
						f.clear();
						filteredObject = objectInfo.objectCloud;
						orientation = objectInfo.orientation;
						diameter = objectInfo.diameter;
						
						std::ofstream send(projectPath + "log.txt", std::ofstream::app);
						send << diameter << " " << orientation << "\n";
						std::cout << "orientation " << orientation << "\n" << "size " << diameter << "\n" << line << "\n" << "\n";
					//	cout << "orientation: " << orientation << endl;
					//	cout << "diameter: " << diameter << endl;
					}
				}
				else { //If hand is not in idle mode
					
					cout << "Class " << graspnum << std::endl;
					cout << "orientation: " << orientation << endl;
					cout << "diameter: " << diameter << endl;
				}

			
				
				

				pcl::visualization::PointCloudColorHandlerCustom<WSPoint> cloud_color_h(0, 255, 0);
				viewer->removeAllShapes();
				viewer->removeAllPointClouds();
				viewer->addPointCloud(centerObject, cloud_color_h, "cloudname");
			}
			viewer->spinOnce(1, true);
			
		}
	}	
	else if (!isStarted && mjHand != HandState::Idle) {
		std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
		send << "Idle" << "\n";
		openclose = 0;
		mjHand = HandState::Idle;
	}
}

MainWindow::WSPointCloudPtr MainWindow::points2cloud(rs2::points pts)
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
}
MainWindow::checkCloud MainWindow::filterCloud(WSPointCloudPtr cloud) {
	pcl::VoxelGrid<WSPoint> vox;
	checkCloud newCheckCloud;
	bool check = 0;
	// voxelgrid filter
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.005f, 0.005f, 0.005f); // sets distance between points shown
	vox.filter(*cloud);
	for (auto&& p : cloud->points) {
		if (check == false) {
			if (p.z != 0) {
				check = true;
			}
		}
	}
	if (check == true) {
		newCheckCloud.cloud = cloud;
		newCheckCloud.check = check;
	}
	else if (check == false) {
		newCheckCloud.cloud = cloud;
		newCheckCloud.check = check;
	}

	return newCheckCloud;
}
MainWindow::WSPointCloudPtr MainWindow::extractObject(WSPointCloudPtr filteredCloud) {

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
	segNorm.setDistanceThreshold(0.05);
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
	extract.setNegative(true);
	extract.filter(*cloudSeg);

	return cloudSeg;
}
MainWindow::WSPointCloudPtr MainWindow::getCenterObject(WSPointCloudPtr objectCloud) {
	WSPointCloudPtr fail = objectCloud;
	pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<WSPoint> cluster;
	std::vector<WSPointCloudPtr> objects;
	std::vector<double> ecDist;
	WSPointCloudPtr centerObject(new WSPointCloud);

	//object cluster
	// - https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction
	tree->setInputCloud(objectCloud);
	cluster.setClusterTolerance(0.01); // 2cm
	cluster.setMinClusterSize(100);
	cluster.setMaxClusterSize(2000);
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
	}

	*centerObject = *objects[objectNumber];
	//cout << "size: " << centerObject->size() << endl;

	
	return centerObject;
}
MainWindow::objectSpecs MainWindow::fitCylinder(WSPointCloudPtr centerObject) {
	objectSpecs newCylinder;

	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	pcl::ExtractIndices<WSPoint> extract;
	double orientation;
	double radius;

	//Normals til cylinder 
	pcl::PointCloud<pcl::Normal>::Ptr cylinderNormals(new pcl::PointCloud<pcl::Normal>);
	norm.setInputCloud(centerObject);
	norm.setKSearch(25);
	norm.compute(*cylinderNormals);

	//Segmentere cylinder 
	// - https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#cylinder-segmentation

	pcl::ModelCoefficients::Ptr coefficientsCylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
	WSPointCloudPtr cylinder(new WSPointCloud);
	//pcl::ModelCoefficients lineCoeff; // dette bruges til at lave en linje i den retning som cylinderen peger
	//lineCoeff.values.resize(6);  // We need 6 values
	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_CYLINDER);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.03); // test det her
	segNorm.setRadiusLimits(0.01, 0.08);
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
	if (coefficientsCylinder->values.size() > 0) {
		double xdir = coefficientsCylinder->values[3];
		double ydir = coefficientsCylinder->values[4];
		radius = coefficientsCylinder->values[6];
		// Calculate orientation

		orientation = atan2(ydir, xdir) * 180 / 3.1415;

		if (orientation < 0) {
			orientation = orientation + 180;
		}
	}
	else {

		orientation = 90;
		radius = 1;
	}
	newCylinder.objectCloud = cylinder;
	newCylinder.orientation = orientation;
	newCylinder.diameter = radius * 2;
	
	return newCylinder;
}
MainWindow::objectSpecs MainWindow::fitWine(WSPointCloudPtr centerObject) {
	objectSpecs wineSpecs;
	double orientation= 90;
	if (centerObject->size() < 50) {
		wineSpecs.objectCloud = centerObject;
		wineSpecs.diameter = 6;
		wineSpecs.orientation = orientation;
	}
	else {
	pcl::PCA<pcl::PointXYZ> pca;
	WSPointCloudPtr cloudPCAprojection(new WSPointCloud);
	pca.setInputCloud(centerObject);
	pca.project(*centerObject, *cloudPCAprojection);

	WSPoint minPt;
	WSPoint maxPt;
	pcl::getMinMax3D(*cloudPCAprojection, minPt, maxPt);
	double diameter = abs(minPt.y) + abs(maxPt.y);

	wineSpecs.objectCloud = cloudPCAprojection;
	wineSpecs.orientation = orientation;
	wineSpecs.diameter = diameter;

	}
	return wineSpecs;
}
MainWindow::objectSpecs MainWindow::fitCup(WSPointCloudPtr centerObject) {
	objectSpecs cupSpecs;

	//Segmentere cylinder 
	pcl::ModelCoefficients::Ptr coefficientsCylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersCylinder(new pcl::PointIndices);
	WSPointCloudPtr cylinderObject(new WSPointCloud);
	WSPointCloudPtr cloudPCAprojection(new WSPointCloud);
	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::PointCloud<pcl::Normal>::Ptr cylinderNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	pcl::ExtractIndices<WSPoint> extract;
	norm.setInputCloud(centerObject);
	norm.setKSearch(50);
	norm.compute(*cylinderNormals);

	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_CYLINDER);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.06);
	segNorm.setRadiusLimits(0.02, 0.08);
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

	if (cylinderObject->size() < 50) {
		cupSpecs.objectCloud = cylinderObject;

		cupSpecs.diameter = 6;
	}
	else {
		double orientation;

		pcl::PCA<WSPoint> pca;
		pca.setInputCloud(cylinderObject);
		pca.project(*cylinderObject, *cloudPCAprojection);
	
		WSPoint minPt;
		WSPoint maxPt;
		pcl::getMinMax3D(*cloudPCAprojection, minPt, maxPt);
		double diameter = abs(minPt.x) + abs(maxPt.x);

		cupSpecs.objectCloud = cloudPCAprojection;
		cupSpecs.diameter = diameter;
	}

	if (coefficientsCylinder->values.size() > 0) {
		double xdir = coefficientsCylinder->values[3];
		double ydir = coefficientsCylinder->values[4];
		//radius = coefficientsCylinder->values[6];
		// Calculate orientation

		cupSpecs.orientation = atan2(ydir, xdir) * 180 / 3.1415;

		if (orientation < 0) {
			cupSpecs.orientation = orientation + 180;
		}
	}
	else {
		cupSpecs.orientation = 90;
	}
	return cupSpecs;
}
MainWindow::objectSpecs MainWindow::fitSphere(WSPointCloudPtr centerObject) {
	objectSpecs newSphere;

	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::SACSegmentationFromNormals<WSPoint, pcl::Normal> segNorm;
	pcl::ExtractIndices<WSPoint> extract;
	double orientation;
	double radius;

	//Normals til cylinder 
	pcl::PointCloud<pcl::Normal>::Ptr sphereNormals(new pcl::PointCloud<pcl::Normal>);
	norm.setInputCloud(centerObject);
	norm.setKSearch(25);
	norm.compute(*sphereNormals);

	//Segmentere cylinder 
	// - https://pcl.readthedocs.io/projects/tutorials/en/master/cylinder_segmentation.html#cylinder-segmentation

	pcl::ModelCoefficients::Ptr coefficientsSphere(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliersSphere(new pcl::PointIndices);
	WSPointCloudPtr sphere(new WSPointCloud);
	//pcl::ModelCoefficients lineCoeff; // dette bruges til at lave en linje i den retning som cylinderen peger
	//lineCoeff.values.resize(6);  // We need 6 values
	segNorm.setOptimizeCoefficients(true);
	segNorm.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
	segNorm.setMethodType(pcl::SAC_RANSAC);
	segNorm.setNormalDistanceWeight(0.1);
	segNorm.setMaxIterations(10000);
	segNorm.setDistanceThreshold(0.03); // test det her
	segNorm.setRadiusLimits(0.01, 0.06);
	segNorm.setInputCloud(centerObject);
	segNorm.setInputNormals(sphereNormals);
	segNorm.segment(*inliersSphere, *coefficientsSphere);

	extract.setInputCloud(centerObject);
	extract.setIndices(inliersSphere);
	extract.setNegative(false);
	extract.filter(*sphere);
	
	radius = coefficientsSphere->values[3]; 

	newSphere.objectCloud = sphere;
	newSphere.orientation = 90;
	newSphere.diameter = radius * 2;

	return newSphere;
}
