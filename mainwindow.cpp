#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QFile>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <pcl/visualization/pcl_visualizer.h>

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

	std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
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
	depth_table.depthClampMin = 105; // m�ske 280 (28 cm)
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
	viewer->addCoordinateSystem(0.1);

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
			std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
			send << "Grasp " << graspnum << "\n" << "diameter " << "\n" << "Open";
			std::cout << "open";
			openclose = -2;
			mjHand = HandState::Open;
		}
	}
	else if(mjHand == HandState::Open && openclose == 0)
	{
		std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
		send << "Grasp " << graspnum << "\n" << "Close";
		std::cout << "close";
		mjHand = HandState::Close;
	}
	else if (mjHand == HandState::Close && openclose == 2) {
		std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
		send << "Grasp " << graspnum << "\n" << "Open";
		std::cout << "Open";
		openclose = -2;
		mjHand = HandState::Open;
	}
	

}
void MainWindow::on_StartQuitButton_clicked()
{
	std::cout << "clicked Start" << std::endl;
	
	//Launch python program
   //turn on camera and PCL
   //close program
}
void MainWindow::on_IdleButton_clicked()
{
	std::ofstream send("C:\\Users\\Melvin\\Documents\\GUI_test\\ReadfromPCL.txt", std::ofstream::trunc);
	send << "Idle"<< "\n";
	openclose = 0;
	mjHand = HandState::Idle;
}
void MainWindow::PCLupdate()
{
	if (mjHand == HandState::Open)
		ui_OpenCloseButton->setText("Close");
	else if (mjHand == HandState::Close)
		ui_OpenCloseButton->setText("Open");
	else if (mjHand == HandState::Idle)
		ui_OpenCloseButton->setText("Open");
		//Variables that are reset every loop
		rs2::pointcloud pc;
		rs2::points points;
		WSPointCloudPtr cloud(new WSPointCloud);
		WSPointCloudPtr filteredCloud(new WSPointCloud);
		WSPointCloudPtr objects(new WSPointCloud);
		WSPointCloudPtr centerObject(new WSPointCloud);
		WSPointCloudPtr filteredObject(new WSPointCloud);
		//pcl::ModelCoefficients lineCoeff;
		double orientation;
		double diameter;

		viewer->removeAllShapes();
		viewer->removeAllPointClouds();
		auto frames = pipe.wait_for_frames();
		auto depth = frames.get_depth_frame();
		rs2::frame colorimg = frames.get_color_frame();

		points = pc.calculate(depth);
		cloud = points2cloud(points);
		checkCloud newCheckCloud;
		newCheckCloud = filterCloud(cloud);
		filteredCloud = newCheckCloud.cloud;
		bool check = newCheckCloud.check;
		if (check) {

			objects = extractObject(filteredCloud);
			centerObject = getCenterObject(objects);

			if (centerObject != objects) {
				//pcl::io::savePCDFileASCII("test.pcd", *centerObject); //til at gemme .pcd filer (fuck den lange funktion)
				//pipe.stop();
				//saveRGB2File(serialnumber_, centerObject);
					objectSpecs objectInfo;
				if (mjHand == HandState::Idle){
					const int w = colorimg.as<rs2::video_frame>().get_width();
					const int h = colorimg.as<rs2::video_frame>().get_height();
					cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorimg.get_data(), cv::Mat::AUTO_STEP);
					if (std::filesystem::is_empty("C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\images")) {

						cv::imwrite("C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\images\\MyImage.png", image); //write the image to a file as JPEG 
						std::cout << "pic saved \n";
					}

					//pipe.start();
					///////////// return object classification from google here ///////////////////////
					std::string line;
					std::cout << "waiting for response from google vision" << std::endl;
					while (!std::filesystem::is_empty("C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\images")) {

					}
					std::ifstream f("C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\ReadfromPython.txt");
					std::cout << "readfromPython" << std::endl;
					cv::waitKey(100);
					if (getline(f, line))
						std::cout << line << std::endl;
					//std::ofstream send("C:\\Users\\Melvin\\Documents\\P4_project\\Read.txt", std::ofstream::trunc);
					//send << "1 \n";
					if (line == "Grasp 1") {
						graspnum = 1;
						objectInfo = fitCylinder(centerObject);
						std::cout << line << std::endl;
					}
					else if (line == "Grasp 2") {
						graspnum = 2;
						objectInfo = fitWine(centerObject);
					}
					else if (line == "Grasp 3") {
						graspnum = 3;
						objectInfo = fitCup(centerObject);
					}
					else if (line == "NULL") {
						graspnum = 1;
						objectInfo = fitCylinder(centerObject);
						std::cout << line << std::endl;
					}
					f.clear();
				}
				else {
					switch (graspnum)
					{
					case 1:
						objectInfo = fitCylinder(centerObject);
						break;
					case 2:
						objectInfo = fitWine(centerObject);
						break;
					case 3:
						objectInfo = fitCup(centerObject);
						break;
					default:
						break;
					}
				}
				//objectInfo = fitCylinder(centerObject);
				filteredObject = objectInfo.objectCloud;
				orientation = objectInfo.orientation;
				diameter = objectInfo.diameter;
				cout << "orientation: " << orientation << endl;
				cout << "diameter: " << diameter << endl;

				pcl::visualization::PointCloudColorHandlerCustom<WSPoint> cloud_color_h(0, 255, 0);
				viewer->addPointCloud(filteredObject, cloud_color_h, "cloudname");
				//viewer->addLine(lineCoeff, "line");
			}
				viewer->spinOnce(1, true);
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

	//Normal Estimation tegner normal vektorer p� overfalder 
	// - Ret sikker p� det kan bruges til at segmentere 
	// - https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#normal-estimation

	pcl::NormalEstimation<WSPoint, pcl::Normal> norm;
	pcl::search::KdTree<WSPoint>::Ptr tree(new pcl::search::KdTree<WSPoint>());
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	norm.setSearchMethod(tree);
	norm.setInputCloud(filteredCloud);
	norm.setKSearch(50);
	norm.compute(*cloudNormals);

	//Segmentation... det virker lidt vildt og er ikke helt 100 p� hvad der sker. 
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
	cluster.setMinClusterSize(150);
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

	//Obejct t�ttest p� center af x og y 
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
	pcl::PCA<pcl::PointXYZ> pca;
	WSPointCloudPtr cloudPCAprojection(new WSPointCloud);
	pca.setInputCloud(centerObject);
	pca.project(*centerObject, *cloudPCAprojection);
	//pca.getEigenVectors();
	//pca.getEigenValues();

	WSPoint minPt;
	WSPoint maxPt;
	pcl::getMinMax3D(*cloudPCAprojection, minPt, maxPt);
	double diameter = abs(minPt.y) + abs(maxPt.y);

	wineSpecs.objectCloud = cloudPCAprojection;
	wineSpecs.orientation = 90;
	wineSpecs.diameter = diameter;
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
	// - find ud af hvilke data vi kan f� ud og hvordan det kan bruges 
	extract.setNegative(true);
	extract.filter(*cylinderObject);

	double orientation;

	pcl::PCA<WSPoint> pca;
	pca.setInputCloud(cylinderObject);
	pca.project(*cylinderObject, *cloudPCAprojection);
	//pca.getEigenVectors();
	//pca.getEigenValues();

	WSPoint minPt;
	WSPoint maxPt;	
	pcl::getMinMax3D(*cloudPCAprojection, minPt, maxPt);
	double diameter = abs(minPt.x)+abs(maxPt.x);
	
	cupSpecs.objectCloud = cloudPCAprojection;
	cupSpecs.orientation = 90;
	cupSpecs.diameter = diameter - 0.03;
	
	return cupSpecs;
}
/*
void saveRGB2File(std::string serialnumber_, WSPointCloudPtr centerObject) {
	/*rs2::config config;
	config.enable_device(serialnumber_);
	config.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 15);
	rs2::pipeline pipe;
	rs2::pipeline_profile profile = pipe.start(config);
	//auto frames = pipe.wait_for_frames();
	//colorimg = frames.get_color_frame();

	const int w = colorimg.as<rs2::video_frame>().get_width();
	const int h = colorimg.as<rs2::video_frame>().get_height();
	cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)colorimg.get_data(), cv::Mat::AUTO_STEP);


	/////////// Benjas kode /////////////////////
	float ratioMaxX = objectMaxX / firstMaxX;
	float ratioMinX = objectMinX / firstMinX;
	float ratioMaxY = objectMaxY / firstMaxY;
	float ratioMinY = objectMinY / firstMinY;

	//std::cout << "Ratio " << ratioMaxX << " " << ratioMinX << " " << ratioMaxY << " " << ratioMinY << std::endl;

	float pixelMaxX = abs(640 + (640 * ratioMaxX));
	float pixelMinX = abs(640 + (640 * ratioMinX));
	float pixelMaxY = abs(360 + (360 * ratioMaxY));
	float pixelMinY = abs(360 + (360 * ratioMinY));

	//std::cout << "Pixel " << pixelMaxX << " " << pixelMinX << " " << pixelMaxY << " " << pixelMinY << std::endl;

	float ROIscalar = 0.2; //Vi vil gerne have 20% af objektet

	int width = abs(pixelMaxX - pixelMinX) * ROIscalar; //Bredden af objektet
	int height = abs(pixelMaxY - pixelMinY) * ROIscalar; //H�jden af objektet

	int maxX = (pixelMaxX + (width)); //x til buttom left corner af objekt
	int minX = (pixelMinX - (width)); //x til upper left corner af objekt
	int maxY = (pixelMaxY + (height)); //y til upper left corner af objekt
	int minY = (pixelMinY - (height)); //y til buttom left corner af objekt

	//std::cout << "Corners " << maxX << " " << minX << " " << maxY << " " << minY << std::endl;

	if (minX < 1) //Hvis objektets x1 er uden for billedet: (kan ske grundet de ekstra 20% kant)
	{
		minX = 1; //Objektets x1 s�ttes i billedets hj�rne
	}

	if (minY < 1) //Hvis objektets y1 er uden for billedet:
	{
		minY = 1; //Objektets y1 s�ttes i billedets hj�rne
	}

	if (maxX > 1279) //Hvis objektets x2 er uden for billedet:
	{
		maxX = 1279; //Objektets x2 s�ttes i billedets hj�rne (-1 px fordi boundingbox g�r 1 px udenfor)
	}

	if (maxY > 719) //Hvis objektets y2 er uden for billedet:
	{
		maxY = 719; //Objektets y2 s�ttes i billedets hj�rne (-1 px fordi boundingbox g�r 1 px udenfor)
	}

	cv::Point P1(minX, minY); //Upper left corner
	cv::Point P2(minX, maxY); //Buttom left corner
	cv::Point P3(maxX, maxY); //Buttom right corner

	cv::RotatedRect rotatedROI = cv::RotatedRect(P1, P2, P3); //Rektangel rotatret rundt om objekt (+20%)
	cv::Rect rectROI = rotatedROI.boundingRect(); //Laver en Rect (retvinklet) for at kunne croppe

	cv::Mat imgROI = image(rectROI); //Nu er imgROI vores croppede
	cv::waitKey(100);

	imwrite("C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\images\\MyImage.png", imgROI); //write the image to a file as JPEG 
	cv::waitKey(100);
	//pipe.stop();
	//cv::waitKey(10);
}
*/