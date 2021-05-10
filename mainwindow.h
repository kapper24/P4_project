#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qtimer.h>



#include <librealsense2/rs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <pcl/common/geometry.h>
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
#include <pcl/common/pca.h>
#include <chrono>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
class QPushButton;
class QLabel;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();


private slots:
    void on_OpenCloseButton_clicked();
    void on_StartQuitButton_clicked();
    void on_IdleButton_clicked();
    void PCLupdate();
private:
    bool isStarted = false;
    int startClicked = 0;
    QTimer* PCLtimer;
    Ui::MainWindow* ui;
    QPushButton* ui_OpenCloseButton;
    QPushButton* ui_StartQuitButton;
    QPushButton* ui_IdleButton;
    QLabel* ui_GraspGorithmLabel;
    double orientation = 0;
    double diameter = 0;

    std::string readFromPCLPath = "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\MujocoProgram\\ReadfromPCL.txt";
    std::string projectPath = "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\";
    std::string ImageFolderPath = "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\images";
    std::string readFromPythonPath = "C:\\Users\\Melvin\\source\\repos\\kapper24\\P4_project\\ReadfromPython.txt";
   
    enum HandState{
        Open,
        Close,
        Idle
    };
    HandState mjHand = Idle;
    typedef pcl::PointXYZ WSPoint;
    typedef pcl::PointCloud<WSPoint> WSPointCloud;
    typedef WSPointCloud::Ptr WSPointCloudPtr;

    int graspnum = 0;

    float objectMaxX = -10;
    float objectMinX = -60;
    float objectMaxY = -10;
    float objectMinY = -60;

    pcl::PointXYZ minPt;
    pcl::PointXYZ maxPt;
    bool startQuit = true;
    int openclose = 0;
    //Global pcl Variables
    std::string serialnumber_;
   
    rs2::pipeline pipe;
    // det skal måske bruges i extractObject (det er måske bedre til at sortere støj væk)
    struct objectNormal {
        WSPointCloudPtr objects;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
    };

    struct objectSpecs {
        WSPointCloudPtr objectCloud;
        double orientation;
        double diameter;
    };
    struct checkCloud {
        WSPointCloudPtr cloud;
        bool check;
    };


    WSPointCloudPtr points2cloud(rs2::points pts);
    checkCloud filterCloud(WSPointCloudPtr cloud);
    WSPointCloudPtr extractObject(WSPointCloudPtr filteredCloud);
    WSPointCloudPtr getCenterObject(WSPointCloudPtr objects);
    //void saveRGB2File(Mat image, WSPointCloudPtr centerObject);
    objectSpecs fitCylinder(WSPointCloudPtr centerObject);
    objectSpecs fitWine(WSPointCloudPtr centerObject);
    objectSpecs fitCup(WSPointCloudPtr centerObject);
    objectSpecs fitSphere(WSPointCloudPtr centerObject);
};
#endif // MAINWINDOW_H
