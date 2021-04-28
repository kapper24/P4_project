#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <qtimer.h>

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
    void PCLupdate();
private:
    QTimer* PCLtimer;
    Ui::MainWindow* ui;
    QPushButton* ui_OpenCloseButton;
    QPushButton* ui_StartQuitButton;
    QLabel* ui_GraspGorithmLabel;
};
#endif // MAINWINDOW_H
