// 
// create 2021/12/30
// by 陶先童
// taoxiantong@qq.com
// 
#include <pcl/visualization/pcl_visualizer.h>

#include <math.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

namespace ToolPlot{


typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualizer;

PCLVisualizer createViewer(std::string title);
void plotCube(PCLVisualizer &viewer, const Eigen::Vector3d &position,
                    const Eigen::Quaterniond &rotation, double width,
                    double height, double depth, const std::string &name,
                    const Eigen::Vector3d &rgb) ;

void plotHand3D(PCLVisualizer &viewer, Eigen::Vector3d position_, Eigen::Matrix3d orientation_,
                      double outer_diameter, double finger_width,
                      double hand_depth, double hand_height, int idx,
                      const Eigen::Vector3d &rgb);

void plotHand3D(PCLVisualizer &viewer, Eigen::Vector3d position_, Eigen::Matrix3d orientation_,
                      double outer_diameter, double finger_width,
                      double hand_depth, double hand_height, int idx,
                      const Eigen::Vector3d &rgb) {

  const double hw = 0.5 * outer_diameter;
  const double base_depth = 0.02;
  const double approach_depth = 0.07;

  Eigen::Vector3d left_bottom =
      position_ - (hw - 0.5 * finger_width) * orientation_.col(1);
  Eigen::Vector3d right_bottom =
      position_ + (hw - 0.5 * finger_width) * orientation_.col(1);
  Eigen::VectorXd left_center =
      left_bottom + 0.5 * hand_depth * orientation_.col(0);
  Eigen::VectorXd right_center =
      right_bottom + 0.5 * hand_depth * orientation_.col(0);
  Eigen::Vector3d base_center = left_bottom +
                                0.5 * (right_bottom - left_bottom) -
                                0.01 * orientation_.col(0);
  Eigen::Vector3d approach_center = base_center - 0.04 * orientation_.col(0);

  const Eigen::Quaterniond quat(orientation_);
  const std::string num = std::to_string(idx);

  plotCube(viewer, left_center, quat, hand_depth, finger_width, hand_height,
           "left_finger_" + num, rgb);
  plotCube(viewer, right_center, quat, hand_depth, finger_width, hand_height,
           "right_finger_" + num, rgb);
  plotCube(viewer, base_center, quat, base_depth, outer_diameter, hand_height,
           "base_" + num, rgb);
  plotCube(viewer, approach_center, quat, approach_depth, finger_width,
           0.5 * hand_height, "approach_" + num, rgb);
}

void plotCube(PCLVisualizer &viewer, const Eigen::Vector3d &position,
                    const Eigen::Quaterniond &rotation, double width,
                    double height, double depth, const std::string &name,
                    const Eigen::Vector3d &rgb) {
  viewer->addCube(position.cast<float>(), rotation.cast<float>(), width, height,
                  depth, name);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                      rgb(0), rgb(1), rgb(2), name);
  viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, 0.25, name);
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void) {
  pcl::visualization::PCLVisualizer *viewer =
      static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
  if (event.getKeySym() == "a" && event.keyDown()) {
    if (viewer->contains("ref")) {
      viewer->removeCoordinateSystem("ref");
    } else {
      viewer->addCoordinateSystem(0.1, "ref");
    }
  }
}

PCLVisualizer createViewer(std::string title) {
  PCLVisualizer viewer(new pcl::visualization::PCLVisualizer(title));
  viewer->setPosition(0, 0);
  viewer->setSize(640, 480);
  viewer->setBackgroundColor(1.0, 1.0, 1.0);
  viewer->registerKeyboardCallback(&keyboardEventOccurred, (void *)viewer.get());

  return viewer;
}

}

// int main(int argc, char * argv[]){
//     Eigen::Vector3d position_ (0.0, 0.0, 0.15);
//     Eigen::Quaterniond Q(1., 0.0,  0.0, 0.0);
//     Eigen::Matrix3d orientation_(Q);

//     double outer_diameter   = argc>=1 ? std::atof(argv[1]) : 0.01;          // 控制抓取宽度
//     double finger_width     = 0.025;            // 指的宽度
//     double hand_depth       = 0.06;             // 指的长度
//     double hand_height      = 0.02;             // 指的厚度
//     int idx                 = 0;
//     const Eigen::Vector3d rgb(1.0, 0, 0);
//     std::cout<<"input: .exe 0.2"<<std::endl;
//     PCLVisualizer view = createViewer("hand");
//     plotHand3D(view, position_, orientation_, outer_diameter, finger_width, hand_depth, hand_height,idx, rgb);

//     view->addCoordinateSystem(0.3);
//     view->spin();
//     return 1;
// }