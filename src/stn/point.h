#include "FullSystem/HessianBlocks.h"
#include "FullSystem/ImmaturePoint.h"
#include "util/NumType.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/***
点的目的是
1. 转化格式，需要从原始的地图中的点，转化格式为pcl的格式
2. 给点filter，减少计算量
3. 用于显示

***/
namespace dso {

class point {
public:
  point();

  void pointInit();
  void passRun(const pcl::PointIndices::Ptr indices);
  void normalEst(const pcl::PointIndices::Ptr indices);
  void voxelGrid(const pcl::PointIndices::Ptr indices);
  void addFrame(Vec3 pos);
  void appendCloud(pcl::PointCloud<PointT>::Ptr src,
                   pcl::PointCloud<PointT>::Ptr dst);
  void swap(pcl::PointCloud<PointT>::Ptr ptr1,
            pcl::PointCloud<PointT>::Ptr ptr2);
  int readPLY(const std::string &filename);
  int writePLY(const std::string &filename,
               const pcl::PointCloud<PointT>::Ptr cloud);
  int writePCD(const std::string &filename,
               const pcl::PointCloud<PointT>::Ptr cloud);
  int readPCD(const std::string &filename);
  void retrivePointFromMap(FrameHessian *fh, CalibHessian *HCalib);

private:
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
  pcl::PassThrough<PointT> pass;
  pcl::VoxelGrid<PointT> sor;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree;
  int ptIndices;
  pcl::PointCloud<PointT>::Ptr frameCloud;
};
}
