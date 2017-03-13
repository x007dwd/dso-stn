#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "util/NumType.h"
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
  void passRun(pcl::PointIndices indices);
  void normalEst(pcl::PointIndices indices);
  void voxelGrid(pcl::PointIndices indices);

  void swap(pcl::PointCloud<PointT>::Ptr ptr1,
            pcl::PointCloud<PointT>::Ptr ptr2);

private:
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
  pcl::PassThrough<PointT> pass;
  pcl::VoxelGrid<PointT> sor;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree;
};
}
