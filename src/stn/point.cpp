#include "stn/point.h"

/***
点的目的是
1. 转化格式，需要从原始的地图中的点，转化格式为pcl的格式
2. 给点filter，减少计算量
3. 用于显示

***/
namespace dso {
typedef pcl::PointXYZ PointT;
point::point() {
  cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  cloud_normals =
      pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  cloud_filtered = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  tree = pcl::search::KdTree<PointT>::Ptr(new pcl::search::KdTree<PointT>);
}

void point::swap(pcl::PointCloud<PointT>::Ptr ptr1,
                 pcl::PointCloud<PointT>::Ptr ptr2) {
  pcl::PointCloud<PointT>::Ptr ptrtmp;
  ptrtmp = ptr1;
  ptr1 = ptr2;
  ptr2 = ptrtmp;
}
void point::pointInit() {
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0, 1.5);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  ne.setSearchMethod(tree);
  ne.setKSearch(50);

  sor.setInputCloud(cloud);
  pass.setInputCloud(cloud);
  ne.setInputCloud(cloud);
}

void point::passRun(pcl::PointIndices indices) {
  std::cout << "running a point cloud filter" << std::endl;
  // pass.setIndices(indices);
  pass.filter(*cloud_filtered);
  std::cout << "the point number before filted is "
            << cloud->height * cloud->width << std::endl;
  std::cout << "the point number after filted is "
            << cloud_filtered->height * cloud_filtered->width << std::endl;
}

void point::voxelGrid(pcl::PointIndices indices) {

  // sor.setIndices(indices);
  sor.filter(*cloud_filtered);
}

void point::normalEst(pcl::PointIndices indices) {
  // ne.setIndices(indices);

  ne.compute(*cloud_normals);
}
}
