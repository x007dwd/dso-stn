#include "stn/point.h"

/***
点的目的是
1. 转化格式，需要从原始的地图中的点，转化格式为pcl的格式
2. 给点filter，减少计算量
3. 用于显示

***/
namespace dso {

point::point() : ptIndices(0) {
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

int point::readPLY(const std::string &filename) {
  if (pcl::io::loadPLYFile(filename.c_str(), *cloud) < 0) {
    std::cout << "Error loading point cloud " << filename << std::endl
              << std::endl;
    return -1;
  }
  return 0;
}

int point::writePLY(const std::string &filename,
                    const pcl::PointCloud<PointT>::Ptr cloud) {
  if (pcl::io::savePLYFileASCII(filename.c_str(), *cloud) < 0) {
    std::cout << " Error loading point cloud " << filename << std::endl
              << std::endl;
  }
}

int point::writePCD(const std::string &filename,
                    const pcl::PointCloud<PointT>::Ptr cloud) {
  if (pcl::io::savePCDFileASCII(filename.c_str(), *cloud) < 0) {
    std::cout << " Error loading point cloud " << filename << std::endl
              << std::endl;
  }
}
int point::readPCD(const std::string &filename) {
  if (pcl::io::loadPCDFile(filename.c_str(), *cloud) < 0) {
    std::cout << "Error loading point cloud " << filename << std::endl
              << std::endl;
    // showHelp(argv[0]);
    return -1;
  }
  return 0;
}
// 实际上整体旋转和每个点分别旋转是一样的，并没有加速的方法。
// 因此 所有的点都放在一起是可以的，每一帧的点分别平滑是可以的
// 这么说吧其实点的数量是很少的，因为大部分点都是边缘点，因此点其实比较少。
// 没必要使用很多在上边，主要的问题是需要把双目出现的深度点有很多，是密集的，需要考虑
void point::addFrame(Vec3 pos) {
  pcl::PointCloud<PointT>::Ptr tmpcloud(new pcl::PointCloud<PointT>);
  frameCloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

  tmpcloud->points.push_back(PointT(pos[0], pos[1], pos[2]));
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(tmpcloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*frameCloud);
  appendCloud(frameCloud, cloud);
}
void point::appendCloud(pcl::PointCloud<PointT>::Ptr src,
                        pcl::PointCloud<PointT>::Ptr dst) {
  dst->points.insert(std::end(dst->points), std::begin(src->points),
                     std::end(src->points));
}

void point::passRun(const pcl::PointIndices::Ptr indices) {
  std::cout << "running a point cloud filter" << std::endl;
  pass.setIndices(indices);
  pass.filter(*cloud_filtered);
  std::cout << "the point number before filted is "
            << cloud->height * cloud->width << std::endl;
  std::cout << "the point number after filted is "
            << cloud_filtered->height * cloud_filtered->width << std::endl;
}

void point::voxelGrid(const pcl::PointIndices::Ptr indices) {

  sor.setIndices(indices);
  sor.filter(*cloud_filtered);
}

void point::normalEst(const pcl::PointIndices::Ptr indices) {
  ne.setIndices(indices);
  ne.compute(*cloud_normals);
}
}
