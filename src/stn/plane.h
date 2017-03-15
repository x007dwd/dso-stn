#include "FullSystem/HessianBlocks.h"
#include "string"
#include "util/NumType.h"
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <utility>
#include <vector>

using namespace std;
namespace dso {

#define NormProductTH 1.0
#define PtNumTH 20

class plane;
class planePoint {
public:
  planePoint(Vec3 pt3);
  PointHessian *ph;
  plane *pnStruct;
  Vec3 pt;

private:
};

class plane {
public:
  plane(Vec3 normv = Vec3(), Vec3 center = Vec3())
      : normvec(normv), res(0), centroid(center), ptNum(0){};
  // plane(const plane &other);
  // plane &operator=(const plane &other);
  Vec3 calcNormalVector(const vector<Vec3> ptSet);
  bool isInPlannar(const Vec3 &pt);
  double distance(const Eigen::MatrixXd data);
  bool addPlanarPoint(planePoint *ptp);
  void best_plane_from_points(const std::vector<Vec3> &c);
  void best_plane_from_points(const std::vector<planePoint> &c);
  void best_plane_from_points(Eigen::MatrixXd &coord);

  void EigenMatrix2PCLPoint(const Eigen::MatrixXd &em,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  int segment();

  void seg_init(float th, int method = pcl::SACMODEL_PLANE);
  void run_seg();
  int matrix_transform(const Eigen::Affine3f &T);

  void pcl_view(const std::string &viewer_str);
  std::pair<Vec3, Vec3> best_line_from_points(const std::vector<Vec3> &c);
  Vec3 normvec;
  Vec3 centroid;

private:
  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  vector<planePoint *> ptSet;
  int ptNum;
  float res;
  int sacmodel_types;
  bool isNumPtEnough() { return ptNum >= PtNumTH; }
  bool isPtGood(const Vec3 &pt) { return normvec.dot(pt) < NormProductTH; }
  bool updateRes(const Vec3 &pt) { res += normvec.dot(pt); }
};
}
