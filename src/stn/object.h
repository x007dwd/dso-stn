#ifndef OBJECT_H
#define OBJECT_H

#include "FullSystem/HessianBlocks.h"
#include "util/NumType.h"
#include <opencv2/core/core.hpp>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// using namespace cv;
using namespace std;

namespace dso {

class trajectory {
public:
  trajectory();
  void addPoint(const SE3 &pose);

private:
  std::vector<SE3> PosetoWorld;
};

class poseGraph {
public:
  void loopDetect(FrameHessian *fh);
  void PoseOptimize();

private:
  trajectory *traj;
};

class object {
public:
  object(float ptRatio);
  vector<PointHessian *> ptSet;
  vector<Vec3> ptVec;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> ptMat;
  void updatePose();
  void addPoint(PointHessian *ph);
  void addPoint(const Vec3 &pt);

private:
  SE3 Pose;
  float ratioInbox;
};

class dynamicalObj : public object {
public:
  dynamicalObj(const Vec3 &pos, double height, double width);
  void UpdateModel(FrameHessian *ph, const cv::Mat &label);
  bool trackDynamicObj(FrameHessian *ph, cv::Mat &ObjMark);
  void predictNext(cv::Mat &ObjMark);
  void predictMotion();

private:
  Vec3 spd;
  double height, width;
  Vec3 pos;
  trajectory obj_traj;
  trajectory camera_traj;
};
}

#endif /* end of include guard: OBJECT_H */
