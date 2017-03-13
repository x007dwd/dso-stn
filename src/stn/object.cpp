#include "stn/object.h"
#include "stn/marginKF.h"
#include <Eigen/Dense>
using namespace Eigen;
namespace dso {

trajectory::trajectory() {}
void trajectory::addPoint(const SE3 &pose) { PosetoWorld.push_back(pose); }
void poseGraph::loopDetect(FrameHessian *fh) {}
void poseGraph::PoseOptimize() {}

object::object(float ptRatio) : ratioInbox(ptRatio) {
  Matrix4d R = Matrix4d::Identity();
  ptMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>(3, 0);
  Pose = SE3(R);
}
void object::addPoint(PointHessian *ph) {
  ptSet.push_back(ph);

  updatePose();
}

void object::addPoint(const Vec3 &pt) {
  ptVec.push_back(pt);
  ptMat.conservativeResize(ptMat.rows(), ptMat.cols() + 1);
  ptMat.col(ptMat.cols() - 1) = pt;
  // updatePose();
}

void object::updatePose() {
  Vec3 center(ptMat.row(0).mean(), ptMat.row(1).mean(), ptMat.row(2).mean());

  std::cout << "pt size" << ptMat.cols() << std::endl;
  std::cout << "center pos" << center[0] << "\t" << center[1] << "\t"
            << center[2] << "\t" << std::endl;
}

dynamicalObj::dynamicalObj(const Vec3 &pos, double height, double width)
    : object(0.8), pos(pos), height(height), width(width) {
  spd = Vec3(0, 0, 0);
}
void dynamicalObj::UpdateModel(FrameHessian *fh, const Mat &label) {}
}
