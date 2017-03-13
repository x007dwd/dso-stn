#ifndef MARGIN_KEY_FRAME_H_
#define MARGIN_KEY_FRAME_H_

#include "FullSystem/HessianBlocks.h"
#include <algorithm>
#include <caffe/caffe.hpp>
#include <iosfwd>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

using namespace caffe; // NOLINT(build/namespaces)
using namespace cv;
namespace dso {

class marginNetKF {
  marginNetKF(const string &model_file);
  void UpdateModel(FrameHessian *fh, bool label);
  bool IsFrameMG(FrameHessian *fh);

private:
  void NetInit();

  std::shared_ptr<Net<float>> net_;
};

class dataAssEval {
  dataAssEval();
  void UpdateModel(PointHessian *ph, bool label);
  void initModel();
};
}

#endif /* end of include guard: MARGINKAYFRAME_H_ */
