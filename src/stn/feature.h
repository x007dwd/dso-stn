#ifndef _CNNFEATURE_H_
#define _CNNFEATURE_H_

#include <caffe/caffe.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace caffe; // NOLINT(build/namespaces)
using std::string;

/* Pair (label, confidence) representing a prediction. */
typedef std::pair<string, float> Prediction;
namespace dso {
class CNNFeature {
public:
  CNNFeature(const string &model_file, const string &trained_file,
             const string &mean_file);

  // std::vector<Prediction> Classify(const cv::Mat& img, int N = 5);
  Blob<float> *ExtractForwardedLayerData(string layer_name);
  std::vector<float> Predict(const cv::Mat &img);

private:
  void SetMean(const string &mean_file);
  void WrapInputLayer(std::vector<cv::Mat> *input_channels);
  void WarpMidLayers(std::vector<cv::Mat> *layer_channels, string layer_name);
  void Preprocess(const cv::Mat &img, std::vector<cv::Mat> *input_channels);

private:
  shared_ptr<Net<float>> net_;
  cv::Size input_geometry_;
  int num_channels_;
  cv::Mat mean_;
  std::vector<string> labels_;
};
}
#endif
