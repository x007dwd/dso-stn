/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */

#include "stn/ransac_applications.h"
#include "stn/plane.h"
/*---------------------------------------------------------------
                Aux. functions needed by ransac_detect_3D_planes
 ---------------------------------------------------------------*/
using namespace std;
namespace dso {

template <typename T>
void ransac3Dplane_fit(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &allData,
    const vector<int> &useIndices,
    vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> &fitModels) {

  assert(useIndices.size() == 3);
  plane pn;
  pn.best_line_from_points(allData);
  fitModels.resize(1);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &M = fitModels[0];

  M.setSize(1, 4);
  // for (size_t i = 0; i < 4; i++)
  //   0;
  // M(0, i) = plane.coefs[i];

  // fitModels.clear();
  return;
}

template <typename T>
void ransac3Dplane_distance(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &allData,
    const vector<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> &testModels,
    const T distanceThreshold, unsigned int &out_bestModelIndex,
    std::vector<int> &out_inlierIndices) {
  assert(testModels.size() == 1);
  out_bestModelIndex = 0;
  const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &M = testModels[0];

  assert(M.rows() == 1 && M.cols() == 4);
  plane pn;
  const size_t N = size(allData, 2);
  out_inlierIndices.clear();
  out_inlierIndices.reserve(100);
  for (size_t i = 0; i < N; i++) {
    const double d = pn.distance(allData);
    if (d < distanceThreshold)
      out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
template <typename T>
bool ransac3Dplane_degenerate(
    const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &allData,
    const std::vector<int> &useIndices) {
  // MRPT_UNUSED_PARAM(allData);
  // MRPT_UNUSED_PARAM(useIndices);
  return false;
}

/*---------------------------------------------------------------
                                ransac_detect_3D_planes
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
void ransac_detect_3D_planes(
    const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, s> &pt,
    vector<pair<size_t, TPlane>> &out_detected_planes, const double threshold,
    const size_t min_inliers_for_valid_plane) {

  out_detected_planes.clear();

  if (pt.rows() == 0 || pt.cols() == 0)
    return;

  // ---------------------------------------------
  // For each plane:
  // ---------------------------------------------
  for (;;) {
    std::vector<int> this_best_inliers;
    Eigen::Matrix<NUMTYPE, Eigen::Dynamic, s> this_best_model;

    math::RANSAC_Template<NUMTYPE>::execute(pt, ransac3Dplane_fit,
                                            ransac3Dplane_distance,
                                            ransac3Dplane_degenerate, threshold,
                                            3, // Minimum set of points
                                            this_best_inliers, this_best_model,
                                            true, // Verbose
                                            0.999 // Prob. of good result
                                            );

    // Is this plane good enough?
    if (this_best_inliers.size() >= min_inliers_for_valid_plane) {
      // Add this plane to the output list:
      out_detected_planes.push_back(std::make_pair<size_t, TPlane>(
          this_best_inliers.size(),
          TPlane(this_best_model(0, 0), this_best_model(0, 1),
                 this_best_model(0, 2), this_best_model(0, 3))));

      out_detected_planes.rbegin()->second.unitarize();

      // Discard the selected points so they are not used again for finding
      // subsequent planes:
      pt.removeColumns(this_best_inliers);
    } else {
      break; // Do not search for more planes.
    }
  }
}

// Template explicit instantiations:
// #define EXPLICIT_INST_ransac_detect_3D_planes(_TYPE_)                          \
//   template void BASE_IMPEXP dso::ransac_detect_3D_planes<_TYPE_>(              \
//       const Eigen::Matrix<_TYPE_, Eigen::Dynamic, 1> &pt,                      \
//       vector<pair<size_t, TPlane>> &out_detected_planes,                       \
//       const double threshold, const size_t min_inliers_for_valid_plane);
//
// EXPLICIT_INST_ransac_detect_3D_planes(float)
//     EXPLICIT_INST_ransac_detect_3D_planes(double)
// #ifdef HAVE_LONG_DOUBLE
//         EXPLICIT_INST_ransac_detect_3D_planes(long double)
// #endif

} // end namespace
