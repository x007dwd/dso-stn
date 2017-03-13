#ifndef ransac_optimizers_H
#define ransac_optimizers_H

#include <stn/ransac.h>

namespace dso {
using std::vector;

/** Fit a number of 3-D planes to a given point cloud, automatically determining
 * the number of existing planes by means of the provided threshold and minimum
 * number of supporting inliers.
  * \param out_detected_planes The output list of pairs: number of supporting
 * inliers, detected plane.
  * \param threshold The maximum distance between a point and a temptative plane
 * such as the point is considered an inlier.
  * \param min_inliers_for_valid_plane  The minimum number of supporting inliers
 * to consider a plane as valid.
  */
// template <typename NUMTYPE>
// void BASE_IMPEXP ransac_detect_3D_planes(
//     const Eigen::Matrix<NUMTYPE, Eigen::Dynamic, 1> &pt,
//     std::vector<std::pair<size_t, TPlane>> &out_detected_planes,
//     const double threshold, const size_t min_inliers_for_valid_plane = 10);

/** A stub for ransac_detect_3D_planes() with the points given as a
 * mrpt::maps::CPointsMap
  */
// template <class POINTSMAP>
// inline void ransac_detect_3D_planes(
//     const POINTSMAP *points_map,
//     std::vector<std::pair<size_t, TPlane>> &out_detected_planes,
//     const double threshold, const size_t min_inliers_for_valid_plane) {
//   CVectorFloat xs, ys, zs;
//   points_map->getAllPoints(xs, ys, zs);
//   ransac_detect_3D_planes(xs, ys, zs, out_detected_planes, threshold,
//                           min_inliers_for_valid_plane);
// }

/** @} */
/** @} */ // end of grouping

} // End of namespace

#endif
