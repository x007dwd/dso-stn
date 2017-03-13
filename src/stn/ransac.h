/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#ifndef mrpt_ransac_H
#define mrpt_ransac_H
#include "Eigen/Core"
#include <set>
using Eigen::Dynamic;
using Eigen::Matrix;
namespace dso {

/*implementation defined*/

/** @addtogroup ransac_grp RANSAC and other model fitting algorithms
  * \ingroup mrpt_base_grp
  * @{ */

/** A generic RANSAC implementation with models as matrices.
  *  See \a RANSAC_Template::execute for more info on usage.
  *  \sa math::ModelSearch, a more versatile RANSAC implementation where models
 * can be anything else, not only matrices.
  */
template <typename NUMTYPE> class RANSAC_Template {
public:
  /** The type of the function passed to dso::ransac - See the documentation for
   * that method for more info. */
  typedef void (*TRansacFitFunctor)(
      const Eigen::Matrix<NUMTYPE, Dynamic, Dynamic> &allData,
      const std::vector<int> &useIndices,
      std::vector<Eigen::Matrix<NUMTYPE, Dynamic, Dynamic>> &fitModels);

  /** The type of the function passed to std::math::ransac  - See the
   * documentation for that method for more info. */
  typedef void (*TRansacDistanceFunctor)(
      const Eigen::Matrix<NUMTYPE, Dynamic, Dynamic> &allData,
      const std::vector<Eigen::Matrix<NUMTYPE, Dynamic, Dynamic>> &testModels,
      const NUMTYPE distanceThreshold, unsigned int &out_bestModelIndex,
      std::vector<int> &out_inlierIndices);

  /** The type of the function passed to std::math::ransac  - See the
   * documentation for that method for more info. */
  typedef bool (*TRansacDegenerateFunctor)(
      const Eigen::Matrix<NUMTYPE, Dynamic, Dynamic> &allData,
      const std::vector<int> &useIndices);

  /** An implementation of the RANSAC algorithm for robust fitting of models to
   * data.
    *
    *  \param data A DxN matrix with all the observed data. D is the
   * dimensionality of data points and N the number of points.
    *  \param
    *
    *  This implementation is highly inspired on Peter Kovesi's MATLAB scripts
   * (http://www.csse.uwa.edu.au/~pk).
    * \return false if no good solution can be found, true on success.
    */
  static bool
  execute(const Eigen::Matrix<NUMTYPE, Dynamic, Dynamic> &data,
          TRansacFitFunctor fit_func, TRansacDistanceFunctor dist_func,
          TRansacDegenerateFunctor degen_func, const double distanceThreshold,
          const unsigned int minimumSizeSamplesToFit,
          std::vector<int> &out_best_inliers,
          Eigen::Matrix<NUMTYPE, Dynamic, Dynamic> &out_best_model,
          bool verbose = false, const double prob_good_sample = 0.999,
          const size_t maxIter = 2000);

}; // end class

typedef RANSAC_Template<double>
    RANSAC; //!< The default instance of RANSAC, for double type

/** @} */

} // End of namespace

#endif
