#ifndef MOTIONBASE_H
#define MOTIONBASE_H

#include "yaml-cpp/yaml.h"

#include <cassert>
#include <cfloat>

namespace tioga_nalu {

class MotionBase
{
public:
  //! Define matrix type alias
  static constexpr int trans_mat_size = 4;
  using trans_mat_type = std::array<std::array<double, trans_mat_size>, trans_mat_size>;

  //! Define 3D vector type alias
  static constexpr int threeD_vec_size = 3;
  using threeD_vec_type = std::array<double, threeD_vec_size>;

  MotionBase() {}

  virtual ~MotionBase() {}

  virtual void build_transformation(const double, const double* = nullptr) = 0;

  /** Function to compute motion-specific velocity
   *
   * @param[in] time           Current time
   * @param[in] comp_trans_mat Transformation matrix
   *                           for points other than xyz
   * @param[in] xyz            Transformed coordinates
   */
  virtual threeD_vec_type compute_velocity(
    double time,
    const trans_mat_type& comp_trans,
    double* xyz ) = 0;

  /** Composite addition of motions
   *
   * @param[in] motionL Left matrix in composite transformation of matrices
   * @param[in] motionR Right matrix in composite transformation of matrices
   * @return    4x4 matrix representing composite addition of motions
   */
  trans_mat_type add_motion(
    const trans_mat_type& motionL,
    const trans_mat_type& motionR);

  const trans_mat_type& get_trans_mat() const {
    return trans_mat_; }

  static const trans_mat_type identity_mat_;

protected:
  void reset_mat(trans_mat_type& mat) {
    mat = identity_mat_; }

  /** Transformation matrix
   *
   * A 4x4 matrix that combines rotation, translation, scaling,
   * allowing representation of all affine transformations
   */
  trans_mat_type trans_mat_ = identity_mat_;

  double start_time_{0.0};
  double end_time_{DBL_MAX};
  const double eps_{1e-14};

private:
    MotionBase(const MotionBase&) = delete;
};

} // tioga_nalu

#endif /* MOTIONBASE_H */
