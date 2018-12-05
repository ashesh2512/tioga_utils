#ifndef MOTIONBASE_H
#define MOTIONBASE_H

#include "stk_mesh/base/MetaData.hpp"

#include "yaml-cpp/yaml.h"

#include <cassert>
#include <float.h>

namespace tioga_nalu {

class MotionBase
{
public:
  /** Define matrix type alias and size for transformation matrices
   */
  static constexpr int trans_mat_size = 4;
  using trans_mat_type = std::array<std::array<double, trans_mat_size>, trans_mat_size>;

  MotionBase(stk::mesh::MetaData &meta)
    : meta_(meta) {}

  virtual ~MotionBase() {}

  virtual void build_transformation(double) = 0;

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

protected:
  void reset(trans_mat_type& mat) {
    mat = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}}; }

  stk::mesh::MetaData& meta_;

  /** Transformation matrix
   *
   * A 4x4 matrix that combines rotation, translation, scaling,
   * allowing representation of all affine transformations
   */
  trans_mat_type trans_mat_ = {{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}}};

  double start_time_{0.0};
  double end_time_{DBL_MAX};
  const double eps_{1e-14};

  // booleans to keep track of 1 time mesh motion at t>0.0
  bool move_once_{false};
  bool has_moved_{false};

private:
    MotionBase() = delete;
    MotionBase(const MotionBase&) = delete;
};

} // tioga_nalu

#endif /* MOTIONBASE_H */
