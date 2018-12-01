#ifndef MOTIONBASE_H
#define MOTIONBASE_H

#include "stk_mesh/base/MetaData.hpp"

#include <cassert>
#include <vector>

namespace tioga_nalu {

class MotionBase
{
public:
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
  std::vector<std::vector<double>> add_motion(
    const std::vector<std::vector<double>>& motionL,
    const std::vector<std::vector<double>>& motionR);

  const std::vector<std::vector<double>>& get_trans_mat() const {
    return trans_mat_ ; }

protected:
  void reset_trans_mat(){
    for (auto &vec : trans_mat_)
        std::fill(vec.begin(), vec.end(), 0.0); }

  stk::mesh::MetaData& meta_;

  /** Transformation matrix
   *
   * A 4x4 matrix that combines rotation, translation, scaling,
   * allowing representation of all affine transformations
   */
  std::vector<std::vector<double>> trans_mat_ =
      std::vector<std::vector<double>> (4,std::vector<double>(4,0.0));

private:
    MotionBase() = delete;
    MotionBase(const MotionBase&) = delete;
};


} // tioga_nalu

#endif /* MOTIONBASE_H */
