#ifndef MOTIONSCALING_H
#define MOTIONSCALING_H

#include "MotionBase.h"

namespace tioga_nalu {

class MotionScaling : public MotionBase
{
public:
  MotionScaling(const YAML::Node&);

  virtual ~MotionScaling() {}

  virtual void build_transformation(const double, const double* = nullptr);

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
    double* xyz );

private:
  MotionScaling() = delete;
  MotionScaling(const MotionScaling&) = delete;

  void load(const YAML::Node&);

  void scaling_mat(const threeD_vec_type&);

  threeD_vec_type factor_;
  threeD_vec_type velocity_;
  threeD_vec_type origin_;

  bool use_velocity_;
};


} // tioga_nalu

#endif /* MOTIONSCALING_H */
