#ifndef MESHROTATION_H
#define MESHROTATION_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshRotation : public MotionBase
{
public:
  MeshRotation(const YAML::Node&);

  virtual ~MeshRotation() {}

  virtual void build_transformation(const double);

  /** Function to compute motion-specific velocity
   *
   * @param[in] time           Current time
   * @param[in] comp_trans_mat Transformation matrix
   *                           for points other than xyz
   * @param[in] xyz            3D coordinates of a point
   */
  virtual threeD_vec_type compute_velocity(
    double time,
    const trans_mat_type& comp_trans,
    double* xyz );

private:
  MeshRotation() = delete;
  MeshRotation(const MeshRotation&) = delete;

  void load(const YAML::Node&);

  void rotation_mat(const double);

  threeD_vec_type origin_;
  threeD_vec_type axis_;

  double omega_{0.0};
  double angle_{0.0};

  bool use_omega_;
};


} // tioga_nalu

#endif /* MESHROTATION_H */
