#ifndef MESHPULSATINGSPHERE_H
#define MESHPULSATINGSPHERE_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshPulsatingSphere : public MotionBase
{
public:
  MeshPulsatingSphere(const YAML::Node&);

  virtual ~MeshPulsatingSphere() {}

  virtual void build_transformation(const double);

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
  MeshPulsatingSphere() = delete;
  MeshPulsatingSphere(const MeshPulsatingSphere&) = delete;

  void load(const YAML::Node&);

  void scaling_mat(const double);

  double radius_;
  double amplitude_{1.0};
  double frequency_{1.0};

  threeD_vec_type origin_;
};

} // tioga_nalu

#endif /* MESHPULSATINGSPHERE_H */
