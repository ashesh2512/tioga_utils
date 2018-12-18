#ifndef MOTIONTRANSLATION_H
#define MOTIONTRANSLATION_H

#include "MotionBase.h"

namespace tioga_nalu {

class MotionTranslation : public MotionBase
{
public:
  MotionTranslation(const YAML::Node&);

  virtual ~MotionTranslation() {}

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
  MotionTranslation() = delete;
  MotionTranslation(const MotionTranslation&) = delete;

  void load(const YAML::Node&);

  void translation_mat(const threeD_vec_type&);

  threeD_vec_type displacement_;
  threeD_vec_type velocity_;

  bool use_velocity_;
};


} // tioga_nalu

#endif /* MOTIONTRANSLATION_H */
