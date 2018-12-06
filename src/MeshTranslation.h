#ifndef MESHTRANSLATION_H
#define MESHTRANSLATION_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshTranslation : public MotionBase
{
public:
  MeshTranslation(const YAML::Node&);

  virtual ~MeshTranslation() {}

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
  MeshTranslation() = delete;
  MeshTranslation(const MeshTranslation&) = delete;

  void load(const YAML::Node&);

  void translation_mat(const threeD_vec_type&);

  threeD_vec_type displacement_;
  threeD_vec_type velocity_;

  bool use_velocity_;
};


} // tioga_nalu

#endif /* MESHTRANSLATION_H */
