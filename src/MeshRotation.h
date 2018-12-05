#ifndef MESHROTATION_H
#define MESHROTATION_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshRotation : public MotionBase
{
public:
  MeshRotation(
    stk::mesh::MetaData&,
    const YAML::Node&);

  virtual ~MeshRotation() {}

  virtual void build_transformation(const double);

private:
  MeshRotation() = delete;
  MeshRotation(const MeshRotation&) = delete;

  void load(const YAML::Node&);

  void rotation_mat(const double);

  std::vector<double> origin_;
  std::vector<double> axis_;

  double omega_{0.0};
  double angle_{0.0};

  bool use_omega_;
};


} // tioga_nalu

#endif /* MESHROTATION_H */
