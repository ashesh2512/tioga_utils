#ifndef MESHSCALING_H
#define MESHSCALING_H

#include "MotionBase.h"

namespace tioga_nalu {

class MeshScaling : public MotionBase
{
public:
  MeshScaling(
    stk::mesh::MetaData&,
    const YAML::Node&);

  virtual ~MeshScaling() {}

  virtual void build_transformation(const double);

private:
  MeshScaling() = delete;
  MeshScaling(const MeshScaling&) = delete;

  void load(const YAML::Node&);

  void scaling_mat(const double);

  std::vector<double> factor_;
  std::vector<double> origin_;
};


} // tioga_nalu

#endif /* MESHSCALING_H */
