
#include "MeshTranslation.h"

#include <cmath>

namespace tioga_nalu {

MeshTranslation::MeshTranslation(
  stk::mesh::MetaData& meta,
  const YAML::Node& node )
: MotionBase(meta)
{
    load(node);
}

void MeshTranslation::load(const YAML::Node& node)
{
    direction_ = node["direction"].as<std::vector<double>>();
    assert(direction_.size() == 3);
}

void MeshTranslation::build_transformation(double time)
{
  reset(trans_mat_);

  // Build matrix for translating object to cartesian origin
  for(int in = 0; in < 4; in++)
    trans_mat_[in][in] = 1.0;

  trans_mat_[0][3] = direction_[0]*time;
  trans_mat_[1][3] = direction_[1]*time;
  trans_mat_[2][3] = direction_[2]*time;
}

} // tioga_nalu
