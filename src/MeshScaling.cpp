
#include "MeshScaling.h"

#include <cmath>

namespace tioga_nalu {

MeshScaling::MeshScaling(
  stk::mesh::MetaData& meta,
  const YAML::Node& node )
: MotionBase(meta)
{
    load(node);
}

void MeshScaling::load(const YAML::Node& node)
{
    factor_ = node["factor"].as<std::vector<double>>();
    origin_ = node["origin"].as<std::vector<double>>();

    assert(factor_.size() == 3);
    assert(origin_.size() == 3);
}

void MeshScaling::build_transformation(double time)
{
  reset_trans_mat();

  // Build matrix for translating object to cartesian origin
  trans_mat_ =
    { { 1.0, 0.0, 0.0, -origin_[0] },
      { 0.0, 1.0, 0.0, -origin_[1] },
      { 0.0, 0.0, 1.0, -origin_[2] },
      { 0.0, 0.0, 0.0,  1.0        } };

  // Build matrix for scaling object
  std::vector<std::vector<double>> curr_trans_mat_ =
  { { factor_[0]*time,             0.0,             0.0, 0.0 },
    {             0.0, factor_[1]*time,             0.0, 0.0 },
    {             0.0,             0.0, factor_[2]*time, 0.0 },
    {             0.0,             0.0,             0.0, 1.0 } };

  // composite addition of motions in current group
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);

  // Build matrix for translating object back to its origin
  curr_trans_mat_ =
    { { 1.0, 0.0, 0.0, origin_[0] },
      { 0.0, 1.0, 0.0, origin_[1] },
      { 0.0, 0.0, 1.0, origin_[2] },
      { 0.0, 0.0, 0.0, 1.0        } };

  // composite addition of motions
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);
}

} // tioga_nalu
