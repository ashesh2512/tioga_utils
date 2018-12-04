
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
  reset(trans_mat_);

  // Build matrix for translating object to cartesian origin
  trans_mat_[0][0] = 1.0; trans_mat_[0][3] = -origin_[0];
  trans_mat_[1][1] = 1.0; trans_mat_[1][3] = -origin_[1];
  trans_mat_[2][2] = 1.0; trans_mat_[2][3] = -origin_[2];
  trans_mat_[3][3] = 1.0;

  // Build matrix for scaling object
  MotionBase::trans_mat_type curr_trans_mat_ = {};

  curr_trans_mat_[0][0] = factor_[0]*time;
  curr_trans_mat_[1][1] = factor_[1]*time;
  curr_trans_mat_[2][2] = factor_[2]*time;
  curr_trans_mat_[3][3] = 1.0;

  // composite addition of motions in current group
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);

  // Build matrix for translating object back to its origin
  reset(curr_trans_mat_);
  curr_trans_mat_[0][0] = 1.0; curr_trans_mat_[0][3] = origin_[0];
  curr_trans_mat_[1][1] = 1.0; curr_trans_mat_[1][3] = origin_[1];
  curr_trans_mat_[2][2] = 1.0; curr_trans_mat_[2][3] = origin_[2];
  curr_trans_mat_[3][3] = 1.0;

  // composite addition of motions
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);
}

} // tioga_nalu
