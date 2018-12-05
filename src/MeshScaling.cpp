
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
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

  if (node["move_once"])
      move_once_ = node["move_once"].as<bool>();

  factor_ = node["factor"].as<std::vector<double>>();
  origin_ = node["origin"].as<std::vector<double>>();

  assert(factor_.size() == 3);
  assert(origin_.size() == 3);
}

void MeshScaling::build_transformation(const double time)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // if move once is specified we stop moving after start time
    if(move_once_ && has_moved_)
      return;

    if(move_once_)
      scaling_mat(1.0);
    else
      scaling_mat(time);

    has_moved_ = true;
  }
}

void MeshScaling::scaling_mat(const double time)
{
  reset(trans_mat_);

  // Build matrix for translating object to cartesian origin
  trans_mat_[0][3] = -origin_[0];
  trans_mat_[1][3] = -origin_[1];
  trans_mat_[2][3] = -origin_[2];

  // Build matrix for scaling object
  trans_mat_type curr_trans_mat_ = {};

  curr_trans_mat_[0][0] = factor_[0]*time;
  curr_trans_mat_[1][1] = factor_[1]*time;
  curr_trans_mat_[2][2] = factor_[2]*time;
  curr_trans_mat_[3][3] = 1.0;

  // composite addition of motions in current group
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);

  // Build matrix for translating object back to its origin
  reset(curr_trans_mat_);
  curr_trans_mat_[0][3] = origin_[0];
  curr_trans_mat_[1][3] = origin_[1];
  curr_trans_mat_[2][3] = origin_[2];

  // composite addition of motions
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);

  for (int r = 0; r < trans_mat_size; r++) {
    for (int c = 0; c < trans_mat_size; c++) {
      std::cout<<trans_mat_[r][c]<<" ";
    } // end for loop - column index
    std::cout<<std::endl;
  } // end for loop - row index

}

} // tioga_nalu
