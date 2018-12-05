
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
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

  if (node["move_once"])
      move_once_ = node["move_once"].as<bool>();

  // rotation could be based on angular velocity or angle
  if(node["velocity"]){
    use_velocity_ = true;
    velocity_ = node["velocity"].as<std::vector<double>>();
  }
  if(node["displacement"])
  {
    use_velocity_ = false;
    displacement_ = node["displacement"].as<std::vector<double>>();
  }
  // ensure only 1 of velocity or displacement vector is specified
  assert(velocity_.size() + displacement_.size() == 3);
}

void MeshTranslation::build_transformation(const double time)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // if move once is specified we stop moving after start time
    if(move_once_ && has_moved_)
      return;

    // determine current displacement
    std::vector<double> curr_disp = {0.0,0.0,0.0};
    if (use_velocity_)
      for (int d=0; d < meta_.spatial_dimension(); d++)
        curr_disp[d] = velocity_[d]*(time-start_time_);
    else
      curr_disp = displacement_;

    translation_mat(curr_disp);
    has_moved_ = true;
  }
}

void MeshTranslation::translation_mat(const std::vector<double>& curr_disp)
{
  reset(trans_mat_);

  // Build matrix for translating object
  trans_mat_[0][3] = curr_disp[0];
  trans_mat_[1][3] = curr_disp[1];
  trans_mat_[2][3] = curr_disp[2];
}

} // tioga_nalu
