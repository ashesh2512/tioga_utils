
#include "MotionTranslation.h"

#include <cmath>

namespace tioga_nalu {

MotionTranslation::MotionTranslation(const YAML::Node& node)
  : MotionBase()
{
  load(node);
}

void MotionTranslation::load(const YAML::Node& node)
{
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

  // rotation could be based on angular velocity or angle
  if(node["velocity"]){
    use_velocity_ = true;
    velocity_ = node["velocity"].as<threeD_vec_type>();
  }
  if(node["displacement"])
  {
    use_velocity_ = false;
    displacement_ = node["displacement"].as<threeD_vec_type>();
  }
  // ensure only 1 of velocity or displacement vector is specified
  assert(velocity_.size() + displacement_.size() == 3);
}

void MotionTranslation::build_transformation(
  const double time,
  const double* xyz)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // determine translation based on user defined input
    if (use_velocity_)
    {
      threeD_vec_type curr_disp = {};
      for (int d=0; d < threeD_vec_size; d++)
        curr_disp[d] = velocity_[d]*(time-start_time_);

      translation_mat(curr_disp);
    }
    else
      translation_mat(displacement_);
  }
}

void MotionTranslation::translation_mat(const threeD_vec_type& curr_disp)
{
  reset_mat(trans_mat_);

  // Build matrix for translating object
  trans_mat_[0][3] = curr_disp[0];
  trans_mat_[1][3] = curr_disp[1];
  trans_mat_[2][3] = curr_disp[2];
}

MotionBase::threeD_vec_type MotionTranslation::compute_velocity(
  double time,
  const trans_mat_type& comp_trans,
  double* xyz )
{
  threeD_vec_type vel = {};

  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
    vel = velocity_;

  return vel;
}

} // tioga_nalu
