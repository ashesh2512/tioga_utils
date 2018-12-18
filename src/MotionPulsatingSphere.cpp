
#include "MotionPulsatingSphere.h"

#include <cmath>

namespace tioga_nalu {

MotionPulsatingSphere::MotionPulsatingSphere(const YAML::Node& node)
  : MotionBase()
{
  load(node);
}

void MotionPulsatingSphere::load(const YAML::Node& node)
{
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

  if(node["amplitude"])
    amplitude_ = node["amplitude"].as<double>();

  if(node["frequency"])
    frequency_ = node["frequency"].as<double>();

  origin_ = node["origin"].as<threeD_vec_type>();
  assert(origin_.size() == threeD_vec_size);
}

void MotionPulsatingSphere::build_transformation(
  const double time,
  const double* xyz)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
    scaling_mat(time,xyz);
}

void MotionPulsatingSphere::scaling_mat(
  const double time,
  const double* xyz)
{
  reset_mat(trans_mat_);

  double radius = std::sqrt( std::pow(xyz[0]-origin_[0],2)
                            +std::pow(xyz[1]-origin_[1],2)
                            +std::pow(xyz[2]-origin_[2],2));

  double curr_radius = radius + amplitude_*(1 - std::cos(2*M_PI*frequency_*time));

  double uniform_scaling = curr_radius/radius;

  // Build matrix for translating object to cartesian origin
  trans_mat_[0][3] = -origin_[0];
  trans_mat_[1][3] = -origin_[1];
  trans_mat_[2][3] = -origin_[2];

  // Build matrix for scaling object
  trans_mat_type curr_trans_mat_ = {};

  curr_trans_mat_[0][0] = uniform_scaling;
  curr_trans_mat_[1][1] = uniform_scaling;
  curr_trans_mat_[2][2] = uniform_scaling;
  curr_trans_mat_[3][3] = 1.0;

  // composite addition of motions in current group
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);

  // Build matrix for translating object back to its origin
  reset_mat(curr_trans_mat_);
  curr_trans_mat_[0][3] = origin_[0];
  curr_trans_mat_[1][3] = origin_[1];
  curr_trans_mat_[2][3] = origin_[2];

  // composite addition of motions
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);
}

MotionBase::threeD_vec_type MotionPulsatingSphere::compute_velocity(
  double time,
  const trans_mat_type& comp_trans,
  double* xyz )
{
  threeD_vec_type vel = {};

  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    double radius = std::sqrt( std::pow(xyz[0]-origin_[0],2)
                              +std::pow(xyz[1]-origin_[1],2)
                              +std::pow(xyz[2]-origin_[2],2));

    double pulsating_velocity =
      amplitude_ * std::sin(2*M_PI*frequency_*time) * 2*M_PI*frequency_ / radius;

    for (int d=0; d < threeD_vec_size; d++)
    {
      int signum = (-eps_ < xyz[d]-origin_[d]) - (xyz[d]-origin_[d] < eps_);
      vel[d] = signum * pulsating_velocity * (xyz[d]-origin_[d]);
    }
  }

  return vel;
}

} // tioga_nalu
