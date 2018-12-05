
#include "MeshRotation.h"

#include <cmath>

namespace tioga_nalu {

MeshRotation::MeshRotation(
  stk::mesh::MetaData& meta,
  const YAML::Node& node )
: MotionBase(meta)
{
  load(node);
}

void MeshRotation::load(const YAML::Node& node)
{
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

  if (node["move_once"])
      move_once_ = node["move_once"].as<bool>();

  // rotation could be based on angular velocity or angle
  if(node["omega"]){
    use_omega_ = true;
    omega_ = node["omega"].as<double>();
  }
  if(node["angle"])
  {
    use_omega_ = false;
    angle_ = node["angle"].as<double>();
  }
  // ensure only 1 of omega or angle is specified
  assert((node["omega"]>eps_) + (node["angle"]>eps_) == 1);

  axis_ = node["axis"].as<std::vector<double>>();
  origin_ = node["origin"].as<std::vector<double>>();

  assert(axis_.size() == meta_.spatial_dimension());
  assert(origin_.size() == meta_.spatial_dimension());
}

void MeshRotation::build_transformation(const double time)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // if move once is specified we stop moving after start time
    if(move_once_ && has_moved_)
      return;

    // determine current angle
    double curr_angle = 0.0;
    if (use_omega_)
      curr_angle = omega_*(time-start_time_);
    else
      curr_angle = angle_*M_PI/180;

    rotation_mat(curr_angle);
    has_moved_ = true;
  }
}

void MeshRotation::rotation_mat(const double angle)
{
  reset(trans_mat_);

  // Build matrix for translating object to cartesian origin
  trans_mat_[0][3] = -origin_[0];
  trans_mat_[1][3] = -origin_[1];
  trans_mat_[2][3] = -origin_[2];

  // Build matrix for rotating object
  // compute magnitude of axis around which to rotate
  double mag = 0.0;
  for (int d=0; d < meta_.spatial_dimension(); d++)
      mag += axis_[d] * axis_[d];
  mag = std::sqrt(mag);

  // build quaternion based on angle and axis of rotation
  const double cosang = std::cos(0.5*angle);
  const double sinang = std::sin(0.5*angle);
  const double q0 = cosang;
  const double q1 = sinang * axis_[0]/mag;
  const double q2 = sinang * axis_[1]/mag;
  const double q3 = sinang * axis_[2]/mag;

  // rotation matrix based on quaternion
  trans_mat_type curr_trans_mat_ = {};
  // 1st row
  curr_trans_mat_[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
  curr_trans_mat_[0][1] = 2.0*(q1*q2 - q0*q3);
  curr_trans_mat_[0][2] = 2.0*(q0*q2 + q1*q3);
  // 2nd row
  curr_trans_mat_[1][0] = 2.0*(q1*q2 + q0*q3);
  curr_trans_mat_[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
  curr_trans_mat_[1][2] = 2.0*(q2*q3 - q0*q1);
  // 3rd row
  curr_trans_mat_[2][0] = 2.0*(q1*q3 - q0*q2);
  curr_trans_mat_[2][1] = 2.0*(q0*q1 + q2*q3);
  curr_trans_mat_[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  // 4th row
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
}

} // tioga_nalu
