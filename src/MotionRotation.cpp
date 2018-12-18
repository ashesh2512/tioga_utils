
#include "MotionRotation.h"

#include <cmath>

namespace tioga_nalu {

MotionRotation::MotionRotation(const YAML::Node& node)
  : MotionBase()
{
  load(node);
}

void MotionRotation::load(const YAML::Node& node)
{
  if(node["start_time"])
    start_time_ = node["start_time"].as<double>();

  if(node["end_time"])
    end_time_ = node["end_time"].as<double>();

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

  axis_ = node["axis"].as<threeD_vec_type>();
  origin_ = node["origin"].as<threeD_vec_type>();

  assert(axis_.size() == threeD_vec_size);
  assert(origin_.size() == threeD_vec_size);
}

void MotionRotation::build_transformation(
  const double time,
  const double* xyz)
{
  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // determine current angle
    double curr_angle = 0.0;
    if (use_omega_)
      curr_angle = omega_*(time-start_time_);
    else
      curr_angle = angle_*M_PI/180;

    rotation_mat(curr_angle);
  }
}

void MotionRotation::rotation_mat(const double angle)
{
  reset_mat(trans_mat_);

  // Build matrix for translating object to cartesian origin
  trans_mat_[0][3] = -origin_[0];
  trans_mat_[1][3] = -origin_[1];
  trans_mat_[2][3] = -origin_[2];

  // Build matrix for rotating object
  // compute magnitude of axis around which to rotate
  double mag = 0.0;
  for (int d=0; d < threeD_vec_size; d++)
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
  reset_mat(curr_trans_mat_);
  curr_trans_mat_[0][3] = origin_[0];
  curr_trans_mat_[1][3] = origin_[1];
  curr_trans_mat_[2][3] = origin_[2];

  // composite addition of motions
  trans_mat_ = add_motion(curr_trans_mat_,trans_mat_);
}

MotionBase::threeD_vec_type MotionRotation::compute_velocity(
  double time,
  const trans_mat_type& comp_trans,
  double* xyz )
{
  threeD_vec_type vel = {};

  if( (time >= (start_time_-eps_)) && (time <= (end_time_+eps_)) )
  {
    // construct unit vector
    threeD_vec_type unitVec = {};

    double mag = 0.0;
    for (int d=0; d < threeD_vec_size; d++)
      mag += axis_[d] * axis_[d];
    mag = std::sqrt(mag);

    unitVec[0] = axis_[0]/mag;
    unitVec[1] = axis_[1]/mag;
    unitVec[2] = axis_[2]/mag;

    // transform the origin of the rotating body
    threeD_vec_type trans_origin = {};
    for (int d = 0; d < threeD_vec_size; d++) {
      trans_origin[d] = comp_trans[d][0]*origin_[0]
                       +comp_trans[d][1]*origin_[1]
                       +comp_trans[d][2]*origin_[2]
                       +comp_trans[d][3];
    }

    // compute relative coords and vector omega (dimension 3) for general cross product
    threeD_vec_type relCoord = {};
    threeD_vec_type vecOmega = {};
    for (int d=0; d < threeD_vec_size; d++) {
      relCoord[d] = xyz[d] - trans_origin[d];
      vecOmega[d] = omega_*unitVec[d];
    }

    // cross product v = \omega \cross \x
    vel[0] = vecOmega[1]*relCoord[2] - vecOmega[2]*relCoord[1];
    vel[1] = vecOmega[2]*relCoord[0] - vecOmega[0]*relCoord[2];
    vel[2] = vecOmega[0]*relCoord[1] - vecOmega[1]*relCoord[0];
  }

  return vel;
}

} // tioga_nalu
