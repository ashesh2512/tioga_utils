
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
    omega_ = node["omega"].as<double>();
    axis_ = node["axis"].as<std::vector<double>>();
    origin_ = node["origin"].as<std::vector<double>>();

    assert(axis_.size() == meta_.spatial_dimension());
    assert(origin_.size() == meta_.spatial_dimension());
}

void MeshRotation::build_transformation(double time)
{
  reset_trans_mat();

  // Build matrix for translating object to cartesian origin
  trans_mat_ =
    { { 1.0, 0.0, 0.0, -origin_[0] },
      { 0.0, 1.0, 0.0, -origin_[1] },
      { 0.0, 0.0, 1.0, -origin_[2] },
      { 0.0, 0.0, 0.0,  1.0        } };

  // Build matrix for rotating object
  // compute magnitude of axis around which to rotate
  double mag = 0.0;
  for (int d=0; d < meta_.spatial_dimension(); d++)
      mag += axis_[d] * axis_[d];
  mag = std::sqrt(mag);

  // build quaternion based on angle and axis of rotation
  const double angle = omega_*time;
  const double cosang = std::cos(0.5*angle);
  const double sinang = std::sin(0.5*angle);
  const double q0 = cosang;
  const double q1 = sinang * axis_[0]/mag;
  const double q2 = sinang * axis_[1]/mag;
  const double q3 = sinang * axis_[2]/mag;

  // rotation matrix based on quaternion
  std::vector<std::vector<double>> curr_trans_mat_ =
  { { q0*q0 + q1*q1 - q2*q2 - q3*q3,            2.0*(q1*q2 - q0*q3),           2.0*(q0*q2 + q1*q3), 0.0 },
    {           2.0*(q1*q2 + q0*q3),  q0*q0 - q1*q1 + q2*q2 - q3*q3,           2.0*(q2*q3 - q0*q1), 0.0 },
    {           2.0*(q1*q3 - q0*q2),            2.0*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3, 0.0 },
    {                           0.0,                            0.0,                           0.0, 1.0 } };

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
