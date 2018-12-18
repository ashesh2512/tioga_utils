#ifndef FRAMEINERTIAL_H
#define FRAMEINERTIAL_H

#include "FrameBase.h"

#include "yaml-cpp/yaml.h"

#include <cassert>
#include <float.h>

namespace tioga_nalu {

class FrameInertial : public FrameBase
{
public:
  FrameInertial(
    stk::mesh::MetaData& meta,
    stk::mesh::BulkData& bulk,
    const YAML::Node& node
) : FrameBase(meta,bulk,node,true) {}

  virtual ~FrameInertial() {}

  void update_coordinates_velocity(const double time);

  const MotionBase::trans_mat_type& get_inertial_frame() const {
    return inertial_frame_; }

private:
    FrameInertial() = delete;
    FrameInertial(const FrameInertial&) = delete;

    void compute_transformation(const double);

    /** Inertial frame
     *
     * A 4x4 matrix that defines the composite inertial frame
     * It is initialized to an identity matrix
     */
    MotionBase::trans_mat_type inertial_frame_ = MotionBase::identity_mat_;
};

} // tioga_nalu

#endif /* FRAMEINERTIAL_H */
