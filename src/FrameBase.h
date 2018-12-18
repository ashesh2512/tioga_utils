#ifndef FRAMEBASE_H
#define FRAMEBASE_H

#include "MotionBase.h"

// stk base header files
#include "stk_mesh/base/CoordinateSystems.hpp"
#include "stk_mesh/base/BulkData.hpp"
#include "stk_mesh/base/Field.hpp"
#include "stk_mesh/base/MetaData.hpp"

namespace tioga_nalu {

typedef stk::mesh::Field<double, stk::mesh::Cartesian> VectorFieldType;
typedef stk::mesh::Field<double> ScalarFieldType;

class FrameBase
{
public:
  FrameBase(
    stk::mesh::MetaData&,
    stk::mesh::BulkData&,
    const YAML::Node&,
    bool);

  virtual ~FrameBase() {}

  void setup();

  virtual void update_coordinates_velocity(const double) = 0;

  virtual const MotionBase::trans_mat_type& get_inertial_frame() const {
    throw std::runtime_error("FrameNonInertial: Invalid access of inertial frame"); };

  void set_ref_frame( MotionBase::trans_mat_type& frame ) {
    ref_frame_ = frame; }

  const bool isInertial_;

protected:
  //! Reference to the STK Mesh MetaData object
  stk::mesh::MetaData& meta_;

  //! Reference to the STK Mesh BulkData object
  stk::mesh::BulkData& bulk_;

  /** Motion vector
   *
   *  A vector of size number of motion groups
   */
  std::vector<std::unique_ptr<MotionBase>> meshMotionVec_;

  /** Motion part names
   *
   *  A vector of size number of parts
   */
  std::vector<std::string> partNamesVec_;

  /** Motion parts
   *
   *  A vector of size number of parts
   */
  stk::mesh::PartVector partVec_;

  /** Reference frame
   *
   * A 4x4 matrix that defines the reference frame for subsequent motions
   * It is initialized to an identity matrix
   */
  MotionBase::trans_mat_type ref_frame_ = MotionBase::identity_mat_;

private:
    FrameBase() = delete;
    FrameBase(const FrameBase&) = delete;

    void load(const YAML::Node&);
};

} // tioga_nalu

#endif /* FRAMEBASE_H */
