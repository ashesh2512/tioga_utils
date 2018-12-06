#ifndef MESHMOTION_H
#define MESHMOTION_H

#include "MotionBase.h"

#include "stk_mesh/base/BulkData.hpp"
#include "stk_mesh/base/CoordinateSystems.hpp"
#include "stk_mesh/base/Field.hpp"

#include "yaml-cpp/yaml.h"

#include <memory>

namespace tioga_nalu {

typedef stk::mesh::Field<double, stk::mesh::Cartesian> VectorFieldType;
typedef stk::mesh::Field<double> ScalarFieldType;

class MeshMotion
{
public:
  MeshMotion(
    stk::mesh::MetaData&,
    stk::mesh::BulkData&,
    const YAML::Node&);

  ~MeshMotion() {}

  virtual void setup();

  virtual void initialize();

  virtual void execute(const int);

  int num_steps() { return numSteps_; }

  double current_time() { return currentTime_; }

private:
  MeshMotion() = delete;
  MeshMotion(const MeshMotion&) = delete;

  void load(const YAML::Node&);

  void init_coordinates();

  /**
   *  @param[in] gid       Motion group id
   *  @param[in] trans_mat Transformation matrix
   */
  void update_coordinates_velocity(
    const int gid,
    MotionBase::trans_mat_type trans_mat );

  /**
   *  @param[in] gid           Motion group id
   */
  void set_mesh_velocity(const int gid);

  stk::mesh::MetaData& meta_;

  stk::mesh::BulkData& bulk_;

  /** Motion vector
   *
   *  A two-dimensional list of size number of motion groups
   *  Each entry in the outer list is size number of motions in that group
   */
  std::vector<std::vector<std::unique_ptr<MotionBase>>> meshMotionVec_;

  /** Motion part names
   *
   *  A two-dimensional list of size number of motion groups
   *  Each entry in the outer list is size number of parts in that group
   */
  std::vector<std::vector<std::string>> partNamesVec_;

  /** Motion parts
   *
   *  A two-dimensional list of size number of motion groups
   *  Each entry in the outer list is size number of parts in that group
   */
  std::vector<stk::mesh::PartVector> partVec_;

  double startTime_{0.0};
  double deltaT_{0.0};
  double currentTime_{0.0};

  int numSteps_{0};
};

} // tioga_nalu

#endif /* MESHMOTION_H */
