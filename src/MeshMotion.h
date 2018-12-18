#ifndef MESHMOTION_H
#define MESHMOTION_H

#include "FrameBase.h"

namespace tioga_nalu {

class MeshMotion
{
public:
  MeshMotion(
    stk::mesh::MetaData&,
    stk::mesh::BulkData&,
    const YAML::Node&);

  ~MeshMotion() {}

  void setup();

  void initialize();

  void execute(const int);

  int num_steps() { return numSteps_; }

  double current_time() { return currentTime_; }

private:
  MeshMotion() = delete;
  MeshMotion(const MeshMotion&) = delete;

  void load(const YAML::Node&);

  void init_coordinates();

  /**
   *  @param[in] gid Motion group id
   */
  void set_mesh_velocity(const int gid);

  //! Reference to the STK Mesh MetaData object
  stk::mesh::MetaData& meta_;

  //! Reference to the STK Mesh BulkData object
  stk::mesh::BulkData& bulk_;

  /** Motion frame vector
   *
   *  Vector of type of frame of corresponding motion
   *  Size is the number of motion groups in input file
   */
  std::vector<std::unique_ptr<FrameBase>> frameVec_;

  /** Reference frame map
   *
   *  Map between frame indices and corresponding reference frame indices
   *  Size is the number of motion groups with reference frames
   */
  std::map<int, int> refFrameMap_;

  double startTime_{0.0};
  double deltaT_{0.0};
  double currentTime_{0.0};

  int numSteps_{0};
};

} // tioga_nalu

#endif /* MESHMOTION_H */
