
#include "MeshMotion.h"

#include "FrameInertial.h"
#include "FrameNonInertial.h"

#include <cassert>

namespace tioga_nalu {

MeshMotion::MeshMotion(
  stk::mesh::MetaData& meta,
  stk::mesh::BulkData& bulk,
  const YAML::Node& node
) : meta_(meta),
    bulk_(bulk)
{
  if( meta_.spatial_dimension() != 3 )
    throw std::runtime_error("MeshMotion: Mesh motion is set up for only 3D meshes");

  load(node);
}

void MeshMotion::load(const YAML::Node& node)
{
  // get motion information for entire mesh
  const auto& minfo = node["motion_group"];
  const int num_groups = minfo.size();
  frameVec_.resize(num_groups);

  // temporary vector to store frame names
  std::vector<std::string> frameNames(num_groups);

  for (int i=0; i < num_groups; i++) {

    // extract current motion group info
    const auto& ginfo = minfo[i];

    // get name of motion group
    frameNames[i] = ginfo["name"].as<std::string>();

    // get frame definition of motion group
    std::string frame = ginfo["frame"].as<std::string>();

    if( frame == "inertial" )
      frameVec_[i].reset(new FrameInertial(meta_, bulk_, ginfo));
    else if( frame == "non_inertial" )
      frameVec_[i].reset(new FrameNonInertial(meta_, bulk_, ginfo));
    else
      throw std::runtime_error("MeshMotion: Invalid frame type: " + frame);

    // get the reference frame if it exists
    if(ginfo["reference"])
    {
      std::string refFrameName = ginfo["reference"].as<std::string>();
      auto it = std::find(frameNames.begin(), frameNames.end(), refFrameName);
      refFrameMap_[i] = std::distance(frameNames.begin(), it);
    }
  }

  if (node["start_time"])
    startTime_ = node["start_time"].as<double>();

  numSteps_ = node["num_time_steps"].as<int>();
  deltaT_ = node["delta_t"].as<double>();
}

void MeshMotion::setup()
{
  VectorFieldType& coordinates = meta_.declare_field<VectorFieldType>(
    stk::topology::NODE_RANK, "coordinates");
  VectorFieldType& current_coordinates = meta_.declare_field<VectorFieldType>(
    stk::topology::NODE_RANK, "current_coordinates");
  VectorFieldType& mesh_displacement = meta_.declare_field<VectorFieldType>(
    stk::topology::NODE_RANK, "mesh_displacement");
  VectorFieldType& mesh_velocity = meta_.declare_field<VectorFieldType>(
    stk::topology::NODE_RANK, "mesh_velocity");

  stk::mesh::put_field(coordinates, meta_.universal_part());
  stk::mesh::put_field(current_coordinates, meta_.universal_part());
  stk::mesh::put_field(mesh_displacement, meta_.universal_part());
  stk::mesh::put_field(mesh_velocity, meta_.universal_part());

  // setup the discretization for all motion frames
  for (auto& frame: frameVec_)
    frame->setup();
}

void MeshMotion::initialize()
{
  currentTime_ = startTime_;

  init_coordinates();

  for (int i=0; i < frameVec_.size(); i++)
  {
    // set reference frame if they exist
    if( refFrameMap_.find(i) != refFrameMap_.end() )
    {
      int ref_ind = refFrameMap_[i];
      MotionBase::trans_mat_type ref_frame = frameVec_[ref_ind]->get_inertial_frame();
      frameVec_[i]->set_ref_frame(ref_frame);
    }

    if( ( frameVec_[i]->isInertial_ ) ||
        (!frameVec_[i]->isInertial_ && currentTime_ > 0.0) )
      frameVec_[i]->update_coordinates_velocity(currentTime_);
  }
}

void MeshMotion::init_coordinates()
{
  const int ndim = meta_.spatial_dimension();
  VectorFieldType* modelCoords = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "coordinates");
  VectorFieldType* currCoords = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "current_coordinates");

  stk::mesh::Selector sel = meta_.universal_part();
  const auto& bkts = bulk_.get_buckets(stk::topology::NODE_RANK, sel);

  for (auto b: bkts) {
    for (size_t in=0; in < b->size(); in++) {
      auto node = (*b)[in];
      double* oldxyz = stk::mesh::field_data(*modelCoords, node);
      double* xyz = stk::mesh::field_data(*currCoords, node);

      for (int d=0; d < ndim; d++)
        xyz[d] = oldxyz[d];
    }
  }
}

void MeshMotion::execute(const int istep)
{
  const double curr_time = startTime_ + (istep + 1) * deltaT_;
  currentTime_ = curr_time;

  for (int i=0; i < frameVec_.size(); i++)
    if( !frameVec_[i]->isInertial_ )
      frameVec_[i]->update_coordinates_velocity(currentTime_);
}

} // tioga_nalu
