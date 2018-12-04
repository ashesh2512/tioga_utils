
#include "MeshMotion.h"
#include "MeshRotation.h"
#include "MeshScaling.h"
#include "MeshTranslation.h"

#include "stk_mesh/base/Field.hpp"

#include <cassert>

namespace tioga_nalu {

MeshMotion::MeshMotion(
  stk::mesh::MetaData& meta,
  stk::mesh::BulkData& bulk,
  const YAML::Node& node
) : meta_(meta),
    bulk_(bulk)
{
  load(node);
}

void MeshMotion::load(const YAML::Node& node)
{
  const auto& minfo = node["motion_group"];

  const int num_groups = minfo.size();
  meshMotionVec_.resize(num_groups);
  partNamesVec_.resize(num_groups);

  for (int i=0; i < num_groups; i++) {

    // extract current motion group info
    const auto& ginfo = minfo[i];

    // extract the motions in the current group
    const auto& motions = ginfo["motion"];

    const int num_motions = motions.size();
    meshMotionVec_[i].resize(num_motions);

    // create the classes associated with every motion in current group
    for (int j=0; j < num_motions; j++) {

      // get the motion definition for j-th transformation
      const auto& motion_def = motions[j];

      // motion type should always be defined by the user
      std::string type = motion_def["type"].as<std::string>();

      // determine type of mesh motion based on user definition in input file
      if (type == "rotation") {
        meshMotionVec_[i][j].reset(new MeshRotation(meta_,motion_def));
      }
      else if (type == "scaling") {
        meshMotionVec_[i][j].reset(new MeshScaling(meta_,motion_def));
      }
      else if (type == "translation") {
        meshMotionVec_[i][j].reset(new MeshTranslation(meta_,motion_def));
      }
      else {
        throw std::runtime_error("MeshMotion: Invalid mesh motion type: " + type);
      } // end if

    } // end for loop - j index

    // get part names associated with current motion group
    const auto& fparts = ginfo["mesh_parts"];
    partNamesVec_[i] = fparts.as<std::vector<std::string>>();

  } // end for loop - i index

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

  stk::mesh::put_field(coordinates, meta_.universal_part());
  stk::mesh::put_field(current_coordinates, meta_.universal_part());
  stk::mesh::put_field(mesh_displacement, meta_.universal_part());

  const int num_groups = partNamesVec_.size();
  partVec_.resize(num_groups);

  for (int i=0; i < num_groups; i++) {

    for (auto pName: partNamesVec_[i]) {
      stk::mesh::Part* part = meta_.get_part(pName);
      if (nullptr == part)
        throw std::runtime_error(
          "MeshMotion: Invalid part name encountered: " + pName);
      else
        partVec_[i].push_back(part);
    } // end for loop - partNamesVec_

    for (auto* p: partVec_[i]) {
      stk::mesh::put_field(coordinates, *p);
      stk::mesh::put_field(current_coordinates, *p);
      stk::mesh::put_field(mesh_displacement, *p);
    } // end for loop - partVec_

  } // end for loop - i index
}

void MeshMotion::initialize()
{
  init_coordinates();

  if (startTime_ > 0.0)
  {
    const int num_groups = meshMotionVec_.size();
    const int mat_size = MotionBase::trans_mat_size;

    for (int i=0; i < num_groups; i++)
    {
      // initialize composite transformation matrix to be an identity matrix
      MotionBase::trans_mat_type comp_trans_mat_ = {};
      for(int in = 0; in < mat_size; in++)
        comp_trans_mat_[in][in] = 1.0;

      for (auto& mm: meshMotionVec_[i])
      {
        // build and get transformation matrix
        mm->build_transformation(startTime_);
        const MotionBase::trans_mat_type& curr_trans_mat_ = mm->get_trans_mat();

        // composite addition of motions in current group
        comp_trans_mat_ = mm->add_motion(curr_trans_mat_,comp_trans_mat_);
      }
      update_coordinates( i, comp_trans_mat_ );
    }
  }
}

void MeshMotion::execute(const int istep)
{
  const double curr_time = startTime_ + (istep + 1) * deltaT_;
  currentTime_ = curr_time;

  const int num_groups = meshMotionVec_.size();
  const int mat_size = MotionBase::trans_mat_size;

  for (int i=0; i < num_groups; i++)
  {
    // initialize composite transformation matrix to be an identity matrix
    MotionBase::trans_mat_type comp_trans_mat_ = {};
    for(int in = 0; in < mat_size; in++)
      comp_trans_mat_[in][in] = 1.0;

    for (auto& mm: meshMotionVec_[i])
    {
      // build and get transformation matrix
      mm->build_transformation(currentTime_);
      const MotionBase::trans_mat_type& curr_trans_mat_ = mm->get_trans_mat();

      // composite addition of motions in current group
      comp_trans_mat_ = mm->add_motion(curr_trans_mat_,comp_trans_mat_);
    }
    update_coordinates( i, comp_trans_mat_ );
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

void MeshMotion::update_coordinates(
  const int gid,
  MotionBase::trans_mat_type trans_mat )
{
  const int ndim = meta_.spatial_dimension();
  VectorFieldType* modelCoords = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "coordinates");
  VectorFieldType* currCoords = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "current_coordinates");
  VectorFieldType* displacement = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "mesh_displacement");

  stk::mesh::Selector sel = stk::mesh::selectUnion(partVec_[gid]);
  const auto& bkts = bulk_.get_buckets(stk::topology::NODE_RANK, sel);

  for (auto b: bkts) {
    for (size_t in=0; in < b->size(); in++) {

      auto node = (*b)[in]; // mesh node and not YAML node
      double* oldxyz = stk::mesh::field_data(*modelCoords, node);
      double* xyz = stk::mesh::field_data(*currCoords, node);
      double* dx = stk::mesh::field_data(*displacement, node);

      // perform matrix multiplication between transformation matrix
      // and old coordinates.
      for (int d = 0; d < ndim; d++) {
        xyz[d] = trans_mat[d][0]*oldxyz[0]
                +trans_mat[d][1]*oldxyz[1]
                +trans_mat[d][2]*oldxyz[2]
                +trans_mat[d][3];

      dx[d] = xyz[d] - oldxyz[d];
      } // end for loop - d index
    } // end for loop - in index
  } // end for loop - bkts
}

} // tioga_nalu
