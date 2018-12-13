
#include "MeshMotion.h"
#include "MotionPulsatingSphere.h"
#include "MotionRotation.h"
#include "MotionScaling.h"
#include "MotionTranslation.h"

// stk_mesh/base/fem
#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/FieldBLAS.hpp>

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
      if (type == "pulsating_sphere")
        meshMotionVec_[i][j].reset(new MotionPulsatingSphere(motion_def));
      else if (type == "rotation")
        meshMotionVec_[i][j].reset(new MotionRotation(motion_def));
      else if (type == "scaling")
        meshMotionVec_[i][j].reset(new MotionScaling(motion_def));
      else if (type == "translation")
        meshMotionVec_[i][j].reset(new MotionTranslation(motion_def));
      else
        throw std::runtime_error("MeshMotion: Invalid mesh motion type: " + type);

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
  VectorFieldType& mesh_velocity = meta_.declare_field<VectorFieldType>(
    stk::topology::NODE_RANK, "mesh_velocity");

  stk::mesh::put_field(coordinates, meta_.universal_part());
  stk::mesh::put_field(current_coordinates, meta_.universal_part());
  stk::mesh::put_field(mesh_displacement, meta_.universal_part());
  stk::mesh::put_field(mesh_velocity, meta_.universal_part());

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
      stk::mesh::put_field(mesh_velocity, *p);
    } // end for loop - partVec_

  } // end for loop - i index
}

void MeshMotion::initialize()
{
  currentTime_ = startTime_;

  init_coordinates();

  const int num_groups = meshMotionVec_.size();
  for (int i=0; i < num_groups; i++)
  {
    // compute composite transformation matrix
    MotionBase::trans_mat_type comp_trans_mat = meshMotionVec_[0][0]->identity_mat_;

    for (auto& mm: meshMotionVec_[i])
    {
      // perform initial motions that have been flagged so
      // also perform transient motions if t > 0.0
      if( currentTime_ > 0.0 || mm->move_once_ )
      {
        // build and get transformation matrix
        mm->build_transformation(currentTime_);

        // composite addition of motions in current group
        comp_trans_mat = mm->add_motion(mm->get_trans_mat(),comp_trans_mat);
      }
    }

    // compute velocity resulting of motions in current group
    if( currentTime_ > 0.0 )
    {
      // reset velocity field
      VectorFieldType* mesh_velocity = meta_.get_field<VectorFieldType>(
        stk::topology::NODE_RANK, "mesh_velocity");
      stk::mesh::field_fill(0.0, *mesh_velocity, stk::mesh::selectUnion(partVec_[i]));

      // update coordinates or set mesh velocity only if composite motion is not identity
      if( comp_trans_mat != meshMotionVec_[0][0]->identity_mat_ )
        update_coordinates_velocity(i, comp_trans_mat);
    }

  } // end loop - i index
}

void MeshMotion::execute(const int istep)
{
  const double curr_time = startTime_ + (istep + 1) * deltaT_;
  currentTime_ = curr_time;

  const int num_groups = meshMotionVec_.size();
  for (int i=0; i < num_groups; i++)
  {
    // compute composite transformation matrix
    MotionBase::trans_mat_type comp_trans_mat = meshMotionVec_[0][0]->identity_mat_;

    for (auto& mm: meshMotionVec_[i])
    {
        // build and get transformation matrix
        mm->build_transformation(currentTime_);

        // composite addition of motions in current group
        comp_trans_mat = mm->add_motion(mm->get_trans_mat(),comp_trans_mat);
    }

    // reset velocity field
    VectorFieldType* mesh_velocity = meta_.get_field<VectorFieldType>(
      stk::topology::NODE_RANK, "mesh_velocity");
    stk::mesh::field_fill(0.0, *mesh_velocity, stk::mesh::selectUnion(partVec_[i]));

    // update coordinates or set mesh velocity only if composite motion is not identity
    if( comp_trans_mat != meshMotionVec_[0][0]->identity_mat_ )
      update_coordinates_velocity(i, comp_trans_mat);

  } // end loop - i index
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

void MeshMotion::update_coordinates_velocity(
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
  VectorFieldType* meshVelocity = meta_.get_field<VectorFieldType>(
    stk::topology::NODE_RANK, "mesh_velocity");

  stk::mesh::Selector sel = stk::mesh::selectUnion(partVec_[gid]);
  const auto& bkts = bulk_.get_buckets(stk::topology::NODE_RANK, sel);

  for (auto b: bkts) {
    for (size_t in=0; in < b->size(); in++) {

      auto node = (*b)[in]; // mesh node and NOT YAML node
      double* oldxyz = stk::mesh::field_data(*modelCoords, node);
      double* xyz = stk::mesh::field_data(*currCoords, node);
      double* dx = stk::mesh::field_data(*displacement, node);
      double* velxyz = stk::mesh::field_data(*meshVelocity, node);

      // perform matrix multiplication between transformation matrix
      // and old coordinates to obtain current coordinates
      for (int d = 0; d < ndim; d++) {
        xyz[d] = trans_mat[d][0]*oldxyz[0]
                +trans_mat[d][1]*oldxyz[1]
                +trans_mat[d][2]*oldxyz[2]
                +trans_mat[d][3];

        dx[d] = xyz[d] - oldxyz[d];
      } // end for loop - d index

      // compute velocity vector on current node resulting from all
      // motions in current motion group
      for (auto& mm: meshMotionVec_[gid])
      {
        if( !mm->move_once_ )
        {
          MotionBase::threeD_vec_type mm_vel = mm->compute_velocity(currentTime_,trans_mat,xyz);

          for (int d = 0; d < ndim; d++)
            velxyz[d] += mm_vel[d];
        }
      } // end for loop - mm

    } // end for loop - in index
  } // end for loop - bkts
}

} // tioga_nalu
