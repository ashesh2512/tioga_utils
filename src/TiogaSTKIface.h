#ifndef TIOGASTKIFACE_H
#define TIOGASTKIFACE_H

#include "TiogaBlock.h"

#include <stk_mesh/base/MetaData.hpp>
#include <stk_mesh/base/BulkData.hpp>
#include <stk_mesh/base/Entity.hpp>
#include <stk_mesh/base/Field.hpp>
#include <stk_mesh/base/CoordinateSystems.hpp>

#include "yaml-cpp/yaml.h"

#include <vector>
#include <memory>
#include <array>

class tioga;

namespace tioga_nalu {

struct OversetInfo
{
  //! Fringe node point
  stk::mesh::Entity node_;

  //! Donor element from other mesh
  stk::mesh::Entity donorElem_;

  std::array<double,3> isoCoords_;
  std::array<double,3> nodalCoords_;
};

/** Nalu interface to TIOGA (Topology Independent Overset Grid Assembly)
 *
 *  This class provides a two-way data transfer interface for TIOGA library and
 *  provides overset connectivity capability for Nalu.
 */
class TiogaSTKIface
{
public:
  /**
   *  @param meta STK MetaData
   *  @param bulk STK BulkData
   *  @param node YAML node containing overset inputs
   */
  TiogaSTKIface(stk::mesh::MetaData&,
                stk::mesh::BulkData&,
                const YAML::Node&,
                const std::string&);

  ~TiogaSTKIface();

  /** Setup block structure information (steps before mesh creation)
   */
  void setup();

  /** Initialize mesh data structure (steps after mesh creation)
   */
  void initialize();

  /** Determine overset connectivity by calling into TIOGA API
   *
   *  This method performs several steps: updates coordinates (if necessary,
   *  during mesh motion), registers the mesh blocks to TIOGA, calculate mesh
   *  connectivity information (hole, fringe, and field point determination),
   *  update the "overset inactive part" for hole elements, create the {fringe
   *  node, donor element} mapping pair data structures for overset simulations.
   */
  void execute();

  /** Check interpolation errors from overset on a linear field function
   *
   */
  void check_soln_norm();

  /** Return the TIOGA interface object */
  tioga& tioga_iface()
  { return *tg_; }

private:
  TiogaSTKIface() = delete;
  TiogaSTKIface(const TiogaSTKIface&) = delete;

  /** Process the input parameters and initialize all data structures necessary
   * to call TIOGA.
   */
  void load(const YAML::Node&);

  /** STK Custom Ghosting to transfer donor elements to receptor's MPI rank
   *
   *  This method declares the STK custom ghosting object, actual updates are
   *  performed by the update_ghosting method.
   */
  void initialize_ghosting();

  /** Ghost donor elements to receptor MPI ranks
   */
  void update_ghosting();

  /** Populate the {fringe node, donor element} pair data structure
   */
  void update_fringe_info();

  /** Update the inactive part with hole elements
   */
  void populate_inactive_part();

  /** Reset all connectivity data structures when recomputing connectivity
   */
  void reset_data_structures();

  void get_receptor_info();

  void populate_overset_info();

  //! Reference to the STK MetaData object
  stk::mesh::MetaData& meta_;

  //! Reference to the STK BulkData object
  stk::mesh::BulkData& bulk_;

  //! List of TIOGA data structures for each mesh block participating in overset
  //! connectivity
  std::vector<std::unique_ptr<TiogaBlock>> blocks_;

  //! Reference to the TIOGA API interface
  std::unique_ptr<tioga> tg_;

  //! Pointer to STK Custom Ghosting object
  stk::mesh::Ghosting* ovsetGhosting_;

  //! Work array used to hold donor elements that require ghosting to receptor
  //! MPI ranks
  stk::mesh::EntityProcVec elemsToGhost_;

  //! Fringe {receptor, donor} information
  std::vector<std::unique_ptr<OversetInfo>> ovsetInfo_;

  //! Name of part holding hole elements
  std::string inactivePartName_;

  //! STK Part holding hole elements from overset connectivity
  stk::mesh::Part* inactivePart_;

  //! List of hole elements
  std::vector<stk::mesh::Entity> holeElems_;

  //! List of receptor nodes that are shared entities across MPI ranks. This
  //! information is used to synchronize the field vs. fringe point status for
  //! these shared nodes across processor boundaries.
  std::vector<stk::mesh::EntityId> receptorIDs_;

  //! Donor elements corresponding to TiogaSTKIface::receptorIDs_ that must be
  //! ghosted to another MPI rank to ensure that owned and shared nodes are
  //! consistent.
  std::vector<stk::mesh::EntityId> donorIDs_;

  std::string coordsName_;

  //! Set the symmetry direction for TIOGA, default is z-direction (3)
  int symmetryDir_{3};
};


}  // tioga

#endif /* TIOGASTKIFACE_H */
