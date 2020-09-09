#ifndef TIOGAAMRIFACE_H
#define TIOGAAMRIFACE_H

#include <memory>

#include "StructMesh.h"

namespace YAML {
class Node;
}

namespace TIOGA {
class tioga;
}

namespace tioga_amr {

class TiogaAMRIface
{
public:
    TiogaAMRIface();

    ~TiogaAMRIface();

    void load(const YAML::Node&);

    void initialize();

    void register_mesh(TIOGA::tioga&, const bool verbose=false);

    void register_solution(TIOGA::tioga&);

    void update_solution();

    void write_outputs(const int time_index=0, const double time=0.0);

    int num_total_vars() const { return m_ncell_vars + m_nnode_vars; }

    int num_cell_vars() const { return m_ncell_vars; }

    int num_node_vars() const { return m_nnode_vars; }

    int stk_sol() const { return m_stk_sol; }

    int amr_sol() const { return m_amr_sol; }

private:
    void init_var(Field&, const int nvars, const amrex::Real offset);

    double get_sol(const double, const double, const double, const int, const int);

    void update_solution(const bool isField);

    std::unique_ptr<StructMesh> m_mesh;

    //! Reference to cell variable field
    Field* m_qcell{nullptr};

    //! Reference to node variable field
    Field* m_qnode{nullptr};

    std::vector<int> m_ints;

    std::vector<double> m_reals;

    //! Number of ghost cells
    int m_num_ghost{3};

    //! Number of components in the cell field
    int m_ncell_vars{0};

    //! Number of components for the node field
    int m_nnode_vars{0};

    //! Integers per grid for TIOGA call
    static constexpr int ints_per_grid{10};

    //! Reals per grid for TIOGA call
    static constexpr int reals_per_grid{6};

    //! Analytical solution profile for stk meshes
    int m_stk_sol{1};

    //! Analytical solution profile for amr meshes
    int m_amr_sol{1};
};

}

#endif /* TIOGAAMRIFACE_H */
